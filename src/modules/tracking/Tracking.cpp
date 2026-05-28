/**
 * tracking — visual-servo tracking mode for PX4 (fixed-wing + multicopter).
 *
 * Port of satria-firmware ModeTracking (ArduPlane) to PX4, extended to
 * cover multicopter airframes at runtime.
 *
 * Design
 * ──────
 * The companion computer (seeker/tracker) sends TRACKING_MESSAGE (MAVLink
 * ID 11045) with normalised errors errorx/errory ∈ [-1, 1].  The MAVLink
 * receiver decodes the message and publishes it on the 'tracking_message'
 * uORB topic.  This module subscribes to that topic, applies deadband,
 * settle-ramp, and PID controllers, then publishes vehicle_attitude_setpoint.
 *
 * Vehicle-type-aware publish path
 * ───────────────────────────────
 *   FIXED_WING : thrust_body[0] = throttle/100   (forward body axis)
 *                throttle PID input = pitch error (pitch-for-airspeed)
 *
 *   ROTARY_WING: thrust_body[2] = -throttle/100  (NED, negative = up)
 *                throttle PID input = altitude error vs TRK_TGT_ALT
 *                (pitch-for-airspeed makes no sense on a copter;
 *                 throttle holds altitude while roll/pitch chase target)
 *
 * Activation
 * ──────────
 * The module publishes setpoints only when
 *   vehicle_status.nav_state == NAVIGATION_STATE_EXTERNAL1  (= 23)
 * which is the PX4 "external 1" custom mode.  The GCS switches to this
 * state via MAV_CMD_DO_SET_MODE with custom_mode encoding EXTERNAL1.
 *
 * Axes (same sign convention as satria-firmware)
 * ─────────────────────────────────────────────
 *   errorx > 0  target to the right  → roll right  (positive bank)
 *   errory > 0  target above centre  → pitch up     (positive nav_pitch)
 *
 * Additional features vs. base port
 * ──────────────────────────────────
 *   • GPS-based horizontal distance to TRK_TGT_LAT/LON
 *   • Altitude offset relative to TRK_TGT_ALT
 *   • Proximity alert: GCS notification on enter/exit TRK_CLOSE_M radius
 *   • Rate-limited 1 Hz telemetry matching satria GCS messages
 *   • Configurable cruise throttle baseline via TRK_CRUISE_THR
 *   • Throttle constrained to [3/4 × cruise, 7/5 × cruise] (satria convention)
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/pid/PID.hpp>
#include <mathlib/mathlib.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/tracking_message.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

// External-mode registration (lets PX4 commander show TRACKING as a first-class
// flight mode, the same way ArduPlane exposes its TRACKING custom mode).
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/vehicle_control_mode.h>

using namespace time_literals;

class Tracking : public ModuleBase, public ModuleParams
{
public:
	static Descriptor desc;

	Tracking() : ModuleParams(nullptr) { reset_pids(); }

	~Tracking() override = default;

	static int task_spawn(int argc, char *argv[]);
	static Tracking *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int run_trampoline(int argc, char *argv[]);

	void run() override;

private:
	/* ── subscriptions ─────────────────────────────────────────── */
	uORB::Subscription _tracking_msg_sub{ORB_ID(tracking_message)};
	uORB::Subscription _vehicle_att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_vel_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _hover_thrust_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _thrust_sp_sub{ORB_ID(vehicle_thrust_setpoint)};

	/* ── publications ──────────────────────────────────────────── */
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};

	/* ── external-mode registration ────────────────────────────── */
	uORB::Publication<register_ext_component_request_s> _register_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s>       _unregister_pub      {ORB_ID(unregister_ext_component)};
	uORB::Publication<arming_check_reply_s>             _arming_reply_pub    {ORB_ID(arming_check_reply)};
	uORB::Publication<vehicle_control_mode_s>           _config_control_pub  {ORB_ID(config_control_setpoints)};
	uORB::Subscription _register_reply_sub  {ORB_ID(register_ext_component_reply)};
	uORB::Subscription _arming_request_sub  {ORB_ID(arming_check_request)};

	bool   _sent_mode_registration{false};
	uint8_t _mode_request_id{179};   // arbitrary tag (any uint8 value is fine)
	int8_t _arming_check_id{-1};
	int8_t _mode_id{-1};             // dynamically-assigned nav_state for TRACKING

	void register_tracking_mode();
	void unregister_tracking_mode();
	void configure_tracking_mode();
	void reply_to_arming_check(int8_t request_id);
	void check_mode_registration();

	/* ── PID controllers ───────────────────────────────────────── */
	PID _pid_roll;
	PID _pid_pitch;
	PID _pid_throttle;

	/* ── state ─────────────────────────────────────────────────── */
	float       _errorx_rad{0.f};
	float       _errory_rad{0.f};
	hrt_abstime _mode_entry_us{0};
	hrt_abstime _last_msg_us{0};
	hrt_abstime _last_telem_us{0};
	bool        _was_active{false};
	bool        _close_prev{false};

	// Throttle baseline captured at the instant TRACKING becomes active.
	// Read once from vehicle_thrust_setpoint.xyz[2] (the body-Z thrust the
	// previous controller was commanding) so our publish doesn't introduce
	// a step in throttle. Held constant for the duration of TRACKING; the
	// throttle-PID then adds a small correction on top of this base.
	float       _entry_thrust_base{0.5f};

	/* ── parameters ────────────────────────────────────────────── */
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TRK_MAX_DEG>)    _p_max_deg,
		(ParamFloat<px4::params::TRK_DBAND>)      _p_dband,
		(ParamFloat<px4::params::TRK_SETTLE_S>)   _p_settle_s,
		(ParamFloat<px4::params::TRK_PTCH_OFS>)   _p_pitch_ofs,
		(ParamInt<px4::params::TRK_TIMEOUT>)      _p_timeout_ms,
		(ParamFloat<px4::params::TRK_CRUISE_THR>) _p_cruise_thr,
		(ParamFloat<px4::params::TRK_TGT_LAT>)    _p_tgt_lat,
		(ParamFloat<px4::params::TRK_TGT_LON>)    _p_tgt_lon,
		(ParamFloat<px4::params::TRK_TGT_ALT>)    _p_tgt_alt,
		(ParamFloat<px4::params::TRK_CLOSE_M>)    _p_close_m,
		(ParamFloat<px4::params::TRK_ROLL_P>)     _p_roll_p,
		(ParamFloat<px4::params::TRK_ROLL_I>)     _p_roll_i,
		(ParamFloat<px4::params::TRK_ROLL_D>)     _p_roll_d,
		(ParamFloat<px4::params::TRK_PTCH_P>)     _p_ptch_p,
		(ParamFloat<px4::params::TRK_PTCH_I>)     _p_ptch_i,
		(ParamFloat<px4::params::TRK_PTCH_D>)     _p_ptch_d,
		(ParamFloat<px4::params::TRK_THRO_P>)     _p_thro_p,
		(ParamFloat<px4::params::TRK_THRO_I>)     _p_thro_i,
		(ParamFloat<px4::params::TRK_THRO_D>)     _p_thro_d,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _p_tilt_max
	)

	void reset_pids();
	void update_pid_gains();
};

/* ── External-mode registration helpers ──────────────────────────────────────
 *
 * Mirrors the pattern in mc_nn_control.cpp. At startup we publish a
 * register_ext_component_request announcing this mode; the commander replies
 * with a dynamically-assigned mode_id (the nav_state value the mode will
 * appear as — normally NAVIGATION_STATE_EXTERNAL1 = 23 when we are first to
 * register). After that we listen for arming_check_request and reply so the
 * commander knows we are healthy enough to be selected.
 */

void Tracking::register_tracking_mode()
{
	register_ext_component_request_s req{};
	req.timestamp = hrt_absolute_time();
	// Name appears in QGC mode picker. Keep under sizeof(req.name)-1.
	strncpy(req.name, "Tracking", sizeof(req.name) - 1);
	req.request_id           = _mode_request_id;
	req.px4_ros2_api_version = 1;
	req.register_arming_check = true;
	req.register_mode         = true;
	_register_request_pub.publish(req);
}

void Tracking::unregister_tracking_mode()
{
	unregister_ext_component_s msg{};
	msg.timestamp = hrt_absolute_time();
	strncpy(msg.name, "Tracking", sizeof(msg.name) - 1);
	msg.arming_check_id = _arming_check_id;
	msg.mode_id         = _mode_id;
	_unregister_pub.publish(msg);
}

void Tracking::configure_tracking_mode()
{
	// Declare which controllers we drive. We publish vehicle_attitude_setpoint
	// (q_d quaternion + thrust_body), so the attitude and rate controllers must
	// run to convert that into actuator commands — these were the critical
	// missing flags. We bypass the position controller and altitude controller
	// entirely (we drive thrust ourselves), and disable manual and offboard.
	vehicle_control_mode_s cfg{};
	cfg.timestamp = hrt_absolute_time();
	cfg.source_id = _mode_id;
	cfg.flag_multicopter_position_control_enabled = false;
	cfg.flag_control_manual_enabled               = false;
	cfg.flag_control_offboard_enabled             = false;
	cfg.flag_control_position_enabled             = false;
	cfg.flag_control_velocity_enabled             = false;
	cfg.flag_control_altitude_enabled             = false;
	cfg.flag_control_climb_rate_enabled           = false;
	cfg.flag_control_acceleration_enabled         = false;
	cfg.flag_control_attitude_enabled             = true;   // run mc_att_control
	cfg.flag_control_rates_enabled                = true;   // run mc_rate_control
	cfg.flag_control_allocation_enabled           = true;
	cfg.flag_control_termination_enabled          = true;
	_config_control_pub.publish(cfg);
}

void Tracking::reply_to_arming_check(int8_t request_id)
{
	arming_check_reply_s reply{};
	reply.timestamp = hrt_absolute_time();
	reply.request_id            = request_id;
	reply.registration_id       = _arming_check_id;
	reply.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
	reply.num_events            = 0;
	reply.can_arm_and_run       = true;
	// Sensor requirements for our control law:
	reply.mode_req_angular_velocity = true;
	reply.mode_req_attitude         = true;
	reply.mode_req_local_position   = false;
	reply.mode_req_local_alt        = true;
	reply.mode_req_global_position  = true;  // we read vehicle_global_position
	reply.mode_req_home_position    = false;
	reply.mode_req_mission          = false;
	reply.mode_req_prevent_arming   = false;
	reply.mode_req_manual_control   = false;
	_arming_reply_pub.publish(reply);
}

void Tracking::check_mode_registration()
{
	register_ext_component_reply_s reply{};
	int tries = reply.ORB_QUEUE_LENGTH;

	while (_register_reply_sub.update(&reply) && --tries >= 0) {
		if (reply.request_id == _mode_request_id && reply.success) {
			_arming_check_id = reply.arming_check_id;
			_mode_id         = reply.mode_id;

			/* Translate the assigned nav_state into the (main, sub) tuple
			 * a GCS / companion needs to send via MAV_CMD_DO_SET_MODE to
			 * activate this mode. External-mode nav_states 23..30 map to
			 * AUTO sub-modes EXTERNAL1..EXTERNAL8 (= 11..18). Print this
			 * explicitly so drone-seeker / QGC don't have to guess the
			 * slot — relevant since Guided Course (upstream) also
			 * registers as an external mode and may take slot 1. */
			const int external_idx = (int)_mode_id - 23;  /* 0..7 */
			const int main_mode    = 4;                   /* PX4_CUSTOM_MAIN_MODE_AUTO */
			const int sub_mode     = (external_idx >= 0 && external_idx < 8)
						 ? (11 + external_idx)
						 : -1;
			const uint32_t custom_mode = (sub_mode > 0)
						     ? (((uint32_t)sub_mode) << 24) | (((uint32_t)main_mode) << 16)
						     : 0u;

			PX4_INFO("tracking: registered  arming_check_id=%d  mode_id=%d (nav_state)  "
				 "→ DO_SET_MODE main=%d sub=%d  custom_mode=0x%08x",
				 (int)_arming_check_id, (int)_mode_id,
				 main_mode, sub_mode, (unsigned)custom_mode);
			configure_tracking_mode();
			break;
		}
	}
}

void Tracking::reset_pids()
{
	_pid_roll.resetIntegral();
	_pid_roll.resetDerivative();
	_pid_pitch.resetIntegral();
	_pid_pitch.resetDerivative();
	_pid_throttle.resetIntegral();
	_pid_throttle.resetDerivative();
	_errorx_rad = 0.f;
	_errory_rad = 0.f;
}

void Tracking::update_pid_gains()
{
	_pid_roll.setGains(_p_roll_p.get(), _p_roll_i.get(), _p_roll_d.get());
	_pid_roll.setOutputLimit(math::radians(45.f));   /* ±45 deg bank limit */
	_pid_roll.setIntegralLimit(math::radians(20.f));

	_pid_pitch.setGains(_p_ptch_p.get(), _p_ptch_i.get(), _p_ptch_d.get());
	_pid_pitch.setOutputLimit(math::radians(45.f));   /* matched to roll; needs MPC_TILTMAX_AIR ≥ 45 */
	_pid_pitch.setIntegralLimit(math::radians(20.f));

	_pid_throttle.setGains(_p_thro_p.get(), _p_thro_i.get(), _p_thro_d.get());
	_pid_throttle.setOutputLimit(50.f);              /* ±50 % throttle change */
	_pid_throttle.setIntegralLimit(25.f);
}

void Tracking::run()
{
	update_pid_gains();
	PX4_INFO("tracking: started");

	while (!should_exit()) {

		px4_usleep(5000);   /* 5 ms poll interval → 200 Hz */

		/* Register the mode with commander on first tick, then wait for the
		 * assigned mode_id before doing anything else. Without this, the
		 * commander rejects mode switches to EXTERNAL1 with
		 * "Mode is not registered". */
		if (!_sent_mode_registration) {
			register_tracking_mode();
			_sent_mode_registration = true;
			continue;
		}

		if (_mode_id < 0 || _arming_check_id < 0) {
			check_mode_registration();
			continue;
		}

		/* Reply to commander's arming check polls so we remain selectable. */
		if (_arming_request_sub.updated()) {
			arming_check_request_s req{};
			_arming_request_sub.copy(&req);
			reply_to_arming_check(req.request_id);
		}

		/* update vehicle status */
		vehicle_status_s status{};
		_vehicle_status_sub.copy(&status);

		// _mode_id is the dynamically-assigned nav_state for our mode (= 23
		// = NAVIGATION_STATE_EXTERNAL1 when we are the first external mode).
		const bool active = (status.nav_state == _mode_id);

		/* ── on mode entry ─────────────────────────────────── */
		if (active && !_was_active) {
			reset_pids();
			update_pid_gains();
			_mode_entry_us = hrt_absolute_time();
			_last_telem_us = 0;
			_close_prev    = false;
			// Capture the throttle the previous controller (mc_pos_control or
			// fw equivalent) was commanding RIGHT NOW. This becomes our base
			// for the duration of TRACKING — a snapshot, not a live track,
			// since once we start publishing vehicle_attitude_setpoint the
			// downstream thrust setpoint will reflect OUR commands.
			vehicle_thrust_setpoint_s ts{};

			if (_thrust_sp_sub.copy(&ts)) {
				// xyz[2] is body-Z thrust (negative = up in FRD on copter).
				const float magnitude = fabsf(ts.xyz[2]);

				if (magnitude > 0.05f && magnitude < 1.0f) {
					_entry_thrust_base = magnitude;
				}
			}

			PX4_INFO("tracking: TRACKING active  base_thrust=%.0f%%",
				 (double)(_entry_thrust_base * 100.f));
			// Re-publish the control-mode config so its timestamp is fresh.
			// Commander rejects config_control_setpoints older than 10 ms at
			// the activation instant and silently falls back to default
			// (controllers disabled) — the one-shot publish in
			// check_mode_registration() happens at boot, so by the time the
			// user actually switches to TRACKING it's far too stale.
			configure_tracking_mode();
		}

		/* ── on mode exit ──────────────────────────────────── */
		if (!active && _was_active) {
			reset_pids();
			PX4_INFO("tracking: TRACKING exit");
		}

		_was_active = active;

		if (!active) {
			continue;
		}

		/* ── 1-Hz receive diagnostic (TRACKING-active only) ───────────
		 * Detect new tracking_message arrivals using updated() and
		 * print a per-second summary. updated() is peek-only — the
		 * actual consume happens in the copy() inside the PID block
		 * below; calling update() here would steal the message.
		 */
		{
			static hrt_abstime diag_last_us = 0;
			static uint32_t    diag_count   = 0;
			tracking_message_s diag_trk{};

			if (_tracking_msg_sub.updated()) {
				diag_count++;
				_tracking_msg_sub.copy(&diag_trk);   /* non-consuming peek */
			}

			const hrt_abstime now_diag = hrt_absolute_time();

			if (now_diag - diag_last_us >= 1_s) {
				PX4_INFO("TRK rx: %u msg/s  last_ex=%+.3f last_ey=%+.3f",
					 (unsigned)diag_count,
					 (double)diag_trk.errorx, (double)diag_trk.errory);
				diag_last_us = now_diag;
				diag_count = 0;
			}
		}

		const hrt_abstime now = hrt_absolute_time();

		// Periodic re-publish of config_control_setpoints at ~1 Hz while
		// active. Defensive against commander re-evaluating mode flags on
		// transitions we may have missed (re-arm, brief failsafe trip, etc.)
		static hrt_abstime last_cfg_refresh_us = 0;

		if (now - last_cfg_refresh_us > 1_s) {
			configure_tracking_mode();
			last_cfg_refresh_us = now;
		}

		/* ── consume tracking_message ────────────────────────────────
		 * Gate the PID + setpoint publish on new-message arrivals.
		 * The poll loop runs at 200 Hz (5 ms) but drone-seeker only
		 * pushes TRACKING_MESSAGE at the camera frame rate (~30 Hz).
		 * If we ran the PID every 5 ms with hardcoded dt=0.005, the
		 * I-term would integrate the SAME stale error 6× per real
		 * sample → throttle drift, roll oscillation, D-term spikes.
		 *
		 * Match the controller rate to the actual data rate: only
		 * step PID when copy() returns true (= new msg), and compute
		 * dt from the wall-clock interval between messages.
		 */
		/* Diagnostic state — rate-limited message-rx log + one-shot
		 * timeout-edge log. Both kept across loop iterations so we
		 * can see *whether* seeker packets arrive at all, even when
		 * the PID block below short-circuits on stale data. */
		static hrt_abstime last_rx_log_us = 0;
		static uint32_t    rx_count_since_log = 0;
		static bool        timeout_logged = false;

		tracking_message_s trk{};
		const bool new_msg = _tracking_msg_sub.copy(&trk);

		if (new_msg) {
			const float max_rad = math::radians(_p_max_deg.get());
			_errorx_rad = trk.errorx * max_rad;
			_errory_rad = trk.errory * max_rad;
			_last_msg_us = now;
			timeout_logged = false;   /* re-arm the timeout one-shot */

			rx_count_since_log++;

			if (now - last_rx_log_us >= 1_s) {
				PX4_INFO("TRK rx: errorx=%+.3f errory=%+.3f  (%u msgs in last %.1fs)",
					 (double)trk.errorx, (double)trk.errory,
					 (unsigned)rx_count_since_log,
					 (double)((now - last_rx_log_us) * 1e-6f));
				last_rx_log_us = now;
				rx_count_since_log = 0;
			}
		}

		/* timeout: reset PID integrals if no message received */
		if (now - _last_msg_us > (hrt_abstime)(_p_timeout_ms.get() * 1000ULL)) {
			_pid_roll.resetIntegral();
			_pid_pitch.resetIntegral();
			_pid_throttle.resetIntegral();
			_errorx_rad = 0.f;
			_errory_rad = 0.f;

			if (!timeout_logged) {
				PX4_WARN("TRK no message for %.1f s — PIDs reset, waiting for seeker",
					 (double)((now - _last_msg_us) * 1e-6f));
				timeout_logged = true;
			}
		}

		/* Skip PID + publish until a fresh msg arrives. The MC
		 * attitude controller will hold the last published att_sp,
		 * which is the correct behaviour (no fresh tracking info
		 * → no fresh setpoint).
		 */
		if (!new_msg) {
			continue;
		}

		/* ── get vehicle attitude ───────────────────────────── */
		vehicle_attitude_s att{};
		_vehicle_att_sub.copy(&att);

		vehicle_angular_velocity_s angvel{};
		_angular_vel_sub.copy(&angvel);

		const matrix::Eulerf euler(matrix::Quatf(att.q));
		const float actual_pitch_rad = euler.theta();
		const float roll_rate_radps  = angvel.xyz[0];

		/* ── GPS position ───────────────────────────────────── */
		vehicle_global_position_s gpos{};
		_global_pos_sub.copy(&gpos);

		const bool has_gps    = gpos.lat_lon_valid;
		float horiz_dist_m    = 0.f;
		float alt_rel_m       = 0.f;

		if (has_gps) {
			horiz_dist_m = get_distance_to_next_waypoint(
					       gpos.lat, gpos.lon,
					       (double)_p_tgt_lat.get(),
					       (double)_p_tgt_lon.get());
			alt_rel_m = gpos.alt - _p_tgt_alt.get();

			/* ── proximity alert ────────────────────────────── */
			const float close_m = _p_close_m.get();

			if (close_m > 0.f) {
				const bool close_now = (horiz_dist_m <= close_m);

				if (close_now && !_close_prev) {
					PX4_INFO("TRK proximity: CLOSE (%.0f m)", (double)horiz_dist_m);

				} else if (!close_now && _close_prev) {
					PX4_INFO("TRK proximity: FAR  (%.0f m)", (double)horiz_dist_m);
				}

				_close_prev = close_now;
			}
		}

		/* ── deadband (satria ArduCopter convention) ─────────────────
		 * Both ex and ey go to 0 when below deadband — no constant
		 * pitch-offset fallback. The fixed-wing -pitch_ofs_rad trick
		 * makes a copter drift forward forever when the target is
		 * centred; satria's ArduCopter mode_tracking deliberately
		 * leaves ey=0 below deadband. TRK_PTCH_OFS is preserved as
		 * a param for an eventual fixed-wing split.
		 */
		const float deadband_rad = math::radians(_p_dband.get());
		const float ex = (fabsf(_errorx_rad) > deadband_rad) ? _errorx_rad : 0.f;
		const float ey = (fabsf(_errory_rad) > deadband_rad) ? _errory_rad : 0.f;

		/* ── settle ramp ────────────────────────────────────── */
		const float settle_s = _p_settle_s.get();
		const float elapsed  = (now - _mode_entry_us) * 1e-6f;
		const float ramp     = (settle_s > 0.f) ?
				       math::constrain(elapsed / settle_s, 0.f, 1.f) : 1.f;

		/* dt = wall-clock interval since the previous tracking_message.
		 * We only reach this point on a new-msg arrival, so this is
		 * the actual sample period of the seeker's pipeline (~33 ms
		 * at 30 Hz camera). Clamp to [5 ms, 200 ms] to keep PID
		 * stable across frame-drop spikes.
		 */
		static hrt_abstime last_pid_us = 0;
		const float dt = (last_pid_us == 0) ? 0.033f :
				 math::constrain((now - last_pid_us) * 1e-6f, 0.005f, 0.2f);
		last_pid_us = now;

		/* ── lean limit from MPC_TILTMAX_AIR (degrees) ───────────────
		 * Replaces the hardcoded ±15°. With max_deg=30, a saturated
		 * errorx=±1 maps to ±30° PID input, which a 15° output limit
		 * can't track. Use the airframe's actual tilt ceiling so the
		 * controller can command the bank angle satria does.
		 */
		const float lean_max_rad = math::radians(_p_tilt_max.get());

		/* ── roll PID ───────────────────────────────────────── */
		float roll_sp_rad;

		if (fabsf(ex) < 1e-6f) {
			_pid_roll.resetIntegral();
			/* active damping: oppose roll rate to hold wings level */
			roll_sp_rad = -roll_rate_radps * 0.1f * ramp;

		} else {
			_pid_roll.setSetpoint(0.f);
			roll_sp_rad = _pid_roll.update(-ex, dt) * ramp;
		}

		roll_sp_rad = math::constrain(roll_sp_rad, -lean_max_rad, lean_max_rad);

		/* ── pitch / throttle split (vehicle-type aware) ─────────────
		 * Baseline = TRK_CRUISE_THR (configurable, fraction in [0,1]).
		 *
		 * ROTARY_WING (satria ArduCopter style):
		 *   ey > 0 (target above) → nose UP via pitch PID.
		 *   ey < 0 (target below) → REDUCE THROTTLE, hold pitch level.
		 *                           Pitching a copter down sends it
		 *                           forward (wrong); cutting thrust
		 *                           sends it down (right).
		 *
		 * FIXED_WING (pitch-for-airspeed):
		 *   pitch tracks ey in BOTH directions — a wing can pitch down to
		 *   chase a low target. Throttle then trims around cruise to hold
		 *   airspeed through the pitch change: a nose-up demand costs
		 *   energy → add throttle; nose-down → back off. The throttle PID
		 *   input is the commanded pitch (the airspeed-loss proxy), per
		 *   the module's vehicle-type-aware design.
		 *
		 * PX4 sign note: PID setpoint 0 with input −x gives error +x, so a
		 * positive ey (target above) yields positive pitch output = nose up.
		 */
		const bool is_fixed_wing =
			(status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

		const float cruise_thr = math::constrain(_p_cruise_thr.get() * 0.01f, 0.f, 1.f);
		float thrust_norm = cruise_thr * ramp;
		float pitch_sp_rad = 0.f;

		if (fabsf(ey) < 1e-6f) {
			_pid_pitch.resetIntegral();
			_pid_throttle.resetIntegral();

		} else if (is_fixed_wing) {
			/* plane: pitch toward the target up or down */
			_pid_pitch.setSetpoint(0.f);
			pitch_sp_rad = math::constrain(_pid_pitch.update(-ey, dt) * ramp,
						       -lean_max_rad, lean_max_rad);

			/* throttle trims around cruise to hold airspeed. Input
			 * −pitch_sp_rad (setpoint 0) → error = pitch_sp_rad → output
			 * > 0 when nose-up → add throttle. Output limit is ±50 %
			 * (see update_pid_gains), applied as a fraction of full. */
			_pid_throttle.setSetpoint(0.f);
			const float thro_pct = _pid_throttle.update(-pitch_sp_rad, dt) * ramp;
			thrust_norm = math::constrain(cruise_thr * ramp + thro_pct * 0.01f, 0.f, 1.f);

		} else {
			/* copter: pitch up for target-above, cut throttle for
			 * target-below, hold pitch level otherwise. */
			_pid_throttle.resetIntegral();
			_pid_pitch.setSetpoint(0.f);
			const float pid_out = _pid_pitch.update(-ey, dt) * ramp;

			if (ey > 0.f) {
				/* target above → pitch up */
				pitch_sp_rad = math::constrain(pid_out, -lean_max_rad, lean_max_rad);

			} else {
				/* target below → reduce throttle, keep pitch level */
				const float thro_delta = pid_out / lean_max_rad;
				thrust_norm = math::constrain(thrust_norm + thro_delta, 0.f, 1.f);
			}
		}

		/* ── publish attitude setpoint ──────────────────────── */
		vehicle_attitude_setpoint_s att_sp{};
		att_sp.timestamp = now;

		const matrix::Quatf q_sp(matrix::Eulerf(roll_sp_rad, pitch_sp_rad, euler.psi()));
		q_sp.copyTo(att_sp.q_d);

		// Vehicle-type-aware "body push": which FRD body axis the tracking
		// throttle is applied to.
		//   FIXED_WING  -> forward axis (+X): thrust drives the airframe
		//                  along its nose, lift comes from the wing, so a
		//                  Z-axis push is meaningless.
		//   ROTARY_WING -> up axis (-Z in FRD, i.e. NED down inverted):
		//                  thrust holds/changes altitude, there is no
		//                  body-forward thrust to command.
		// (is_fixed_wing computed once in the pitch/throttle split above.)
		att_sp.thrust_body[0] = is_fixed_wing ? thrust_norm : 0.f;
		att_sp.thrust_body[1] = 0.f;
		att_sp.thrust_body[2] = is_fixed_wing ? 0.f : -thrust_norm;

		_att_sp_pub.publish(att_sp);

		/* ── rate-limited 1 Hz telemetry ────────────────────── */
		if (now - _last_telem_us >= 1_s) {
			_last_telem_us = now;

			if (has_gps) {
				PX4_INFO("TRK d=%.0fm alt=%.0fm ex=%.2f ey=%.2f navx=%.1f navy=%.1f thr=%.0f%%",
					 (double)horiz_dist_m, (double)alt_rel_m,
					 (double)math::degrees(ex), (double)math::degrees(ey),
					 (double)math::degrees(roll_sp_rad), (double)math::degrees(pitch_sp_rad),
					 (double)(thrust_norm * 100.0f));
			}

			PX4_INFO("TRK p_ahrs=%.1f nav=%.1f thr=%.1f%% ramp=%.2f",
				 (double)math::degrees(actual_pitch_rad),
				 (double)math::degrees(pitch_sp_rad),
				 (double)thrust_norm * 100.0f,
				 (double)ramp);
		}
	}

	if (_sent_mode_registration && _mode_id >= 0) {
		unregister_tracking_mode();
	}

	PX4_INFO("tracking: exiting");
}

int Tracking::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return Tracking::instantiate(ac, av);
	}, argc, argv);
}

int Tracking::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("tracking",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  PX4_STACK_ADJUSTED(2000),
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	return 0;
}

Tracking *Tracking::instantiate(int argc, char *argv[])
{
	return new Tracking();
}

int Tracking::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Tracking::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Fixed-wing TRACKING mode — visual-servo control from external seeker errors.

Receives TRACKING_MESSAGE (MAVLink ID 11045) with normalised errorx/errory,
runs roll/pitch/throttle PIDs, and publishes vehicle_attitude_setpoint.

Active only when nav_state == NAVIGATION_STATE_EXTERNAL1 (23).

Features: GPS proximity alerts, 1 Hz status telemetry, configurable cruise throttle.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tracking", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

ModuleBase::Descriptor Tracking::desc{Tracking::task_spawn, Tracking::custom_command, Tracking::print_usage};

extern "C" __EXPORT int tracking_main(int argc, char *argv[])
{
	return ModuleBase::main(Tracking::desc, argc, argv);
}
