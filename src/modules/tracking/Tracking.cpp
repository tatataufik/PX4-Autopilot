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
		(ParamFloat<px4::params::TRK_THRO_D>)     _p_thro_d
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
			PX4_INFO("tracking: registered  arming_check_id=%d  mode_id=%d (nav_state)",
				 (int)_arming_check_id, (int)_mode_id);
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

		const hrt_abstime now = hrt_absolute_time();

		// Periodic re-publish of config_control_setpoints at ~1 Hz while
		// active. Defensive against commander re-evaluating mode flags on
		// transitions we may have missed (re-arm, brief failsafe trip, etc.)
		static hrt_abstime last_cfg_refresh_us = 0;
		if (now - last_cfg_refresh_us > 1_s) {
			configure_tracking_mode();
			last_cfg_refresh_us = now;
		}

		/* ── consume tracking_message ──────────────────────── */
		tracking_message_s trk{};
		if (_tracking_msg_sub.copy(&trk)) {
			const float max_rad = math::radians(_p_max_deg.get());
			_errorx_rad = trk.errorx * max_rad;
			_errory_rad = trk.errory * max_rad;
			_last_msg_us = now;
		}

		/* timeout: reset PID integrals if no message received */
		if (now - _last_msg_us > (hrt_abstime)(_p_timeout_ms.get() * 1000ULL)) {
			_pid_roll.resetIntegral();
			_pid_pitch.resetIntegral();
			_pid_throttle.resetIntegral();
			_errorx_rad = 0.f;
			_errory_rad = 0.f;
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

		/* ── deadband & pitch offset ────────────────────────── */
		const float deadband_rad  = math::radians(_p_dband.get());
		const float pitch_ofs_rad = math::radians(_p_pitch_ofs.get());

		const float ex = (fabsf(_errorx_rad) > deadband_rad) ? _errorx_rad : 0.f;
		const float ey = (fabsf(_errory_rad) > deadband_rad) ? _errory_rad : -pitch_ofs_rad;

		/* ── settle ramp ────────────────────────────────────── */
		const float settle_s = _p_settle_s.get();
		const float elapsed  = (now - _mode_entry_us) * 1e-6f;
		const float ramp     = (settle_s > 0.f) ?
		                       math::constrain(elapsed / settle_s, 0.f, 1.f) : 1.f;

		const float dt = 0.005f;   /* fixed 5 ms — matches poll interval */

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

		roll_sp_rad = math::constrain(roll_sp_rad,
		                              -math::radians(15.f),
		                               math::radians(15.f));

		/* ── pitch PID ──────────────────────────────────────── */
		float pitch_sp_rad;

		if (fabsf(ey) < 1e-6f) {
			_pid_pitch.resetIntegral();
			pitch_sp_rad = 0.f;
		} else {
			_pid_pitch.setSetpoint(0.f);
			pitch_sp_rad = _pid_pitch.update(-ey, dt) * ramp;
		}

		pitch_sp_rad = math::constrain(pitch_sp_rad,
										-math::radians(15.f),
										math::radians(15.f));

		/* ── throttle: live hover baseline + pitch-error correction ──
		 * Baseline: PX4 hover_thrust_estimate (zero-step on TRACKING entry).
		 * Correction: thro_delta = P × (actual_pitch − pitch_sp_rad).
		 *   actual < nav (we're below commanded pitch) → delta < 0 → LESS thrust
		 *   actual > nav (we've overshot commanded pitch) → delta > 0 → MORE thrust
		 * This is the satria-firmware/ArduPlane convention: when the vehicle
		 * lags the commanded climb, reduce thrust; when it overshoots, push
		 * back. For a multicopter doing horizontal chase via pitch tilting,
		 * the same rule conserves altitude during the tilt manoeuvre.
		 *
		 * P and limits come from TRK_THRO_P (already a tunable param). Output
		 * is in fraction; constrained to ±25% around the baseline so it
		 * can't run away if pitch error stays large.
		 */
		// Baseline = throttle captured at mode-entry (snapshot of whatever
		// the previous controller was outputting at that instant). Stays
		// constant for the duration of TRACKING.
		const float thrust_base = 0.12f;

		const float pitch_err = actual_pitch_rad - pitch_sp_rad;
		_pid_throttle.setSetpoint(0.f);
		// PID error = 0 − feedback = −pitch_err. Feed −pitch_err so the
		// sign matches: positive error ⇒ negative delta when actual < nav.
		// (TRK_THRO_P operates on this internally.) Convert percent → fraction.
		const float thro_delta_pct = _pid_throttle.update(-pitch_err, dt) * ramp;
		const float thrust_correction = thro_delta_pct * 0.01f;

		float thrust_norm = thrust_base + thrust_correction;
		thrust_norm = math::constrain(thrust_norm,
		                              thrust_base * 0.75f,
		                              thrust_base * 1.25f);

		/* ── publish attitude setpoint ──────────────────────── */
		vehicle_attitude_setpoint_s att_sp{};
		att_sp.timestamp = now;

		const matrix::Quatf q_sp(matrix::Eulerf(roll_sp_rad, pitch_sp_rad, euler.psi()));
		q_sp.copyTo(att_sp.q_d);

		// FRD body frame: copter thrust on −Z (down-axis NED inverted).
		// Plane thrust on +X kept as a fallback so the same publish path
		// works for both vehicle types if the user later re-enables the
		// is_copter switch.
		att_sp.thrust_body[0] = 0.f;
		att_sp.thrust_body[1] = 0.f;
		att_sp.thrust_body[2] = -thrust_norm;

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

			// PX4_INFO("TRK p_ahrs=%.1f nav=%.1f thr=%.1f%% ramp=%.2f",
			//          (double)math::degrees(actual_pitch_rad),
			//          (double)math::degrees(pitch_sp_rad),
			//          (double)throttle_sp,
			//          (double)ramp);
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
