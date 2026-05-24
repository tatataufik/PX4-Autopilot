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

	/* ── publications ──────────────────────────────────────────── */
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};

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
	_pid_pitch.setOutputLimit(math::radians(30.f));
	_pid_pitch.setIntegralLimit(math::radians(15.f));

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

		/* update vehicle status */
		vehicle_status_s status{};
		_vehicle_status_sub.copy(&status);

		const bool active = (status.nav_state ==
		                     vehicle_status_s::NAVIGATION_STATE_EXTERNAL1);

		/* ── on mode entry ─────────────────────────────────── */
		if (active && !_was_active) {
			reset_pids();
			update_pid_gains();
			_mode_entry_us = hrt_absolute_time();
			_last_telem_us = 0;
			_close_prev    = false;
			PX4_INFO("tracking: TRACKING active");
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
		                              -math::radians(45.f),
		                               math::radians(45.f));

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
		                               -math::radians(30.f),
		                                math::radians(30.f));

		/* ── throttle PID ───────────────────────────────────── */
		/* Mirrors satria: cruise ± pid_out, clamped to [3/4*cruise, 7/5*cruise].
		 * Vehicle-type-aware error source:
		 *   FIXED_WING : pitch_err (pitch-for-airspeed)
		 *   ROTARY_WING: altitude_err vs TRK_TGT_ALT (throttle holds altitude) */
		const bool is_copter = (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
		const float cruise = _p_cruise_thr.get();

		float thro_input;

		if (is_copter) {
			/* +alt_err = below target → need more throttle. Feed −alt_err so
			 * PID(setpoint=0, input=-alt_err) yields positive delta. */
			const float alt_err_m = has_gps ? (_p_tgt_alt.get() - gpos.alt) : 0.f;
			thro_input = -alt_err_m;
		} else {
			thro_input = actual_pitch_rad - pitch_sp_rad;
		}

		_pid_throttle.setSetpoint(0.f);
		const float thro_delta  = _pid_throttle.update(-thro_input, dt) * ramp;
		const float throttle_sp = math::constrain(cruise + thro_delta,
		                                           cruise * 0.75f,
		                                           cruise * 1.4f);

		/* ── publish attitude setpoint ──────────────────────── */
		/* PX4 v1 VehicleAttitudeSetpoint uses q_d quaternion + thrust_body.
		 *   FIXED_WING : thrust_body[0] = throttle [0,1]  (forward body axis)
		 *   ROTARY_WING: thrust_body[2] = −throttle [0,1] (NED, negative = up) */
		vehicle_attitude_setpoint_s att_sp{};
		att_sp.timestamp = now;

		const matrix::Quatf q_sp(matrix::Eulerf(roll_sp_rad, pitch_sp_rad, euler.psi()));
		q_sp.copyTo(att_sp.q_d);

		const float thrust_norm = throttle_sp * 0.01f;
		att_sp.thrust_body[0] = is_copter ? 0.f : thrust_norm;
		att_sp.thrust_body[1] = 0.f;
		att_sp.thrust_body[2] = is_copter ? -thrust_norm : 0.f;

		_att_sp_pub.publish(att_sp);

		/* ── rate-limited 1 Hz telemetry ────────────────────── */
		if (now - _last_telem_us >= 1_s) {
			_last_telem_us = now;

			if (has_gps) {
				PX4_INFO("TRK d=%.0fm alt=%.0fm ex=%.2f ey=%.2f",
				         (double)horiz_dist_m, (double)alt_rel_m,
				         (double)math::degrees(ex), (double)math::degrees(ey));
			}

			PX4_INFO("TRK[%s] p_ahrs=%.1f nav=%.1f thr=%.1f%% ramp=%.2f",
			         is_copter ? "MC" : "FW",
			         (double)math::degrees(actual_pitch_rad),
			         (double)math::degrees(pitch_sp_rad),
			         (double)throttle_sp,
			         (double)ramp);
		}
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
