/**
 * fw_tracking module parameters
 *
 * Mirror of satria-firmware TRK_* parameters for the TRACKING flight mode.
 * The companion computer sends normalised errors in [-1, 1] via the
 * TRACKING_MESSAGE MAVLink message; these parameters configure the on-board
 * PID controllers that translate those errors into attitude setpoints.
 *
 * Defaults match satria-firmware Parameters.cpp revision.
 */

#include <parameters/param.h>

/**
 * Tracking: maximum error angle
 *
 * Normalised error ±1 maps to this angle in degrees.
 * Used to convert errorx/errory [-1,1] to radians before PID.
 *
 * @unit deg
 * @min 1.0
 * @max 90.0
 * @decimal 1
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_MAX_DEG, 30.0f);

/**
 * Tracking: error deadband
 *
 * Errors smaller than this (in degrees) are treated as zero.
 * Prevents hunting around the aim point.
 *
 * @unit deg
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_DBAND, 0.573f);

/**
 * Tracking: settle ramp duration
 *
 * Time in seconds over which PID authority ramps from 0 to 1 after
 * mode entry.  Prevents snapping to full authority on activation.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 1
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_SETTLE_S, 2.0f);

/**
 * Tracking: constant pitch bias
 *
 * Added to the pitch setpoint at all times in TRACKING mode.
 * Positive = nose-up bias (matches satria TRK_PITCH_OFFSET default 3 deg).
 *
 * @unit deg
 * @min -30.0
 * @max  30.0
 * @decimal 1
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_PTCH_OFS, 3.0f);

/**
 * Tracking: signal loss timeout
 *
 * If no TRACKING_MESSAGE is received for this many milliseconds
 * the mode resets accumulated PID state.
 *
 * @unit ms
 * @min 100
 * @max 5000
 * @group FW Tracking
 */
PARAM_DEFINE_INT32(TRK_TIMEOUT, 1000);

/**
 * Tracking: cruise throttle baseline
 *
 * Throttle percentage used as the centre-point for the throttle PID.
 * The throttle PID adds ±delta around this value.
 * Matches TRIM_THROTTLE intention in satria-firmware.
 *
 * @unit %
 * @min 10.0
 * @max 100.0
 * @decimal 1
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_CRUISE_THR, 50.0f);

/**
 * Tracking: target latitude
 *
 * WGS-84 latitude of the ground target, decimal degrees.
 * Used for horizontal distance and proximity alerts.
 * Matches satria-firmware TRK_TGT_LAT default (Bandung area).
 *
 * @unit deg
 * @min -90.0
 * @max  90.0
 * @decimal 7
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_TGT_LAT, -6.8974f);

/**
 * Tracking: target longitude
 *
 * WGS-84 longitude of the ground target, decimal degrees.
 *
 * @unit deg
 * @min -180.0
 * @max  180.0
 * @decimal 7
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_TGT_LON, 107.5666f);

/**
 * Tracking: target MSL altitude
 *
 * MSL altitude of the target in metres.
 * Used for altitude-relative telemetry reports.
 *
 * @unit m
 * @min -100.0
 * @max 5000.0
 * @decimal 1
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_TGT_ALT, 744.0f);

/**
 * Tracking: proximity alert threshold
 *
 * Horizontal distance (m) at which a GCS alert is issued when the
 * drone enters or exits the close-to-target radius.
 * Set to 0 to disable.
 *
 * @unit m
 * @min 0.0
 * @max 50000.0
 * @decimal 0
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_CLOSE_M, 1000.0f);

/**
 * Tracking roll PID — proportional gain
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_ROLL_P, 5.0f);

/**
 * Tracking roll PID — integral gain
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_ROLL_I, 0.1f);

/**
 * Tracking roll PID — derivative gain
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_ROLL_D, 0.05f);

/**
 * Tracking pitch PID — proportional gain
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_PTCH_P, 5.0f);

/**
 * Tracking pitch PID — integral gain
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_PTCH_I, 0.1f);

/**
 * Tracking pitch PID — derivative gain
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_PTCH_D, 0.05f);

/**
 * Tracking throttle PID — proportional gain
 *
 * Applied to pitch-attitude error (actual_pitch - nav_pitch_setpoint).
 *
 * @min 0.0
 * @max 50.0
 * @decimal 2
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_THRO_P, 10.0f);

/**
 * Tracking throttle PID — integral gain
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_THRO_I, 0.5f);

/**
 * Tracking throttle PID — derivative gain
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @group FW Tracking
 */
PARAM_DEFINE_FLOAT(TRK_THRO_D, 0.0f);
