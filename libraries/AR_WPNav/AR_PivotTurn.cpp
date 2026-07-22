/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AR_PivotTurn.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

#define AR_PIVOT_TIMEOUT_MS     100 // pivot controller timesout and reset target if not called within this many milliseconds
#define AR_PIVOT_ANGLE_DEFAULT  60  // default PIVOT_ANGLE parameter value
#define AR_PIVOT_ANGLE_ACCURACY 5   // vehicle will pivot to within this many degrees of destination
#define AR_PIVOT_RATE_DEFAULT   60  // default PIVOT_RATE parameter value
#define AR_PIVOT_DELAY_DEFAULT  0   // default PIVOT_DELAY parameter value
#define AR_PIVOT_DIRECTION_LOCK_DEG 170.0f
#define AR_PIVOT_DIRECTION_RELEASE_DEG 160.0f

const AP_Param::GroupInfo AR_PivotTurn::var_info[] = {

    // @Param: ANGLE
    // @DisplayName: Pivot Angle
    // @Description: Pivot when the difference between the vehicle's heading and its target heading is more than this many degrees. Set to zero to disable pivot turns.  This parameter should be greater than 5 degrees for pivot turns to work.
    // @Units: deg
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE", 1, AR_PivotTurn, _angle, AR_PIVOT_ANGLE_DEFAULT),

    // @Param: RATE
    // @DisplayName: Pivot Turn Rate
    // @Description: Turn rate during pivot turns
    // @Units: deg/s
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE", 2, AR_PivotTurn, _rate_max, AR_PIVOT_RATE_DEFAULT),

    // @Param: DELAY
    // @DisplayName: Pivot Delay
    // @Description: Vehicle waits this many seconds after completing a pivot turn before proceeding
    // @Units: s
    // @Range: 0 60
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("DELAY", 3, AR_PivotTurn, _delay, AR_PIVOT_DELAY_DEFAULT),

    AP_GROUPEND
};

AR_PivotTurn::AR_PivotTurn(AR_AttitudeControl& atc) :
    _atc(atc)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// enable or disable pivot controller
void AR_PivotTurn::enable(bool enable_pivot)
{
    _enabled = enable_pivot;
    if (!_enabled) {
        deactivate();
    }
}

// true if update has been called recently
bool AR_PivotTurn::active() const
{
    return _enabled && _active;
}

// checks if pivot turns should be activated or deactivated
// force_active should be true if the caller wishes to trigger the start of a pivot turn regardless of the heading error
void AR_PivotTurn::check_activation(float desired_heading_deg, bool force_active)
{
    // check cases where we clearly cannot use pivot steering
    if (!_enabled || (_angle <= AR_PIVOT_ANGLE_ACCURACY)) {
        deactivate();
        return;
    }

    const float yaw_error = get_heading_error_deg(desired_heading_deg);

    // if error is larger than _pivot_angle start pivot steering
    if (yaw_error > _angle || force_active) {
        if (!_active) {
            _completion.reset();
            clear_planned_state();
        }
        _active = true;
        _legacy_delay_start_ms = 0U;
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // Preserve the established completion behavior for ordinary AUTO/Guided
    // pivots.  Exact planned spins use update_completion() instead.
    if (_active && !_planned_turn &&
        (yaw_error < AR_PIVOT_ANGLE_ACCURACY) &&
        (_legacy_delay_start_ms == 0U)) {
        // Zero means "not started", so avoid storing the boot-time sentinel.
        _legacy_delay_start_ms = MAX(now_ms, 1U);
    }

    // Leaving the accuracy window invalidates an ordinary pivot's completion
    // delay just as it does for a planned pivot.
    if (_active && !_planned_turn &&
        (yaw_error >= AR_PIVOT_ANGLE_ACCURACY)) {
        _legacy_delay_start_ms = 0U;
    }

    if (!_planned_turn &&
        (_legacy_delay_start_ms > 0U) &&
        ((now_ms - _legacy_delay_start_ms) >= get_delay_duration_ms())) {
        deactivate();
    }
}

// check if pivot turn would be activated given an expected change in yaw in degrees
// note this does not actually active the pivot turn.  To activate use the check_activation method
bool AR_PivotTurn::would_activate(float yaw_change_deg) const
{
    // check cases where we clearly cannot use pivot steering
    if (!_enabled || (_angle <= AR_PIVOT_ANGLE_ACCURACY)) {
        return false;
    }

    // return true if yaw change is larger than _pivot_angle
    return fabsf(wrap_180(yaw_change_deg)) > _angle;
}

bool AR_PivotTurn::activate_planned(int8_t preferred_direction)
{
    if (!_enabled) {
        deactivate();
        return false;
    }
    preferred_direction = constrain_int16(preferred_direction, -1, 1);
    _completion.reset();
    _planned_turn = true;
    _planned_preferred_direction = preferred_direction;
    _planned_direction_initialised = preferred_direction != 0;
    _planned_turn_direction = preferred_direction;
    _legacy_delay_start_ms = 0U;
    _active = true;
    return true;
}

// forcibly deactivate this controller
void AR_PivotTurn::deactivate()
{
    _active = false;
    _legacy_delay_start_ms = 0U;
    _completion.reset();
    clear_planned_state();
}

// get turn rate (in rad/sec) without changing completion state
// desired heading should be the heading towards the next waypoint in degrees
float AR_PivotTurn::get_turn_rate_rads(float desired_heading_deg)
{
    float yaw_error_deg = wrap_180(desired_heading_deg - (AP::ahrs().yaw_sensor * 0.01f));

    if (_planned_turn && !_planned_direction_initialised) {
        _planned_direction_initialised = true;
        if (fabsf(yaw_error_deg) >= AR_PIVOT_DIRECTION_LOCK_DEG) {
            _planned_turn_direction = is_negative(yaw_error_deg) ? -1 : 1;
        }
    }

    if (_planned_turn_direction != 0) {
        if (fabsf(yaw_error_deg) < AR_PIVOT_DIRECTION_RELEASE_DEG) {
            _planned_turn_direction = 0;
            _planned_preferred_direction = 0;
        } else {
            // wrap_PI maps both +PI and -PI to +PI. Keep a negative planned
            // 180-degree turn just inside that boundary so the attitude
            // controller cannot lose the selected direction on a second wrap.
            const float directed_error_deg = MIN(fabsf(yaw_error_deg), 179.99f);
            yaw_error_deg = copysignf(directed_error_deg,
                                      float(_planned_turn_direction));
        }
    }

    // Convert the selected signed error back into a live heading target.  This
    // preserves the planned direction around 180 degrees using the existing
    // attitude-controller interface.
    const float desired_heading_rad = AP::ahrs().get_yaw() + radians(yaw_error_deg);
    return _atc.get_turn_rate_from_heading(desired_heading_rad, radians(_rate_max));
}

// reset completion and planned direction after an estimator yaw reset
void AR_PivotTurn::handle_yaw_reset()
{
    _completion.reset();
    _planned_direction_initialised = _planned_preferred_direction != 0;
    _planned_turn_direction = _planned_preferred_direction;
}

// update completion state after the caller has applied its speed gate
bool AR_PivotTurn::update_completion(float desired_heading_deg, float yaw_rate_rads, uint32_t now_ms)
{
    if (!active()) {
        _completion.reset();
        return false;
    }

    if (!_completion.update(get_heading_error_deg(desired_heading_deg),
                            degrees(yaw_rate_rads),
                            now_ms,
                            get_delay_duration_ms())) {
        return false;
    }

    deactivate();
    return true;
}

// return post-turn delay duration in milliseconds
uint32_t AR_PivotTurn::get_delay_duration_ms() const
{
    return constrain_float(_delay.get(), 0.0f, 60.0f) * 1000;
}

// return absolute heading error in degrees
float AR_PivotTurn::get_heading_error_deg(float desired_heading_deg) const
{
    return fabsf(wrap_180(desired_heading_deg - (AP::ahrs().yaw_sensor * 0.01f)));
}

void AR_PivotTurn::clear_planned_state()
{
    _planned_turn = false;
    _planned_direction_initialised = false;
    _planned_turn_direction = 0;
    _planned_preferred_direction = 0;
}
