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
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AR_WPNav.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

#define AR_WPNAV_TIMEOUT_MS             100
#define AR_WPNAV_SPEED_DEFAULT          2.0f
#define AR_WPNAV_SPEED_MIN              0.05f   // minimum speed between waypoints in m/s
#define AR_WPNAV_SPEED_UPDATE_MIN_MS    500     // max speed cannot be updated more than once in this many milliseconds
#define AR_WPNAV_RADIUS_DEFAULT         2.0f
#define AR_WPNAV_OVERSPEED_RATIO_MAX    5.0f    // if _overspeed_enabled the vehicle may travel as quickly as 5x WP_SPEED
#define AR_WPNAV_SNAP_MAX               15.0f   // scurve snap (change in jerk) in m/s/s/s/s
#define AR_WPNAV_ACCEL_MAX              20.0    // acceleration used when user has specified no acceleration limit
#define AR_WPNAV_PIVOT_RADIUS_DEFAULT   0.2f
#define AR_WPNAV_PIVOT_RELEASE_RATIO    1.5f    // endpoint hysteresis while SPIN owns the vehicle
#define AR_WPNAV_EXACT_POS_LOSS_MS      500U    // tolerate a transient position sample loss while holding SPIN
#define AR_WPNAV_EXACT_DRIFT_MS         250U
#define AR_WPNAV_PIVOT_EXIT_DEFAULT     12.0f
#define AR_WPNAV_PIVOT_DRIFT_DEFAULT    0.5f
#define AR_WPNAV_PIVOT_REJOIN_DEFAULT   0.3f
#define AR_WPNAV_PIVOT_TIMEOUT_DEFAULT  15.0f
#define AR_WPNAV_PIVOT_CAPSPD_DEFAULT   0.2f
#define AR_WPNAV_PIVOT_BLEND_DEFAULT    0.5f
#define AR_WPNAV_CAPTURE_INNER_RATIO    0.75f
#define AR_WPNAV_CAPTURE_MAX_SPEED      0.3f
#define AR_WPNAV_CAPTURE_XTRACK_MAX     0.15f
#define AR_WPNAV_CAPTURE_HEADING_MAX_DEG 5.0f
#define AR_WPNAV_CAPTURE_VIOLATION_MS   250U

const AP_Param::GroupInfo AR_WPNav::var_info[] = {

    // @Param: SPEED
    // @DisplayName: Waypoint speed default
    // @Description: Waypoint speed default
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("SPEED", 1, AR_WPNav, _speed_max, AR_WPNAV_SPEED_DEFAULT),

    // @Param: RADIUS
    // @DisplayName: Waypoint radius
    // @Description: The distance in meters from a waypoint when we consider the waypoint has been reached. This determines when the vehicle will turn toward the next waypoint.
    // @Units: m
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("RADIUS", 2, AR_WPNav, _radius, AR_WPNAV_RADIUS_DEFAULT),

    // 3 was OVERSHOOT

    // 4 was PIVOT_ANGLE
    // 5 was PIVOT_RATE
    // 6 was SPEED_MIN
    // 7 was PIVOT_DELAY

    // @Group: PIVOT_
    // @Path: AR_PivotTurn.cpp
    AP_SUBGROUPINFO(_pivot, "PIVOT_", 8, AR_WPNav, AR_PivotTurn),

    // @Param: ACCEL
    // @DisplayName: Waypoint acceleration
    // @Description: Waypoint acceleration.  If zero then ATC_ACCEL_MAX is used
    // @Units: m/s/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ACCEL", 9, AR_WPNav, _accel_max, 0),

    // @Param: JERK
    // @DisplayName: Waypoint jerk
    // @Description: Waypoint jerk (change in acceleration).  If zero then jerk is same as acceleration
    // @Units: m/s/s/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("JERK", 10, AR_WPNav, _jerk_max, 0),

    // @Param: PIVOT_RADIUS
    // @DisplayName: Planned pivot endpoint radius
    // @Description: Maximum realtime position and cross-track error allowed before a stopping S-curve hands control to a planned differential-drive pivot
    // @Units: m
    // @Range: 0.05 2.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("PIVOT_RADIUS", 11, AR_WPNav, _pivot_radius, AR_WPNAV_PIVOT_RADIUS_DEFAULT),

    // @Param: PIVOT_EXIT
    // @DisplayName: Planned pivot exit angle
    // @Description: Heading error at which a planned pivot atomically hands control to the cached next path. Zero restores the strict five degree, yaw-rate and PIVOT_DELAY completion rule
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PIVOT_EXIT", 12, AR_WPNav, _pivot_exit, AR_WPNAV_PIVOT_EXIT_DEFAULT),

    // @Param: PIVOT_DRIFT
    // @DisplayName: Planned pivot drift limit
    // @Description: Maximum D0 envelope for the one-shot forward capture path and for position drift during a planned pivot. Exceeding it faults instead of chasing D0. Zero disables local capture recovery and uses one-and-a-half PIVOT_RADIUS as the Spin fault limit
    // @Units: m
    // @Range: 0 3
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("PIVOT_DRIFT", 13, AR_WPNav, _pivot_drift, AR_WPNAV_PIVOT_DRIFT_DEFAULT),

    // @Param: PIVOT_REJOIN
    // @DisplayName: Planned pivot forward rejoin
    // @Description: Maximum existing forward displacement on the frozen next route absorbed when a planned pivot hands control to its cached path. This does not add lookahead. Zero disables forward rejoin
    // @Units: m
    // @Range: 0 1
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("PIVOT_REJOIN", 14, AR_WPNav, _pivot_rejoin, AR_WPNAV_PIVOT_REJOIN_DEFAULT),

    // @Param: PIVOT_TOUT
    // @DisplayName: Planned pivot timeout
    // @Description: Maximum time spent in an initial entry pivot or in terminal endpoint capture, planned Spin and recovery before faulting. Normal travel along a waypoint leg is not timed. Zero disables this timeout
    // @Units: s
    // @Range: 0 60
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PIVOT_TOUT", 15, AR_WPNav, _pivot_timeout, AR_WPNAV_PIVOT_TIMEOUT_DEFAULT),

    // @Param: PIVOT_CAPSPD
    // @DisplayName: Planned pivot early-capture speed
    // @Description: Maximum S-curve planned speed at which a vehicle and target already inside PIVOT_RADIUS may hand directly to a planned pivot. Zero disables early capture but never restores realtime point-chasing recovery
    // @Units: m/s
    // @Range: 0 0.5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("PIVOT_CAPSPD", 16, AR_WPNav, _pivot_capture_speed, AR_WPNAV_PIVOT_CAPSPD_DEFAULT),

    // @Param: PIVOT_BLEND
    // @DisplayName: Planned pivot path heading blend
    // @Description: Distance over which the locked next-leg heading controller blends into path steering after a planned pivot. For nonzero values the lock is fully released only after this distance is complete and heading error is within one degree. Zero hands steering directly to the path controller
    // @Units: m
    // @Range: 0 2
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("PIVOT_BLEND", 17, AR_WPNav, _pivot_blend, AR_WPNAV_PIVOT_BLEND_DEFAULT),

    AP_GROUPEND
};

AR_WPNav::AR_WPNav(AR_AttitudeControl& atc, AR_PosControl &pos_control) :
    _pivot(atc),
    _atc(atc),
    _pos_control(pos_control)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise waypoint controller.  speed_max should be set to the maximum speed in m/s (or left at zero to use the default speed)
void AR_WPNav::init(float speed_max)
{
    // determine max speed, acceleration and jerk
    if (is_positive(speed_max)) {
        _base_speed_max = speed_max;
    } else {
        _base_speed_max = _speed_max;
    }
    _base_speed_max = MAX(AR_WPNAV_SPEED_MIN, _base_speed_max);
    float atc_accel_max = MIN(_atc.get_accel_max(), _atc.get_decel_max());
    if (!is_positive(atc_accel_max)) {
        // accel_max of zero means no limit so use maximum acceleration
        atc_accel_max = AR_WPNAV_ACCEL_MAX;
    }
    const float accel_max = is_positive(_accel_max) ? MIN(_accel_max, atc_accel_max) : atc_accel_max;
    const float jerk_max = is_positive(_jerk_max) ? _jerk_max : accel_max;

    // initialise position controller
    _pos_control.set_limits(_base_speed_max, accel_max, _atc.get_turn_lat_accel_max(), jerk_max);

    _scurve_prev_leg.init();
    _scurve_this_leg.init();
    _scurve_next_leg.init();
    _track_scalar_dt = 1.0f;

    // init some flags
    _reached_destination = false;
    _fast_waypoint = false;

    // ensure pivot turns are deactivated
    _pivot.deactivate();
    _pivot_handoff = false;
    _pivot_heading_valid = false;
    _pivot_at_next_wp = false;
    clear_exact_pivot();

    // initialise origin and destination to stopping point
    _orig_and_dest_valid = false;
    set_origin_and_destination_to_stopping_point();

    // initialise nudge speed to zero
    set_nudge_speed_max(0);
}

// update navigation
void AR_WPNav::update(float dt)
{
    // Release an ordinary pivot's zero-output completion edge at the start of
    // the following navigation update.
    _pivot_handoff = false;

    // Exact faults are latched until a new navigation transaction explicitly
    // replaces them.  Do not advance a finished S-curve behind a faulted Hold.
    if (_exact_phase == ExactPivotPhase::Fault) {
        _last_update_ms = AP_HAL::millis();
        _desired_speed_limited = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _desired_lat_accel = 0.0f;
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const bool soft_armed = hal.util->get_soft_armed();
    if (!soft_armed && (_exact_phase != ExactPivotPhase::None)) {
        // Planned navigation may be installed well before arming.  Pivot
        // timeout is execution time, not wall-clock time spent disarmed.
        _exact_timeout_active = false;
        _exact_timeout_start_ms = 0U;
        _exact_position_loss_active = false;
        _exact_position_loss_start_ms = 0U;
        _exact_drift_active = false;
        _exact_drift_start_ms = 0U;
        _exact_capture_violation_start_ms = 0U;
        if (_exact_phase == ExactPivotPhase::Spin) {
            _pivot.reset_completion();
        }
    } else if (soft_armed &&
               !_exact_timeout_active &&
               (((_exact_phase == ExactPivotPhase::Move) &&
                 _pivot.active()) ||
                (_exact_phase == ExactPivotPhase::Spin) ||
                (_exact_phase == ExactPivotPhase::CapturePath))) {
        // Restart the applicable entry, capture-path or Spin budget when
        // execution can actually begin after arming.
        start_exact_pivot_timeout(now_ms);
    }
    if ((_exact_phase != ExactPivotPhase::None) &&
        (_exact_phase != ExactPivotPhase::PromotedMove) &&
        exact_pivot_timed_out(now_ms)) {
        _last_update_ms = now_ms;
        enter_exact_fault(ExactPivotFault::Timeout);
        return;
    }

    // Exact translation requires the EKF-local body-origin position and
    // velocity used by both S-Curve and PSC. get_location() alone can remain
    // true while either local signal is unavailable; without this gate the
    // path silently produces zero forever. SPIN intentionally needs only a
    // global body-origin location and live AHRS yaw, never a linear-speed
    // estimate from a differential drive while rotating in place, but it still
    // requires live AHRS yaw and yaw-rate for steering-rate control.
    const bool exact_translation = (_exact_phase == ExactPivotPhase::Move) ||
                                   (_exact_phase == ExactPivotPhase::CapturePath) ||
                                   (_exact_phase == ExactPivotPhase::PromotedMove);
    const bool exact_spin = _exact_phase == ExactPivotPhase::Spin;
    if ((exact_translation || exact_spin) &&
        soft_armed && _orig_and_dest_valid) {
        _last_update_ms = now_ms;
        Location current_loc;
        Vector2f current_pos_ne;
        Vector3f current_vel_ned;
        const bool global_position_valid = AP::ahrs().get_location(current_loc);
        const bool local_navigation_valid = !exact_translation ||
            (AP::ahrs().get_relative_position_NE_origin(current_pos_ne) &&
             AP::ahrs().get_velocity_NED(current_vel_ned));
        const bool yaw_required = exact_spin || exact_translation;
        const bool yaw_valid = !yaw_required || isfinite(AP::ahrs().get_yaw());
        const bool yaw_rate_valid = !yaw_required || isfinite(AP::ahrs().get_yaw_rate_earth());
        if (global_position_valid) {
            update_exact_pivot_diag_endpoint(current_loc);
        } else {
            _exact_diag.endpoint_valid = false;
        }
        if (!global_position_valid || !local_navigation_valid || !yaw_valid || !yaw_rate_valid) {
            // Freeze on a transient estimator dropout. One continuous 500ms
            // loss becomes a visible, latched fault instead of a permanent
            // Path primitive with zero speed and zero turn rate.
            _desired_speed_limited = 0.0f;
            _desired_turn_rate_rads = 0.0f;
            _desired_lat_accel = 0.0f;
            if (exact_spin || _pivot.active()) {
                _pivot.reset_completion();
            }
            if (!_exact_position_loss_active) {
                _exact_position_loss_active = true;
                _exact_position_loss_start_ms = _last_update_ms;
            } else if ((_last_update_ms - _exact_position_loss_start_ms) >= AR_WPNAV_EXACT_POS_LOSS_MS) {
                enter_exact_fault(ExactPivotFault::Estimator);
            }
            return;
        }
        _exact_position_loss_active = false;
        _exact_position_loss_start_ms = 0U;
        if (exact_spin) {
            update_distance_and_bearing_to_destination();
            update_steering_and_speed(current_loc, dt);
            return;
        }
    }

    // exit immediately if no current location, origin, destination or speed
    Location current_loc;
    float speed;
    if (!soft_armed || !_orig_and_dest_valid || !AP::ahrs().get_location(current_loc) || !_atc.get_forward_speed(speed)) {
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _cross_track_error = 0;
        return;
    }

    // if no recent calls initialise desired_speed_limited to current speed
    if (!is_active()) {
        _desired_speed_limited = speed;
    }
    _last_update_ms = AP_HAL::millis();

    update_distance_and_bearing_to_destination();

    // handle change in max speed
    update_speed_max();

    if (_exact_phase == ExactPivotPhase::CapturePath) {
        update_exact_capture_path(current_loc, dt);
        return;
    }

    // advance target along path unless vehicle is pivoting
    if (!_pivot.active()) {
        switch (_nav_control_type) {
        case NavControllerType::NAV_SCURVE:
            advance_wp_target_along_track(current_loc, dt);
            break;
        case NavControllerType::NAV_PSC_INPUT_SHAPING:
            update_psc_input_shaping(dt);
            break;
        }
    }

    // A terminal Move may atomically install a bounded CapturePath.  Start it
    // in this same navigation cycle rather than emitting a stale endpoint
    // target for one more cycle.
    if (_exact_phase == ExactPivotPhase::CapturePath) {
        update_exact_capture_path(current_loc, dt);
        return;
    }

    // update_steering_and_speed
    update_steering_and_speed(current_loc, dt);
}

void AR_WPNav::set_reversed(bool reversed)
{
    if (_reversed == reversed) {
        return;
    }

    // A planned transaction locks its direction because both the cached path
    // and spin heading were calculated for it. Reject a mid-transaction
    // direction request without destroying the still-valid route. The owner
    // must replace the navigation transaction before changing direction.
    if ((_exact_phase != ExactPivotPhase::None) ||
        _completion_event_pending) {
        return;
    }
    clear_promoted_path_constraints();
    _reversed = reversed;
}

// set maximum speed in m/s.  returns true on success
// this should not be called at more than 3hz or else SCurve path planning may not advance properly
bool AR_WPNav::set_speed_max(float speed_max)
{
    // range check target speed
    if (speed_max < AR_WPNAV_SPEED_MIN) {
        return false;
    }

    _base_speed_max = speed_max;
    return true;
}

// set speed nudge in m/s.  this will have no effect unless nudge_speed_max > speed_max
// nudge_speed_max should always be positive regardless of whether the vehicle is travelling forward or reversing
void AR_WPNav::set_nudge_speed_max(float nudge_speed_max)
{
    _nudge_speed_max = nudge_speed_max;
}

// set desired location and (optionally) next_destination
// next_destination should be provided if known to allow smooth cornering
bool AR_WPNav::set_desired_location(const Location& destination, Location next_destination)
{
    uint32_t acknowledged_generation = 0U;
    if (atomic_handoff_ack_matches(destination, acknowledged_generation)) {
        if (!attach_ordinary_next_leg(destination, next_destination)) {
            // A promoted Exact leg is already the active navigation contract.
            // Failure to convert the following preview must latch Hold instead
            // of returning a start-command failure that AP_Mission may skip.
            enter_exact_fault(ExactPivotFault::Estimator);
            return false;
        }
        if (!_completion_event_pending ||
            (acknowledged_generation != _handoff_generation)) {
            enter_exact_fault(ExactPivotFault::Internal);
            return false;
        }
        _completion_event_pending = false;
        clear_exact_pivot_after_ack();
        _exact_diag.acknowledged_generation = acknowledged_generation;
        mark_exact_pivot_diag_event(DiagAckOrdinary);
        return true;
    }

    // An ordinary target mismatch is a mission change.  Cancel the atomic
    // transaction and use the normal safe full-replan path below.
    const bool replacing_exact_sequence = _completion_event_pending ||
                                          (_exact_phase != ExactPivotPhase::None);
    const ExactPivotFault replaced_exact_fault = _exact_fault;
    if (replacing_exact_sequence) {
        _pivot.deactivate();
        clear_exact_pivot();
    }

    const bool continue_from_previous_fast = _fast_waypoint && !_scurve_next_leg.finished();
    const bool force_entry_pivot = _pivot_at_next_wp;

    // re-initialise if inactive, previous destination has been interrupted or different controller was used
    if (!is_active() || !_reached_destination || (_nav_control_type != NavControllerType::NAV_SCURVE)) {
        if (!set_origin_and_destination_to_stopping_point()) {
            if (replacing_exact_sequence) {
                enter_exact_fault(replaced_exact_fault != ExactPivotFault::None ?
                                  replaced_exact_fault : ExactPivotFault::Internal);
            }
            return false;
        }
        // clear scurves
        _scurve_prev_leg.init();
        _scurve_this_leg.init();
        _scurve_next_leg.init();
    }

    // shift this leg to previous leg
    _scurve_prev_leg = _scurve_this_leg;

    // initialise some variables
    _origin = _destination;
    _destination = destination;
    _orig_and_dest_valid = true;
    _reached_destination = false;

    update_distance_and_bearing_to_destination();

    // convert origin and destination to offset from EKF origin
    Vector2f origin_NE;
    Vector2f destination_NE;
    if (!_origin.get_vector_xy_from_origin_NE(origin_NE) ||
        !_destination.get_vector_xy_from_origin_NE(destination_NE)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        if (replacing_exact_sequence) {
            enter_exact_fault(replaced_exact_fault != ExactPivotFault::None ?
                              replaced_exact_fault : ExactPivotFault::Internal);
        }
        return false;
    }
    origin_NE *= 0.01f;
    destination_NE *= 0.01f;

    // calculate track to destination
    if (continue_from_previous_fast) {
        // skip recalculating this leg by simply shifting next leg
        _scurve_this_leg = _scurve_next_leg;
    } else {
        _track_scalar_dt = 1.0f;
        _scurve_this_leg.calculate_track(Vector3f{origin_NE.x, origin_NE.y, 0.0f},              // origin
                                         Vector3f{destination_NE.x, destination_NE.y, 0.0f},    // destination
                                         _pos_control.get_speed_max(),
                                         _pos_control.get_speed_max(),  // speed up (not used)
                                         _pos_control.get_speed_max(),  // speed down (not used)
                                         _pos_control.get_accel_max(),  // forward back acceleration
                                         _pos_control.get_accel_max(),  // vertical accel (not used)
                                         AR_WPNAV_SNAP_MAX,             // snap
                                         _pos_control.get_jerk_max());
    }

    // handle next destination
    _scurve_next_leg.init();
    _fast_waypoint = false;
    _pivot_at_next_wp = false;
    if (next_destination.initialised()) {
        // check if vehicle should pivot at next waypoint
        const float next_wp_yaw_change = get_corner_angle(_origin, destination, next_destination);
        _pivot_at_next_wp = _pivot.would_activate(next_wp_yaw_change);
        if (!_pivot_at_next_wp) {
            // convert next_destination to offset from EKF origin
            Vector2f next_destination_NE;
            if (!next_destination.get_vector_xy_from_origin_NE(next_destination_NE)) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                if (replacing_exact_sequence) {
                    enter_exact_fault(replaced_exact_fault != ExactPivotFault::None ?
                                      replaced_exact_fault : ExactPivotFault::Internal);
                }
                return false;
            }
            next_destination_NE *= 0.01f;
            _scurve_next_leg.calculate_track(Vector3f{destination_NE.x, destination_NE.y, 0.0f},
                                             Vector3f{next_destination_NE.x, next_destination_NE.y, 0.0f},
                                             _pos_control.get_speed_max(),
                                             _pos_control.get_speed_max(),  // speed up (not used)
                                             _pos_control.get_speed_max(),  // speed down (not used)
                                             _pos_control.get_accel_max(),  // forward back acceleration
                                             _pos_control.get_accel_max(),  // vertical accel (not used)
                                             AR_WPNAV_SNAP_MAX,             // snap
                                             _pos_control.get_jerk_max());

            // next destination provided so fast waypoint
            _fast_waypoint = true;
        }
    }

    // scurves used for navigation to destination
    _pivot.deactivate();
    _pivot_handoff = false;
    _pivot_heading_valid = false;
    clear_exact_pivot();
    _nav_control_type = NavControllerType::NAV_SCURVE;

    // check if vehicle should pivot if vehicle stopped at previous waypoint
    // or journey to previous waypoint was interrupted or navigation has just started
    if (!continue_from_previous_fast) {
        const float pivot_heading_cd = _reversed ? wrap_360_cd(oa_wp_bearing_cd() + 18000) : oa_wp_bearing_cd();
        _pivot.check_activation(pivot_heading_cd * 0.01f, force_entry_pivot);
        if (_pivot.active()) {
            _pivot_heading_cd = pivot_heading_cd;
            _pivot_heading_valid = true;
        }
    }

    update_distance_and_bearing_to_destination();

    return true;
}

bool AR_WPNav::set_desired_location_exact_pivot(const Location &destination, const Location &next_destination)
{
    if (!destination.initialised()) {
        return false;
    }

    uint32_t acknowledged_generation = 0U;
    if (atomic_handoff_ack_matches(destination, acknowledged_generation)) {
        if (!next_destination.initialised() ||
            destination.same_latlon_as(next_destination) ||
            !_pivot.available()) {
            enter_exact_fault(ExactPivotFault::Internal);
            return false;
        }

        SCurve next_leg;
        if (!calculate_stopping_scurve(destination, next_destination, next_leg)) {
            Vector2f destination_ne_cm;
            Vector2f next_destination_ne_cm;
            const bool coordinates_valid =
                destination.get_vector_xy_from_origin_NE(destination_ne_cm) &&
                next_destination.get_vector_xy_from_origin_NE(next_destination_ne_cm);
            enter_exact_fault(coordinates_valid ? ExactPivotFault::Internal :
                                                  ExactPivotFault::Estimator);
            return false;
        }
        if (!_completion_event_pending ||
            (acknowledged_generation != _handoff_generation)) {
            enter_exact_fault(ExactPivotFault::Internal);
            return false;
        }

        _scurve_next_leg = next_leg;
        _next_destination = next_destination;
        _exact_next_destination = next_destination;
        // The promoted D0-D1 leg is already active. Preserve D0 as the
        // incoming-line origin for the strict capture and drift reference at D1.
        _exact_frozen_origin = _origin;
        const float forward_heading_cd = destination.get_bearing_to(next_destination);
        _exact_pivot_heading_cd = _exact_reversed ? wrap_360_cd(forward_heading_cd + 18000.0f) : forward_heading_cd;
        _completion_event_pending = false;
        _completed_destination = Location();
        _active_destination = destination;
        _exact_phase = ExactPivotPhase::Move;
        _exact_turn_direction = 0;
        _exact_capture_attempted = false;
        clear_exact_capture_path();
        _fast_waypoint = false;
        _pivot_at_next_wp = false;
        _reached_destination = false;
        _exact_timeout_active = false;
        _exact_timeout_start_ms = 0U;
        _exact_diag.acknowledged_generation = acknowledged_generation;
        start_exact_pivot_diag_leg(DiagAckExact);
        return true;
    }

    if (!next_destination.initialised() ||
        destination.same_latlon_as(next_destination) ||
        !_pivot.available()) {
        return false;
    }

    // An Exact owner must never silently accept a route mutation while any
    // transaction phase is active.  The caller supplies the policy: AUTO
    // falls back to the ordinary safe-replan setter, while Patrol faults.
    if (_completion_event_pending ||
        (_exact_phase != ExactPivotPhase::None)) {
        return false;
    }

    // Build all geometry before changing the live navigation transaction.  This
    // keeps the active route usable if the EKF origin cannot convert a point.
    SCurve next_leg;
    if (!calculate_stopping_scurve(destination, next_destination, next_leg)) {
        return false;
    }

    Location new_origin;
    SCurve this_leg;
    bool zero_length_plan = false;
    if (!is_active() || !_reached_destination || (_nav_control_type != NavControllerType::NAV_SCURVE)) {
        if (!get_stopping_location(new_origin)) {
            return false;
        }
    } else {
        new_origin = _destination;
    }
    // Treat sub-centimetre geodetic quantisation as an empty MOVE without
    // accepting a genuinely short patrol leg as already complete.
    zero_length_plan = new_origin.get_distance(destination) < 0.01f;
    if (!zero_length_plan && !calculate_stopping_scurve(new_origin, destination, this_leg)) {
        return false;
    }

    _origin = new_origin;
    _destination = destination;
    _scurve_prev_leg.init();
    _scurve_this_leg = this_leg;
    _scurve_next_leg = next_leg;
    _track_scalar_dt = 1.0f;
    _next_destination = next_destination;
    _exact_next_destination = next_destination;
    // Freeze the nominal incoming line independently from private capture-path
    // geometry so recovery can never rewrite the mission route.
    _exact_frozen_origin = new_origin;
    const float forward_pivot_heading_cd = destination.get_bearing_to(next_destination);
    _exact_pivot_heading_cd = _reversed ? wrap_360_cd(forward_pivot_heading_cd + 18000.0f) : forward_pivot_heading_cd;
    _exact_phase = ExactPivotPhase::Move;
    _exact_reversed = _reversed;
    _exact_turn_direction = 0;
    _exact_capture_attempted = false;
    clear_exact_capture_path();
    clear_promoted_path_constraints();
    _reached_destination = false;
    _orig_and_dest_valid = true;
    _nav_control_type = NavControllerType::NAV_SCURVE;
    update_distance_and_bearing_to_destination();
    _fast_waypoint = false;
    _pivot_at_next_wp = false;
    _pivot.deactivate();
    _pivot_heading_valid = false;
    _completion_event_pending = false;
    _completed_destination = Location();
    _active_destination = destination;
    _exact_drift_active = false;
    _exact_drift_start_ms = 0U;
    _exact_timeout_active = false;
    _exact_timeout_start_ms = 0U;
    if (!zero_length_plan) {
        // Initial start/resume still needs the normal entry pivot toward the
        // current MOVE leg.  A promoted preview has just completed this turn,
        // so reactivating it there would introduce a duplicate stop.
        const float forward_entry_heading_cd = _origin.get_bearing_to(_destination);
        const float entry_heading_cd = _reversed ? wrap_360_cd(forward_entry_heading_cd + 18000.0f) : forward_entry_heading_cd;
        _pivot.check_activation(entry_heading_cd * 0.01f);
        if (_pivot.active()) {
            _pivot_heading_cd = entry_heading_cd;
            _pivot_heading_valid = true;
            start_exact_pivot_timeout(AP_HAL::millis());
        }
    }
    start_exact_pivot_diag_leg(DiagNewExact);
    return true;
}

bool AR_WPNav::would_pivot_at_destination(const Location &destination,
                                          const Location &next_destination) const
{
    if (!destination.initialised() || !next_destination.initialised() ||
        destination.same_latlon_as(next_destination)) {
        return false;
    }

    const Location &incoming_origin = _completion_event_pending ? _origin : _destination;
    return _pivot.would_activate(get_corner_angle(incoming_origin,
                                                   destination,
                                                   next_destination));
}

// set desired location to a reasonable stopping point, return true on success
bool AR_WPNav::set_desired_location_to_stopping_location()
{
    Location stopping_loc;
    if (!get_stopping_location(stopping_loc)) {
        return false;
    }
    return set_desired_location(stopping_loc);
}

// set desired location as offset from the EKF origin, return true on success
bool AR_WPNav::set_desired_location_NED(const Vector3f& destination)
{
    // initialise destination to ekf origin
    Location destination_ned;
    if (!AP::ahrs().get_origin(destination_ned)) {
        return false;
    }

    // apply offset
    destination_ned.offset(destination.x, destination.y);
    return set_desired_location(destination_ned);
}

bool AR_WPNav::set_desired_location_NED(const Vector3f &destination, const Vector3f &next_destination)
{
    // initialise destination to ekf origin
    Location dest_loc, next_dest_loc;
    if (!AP::ahrs().get_origin(dest_loc)) {
        return false;
    }
    next_dest_loc = dest_loc;

    // apply offsets
    dest_loc.offset(destination.x, destination.y);
    next_dest_loc.offset(next_destination.x, next_destination.y);
    return set_desired_location(dest_loc, next_dest_loc);
}

// set desired location but expect the destination to be updated again in the near future
// position controller input shaping will be used for navigation instead of scurves
// Note: object avoidance is not supported if this method is used
bool AR_WPNav::set_desired_location_expect_fast_update(const Location &destination)
{
    // initialise if not active
    if (!is_active() || (_nav_control_type != NavControllerType::NAV_PSC_INPUT_SHAPING)) {
        if (!set_origin_and_destination_to_stopping_point()) {
            return false;
        }
    }

    // initialise some variables
    _origin = _destination;
    _destination = destination;
    _orig_and_dest_valid = true;
    _reached_destination = false;

    update_distance_and_bearing_to_destination();

    // check if vehicle should pivot
    _pivot.deactivate();
    _pivot_handoff = false;
    _pivot_heading_valid = false;
    clear_exact_pivot();
    const float pivot_heading_cd = get_pivot_target_heading_cd();
    _pivot.check_activation(pivot_heading_cd * 0.01f);
    if (_pivot.active()) {
        _pivot_heading_cd = pivot_heading_cd;
        _pivot_heading_valid = true;
    }

    // position controller input shaping used for navigation to destination
    _nav_control_type = NavControllerType::NAV_PSC_INPUT_SHAPING;
    return true;
}

// calculate vehicle stopping point using current location, velocity and maximum acceleration
bool AR_WPNav::get_stopping_location(Location& stopping_loc)
{
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return false;
    }

    // get current velocity vector and speed
    const Vector2f velocity = AP::ahrs().groundspeed_vector();
    const float speed = velocity.length();

    // avoid divide by zero
    if (!is_positive(speed)) {
        stopping_loc = current_loc;
        return true;
    }

    // get stopping distance in meters
    const float stopping_dist = _atc.get_stopping_distance(speed);

    // calculate stopping position from current location in meters
    const Vector2f stopping_offset = velocity.normalized() * stopping_dist;
    stopping_loc = current_loc;
    stopping_loc.offset(stopping_offset.x, stopping_offset.y);

    return true;
}

// true if update has been called recently
bool AR_WPNav::is_active() const
{
    return ((AP_HAL::millis() - _last_update_ms) < AR_WPNAV_TIMEOUT_MS);
}

// move target location along track from origin to destination using SCurves navigation
void AR_WPNav::advance_wp_target_along_track(const Location &current_loc, float dt)
{
    // exit immediately if no current location, destination or disarmed
    Vector2f curr_pos_NE;
    Vector3f curr_vel_NED;
    if (!AP::ahrs().get_relative_position_NE_origin(curr_pos_NE) || !AP::ahrs().get_velocity_NED(curr_vel_NED)) {
        return;
    }

    // exit immediately if we can't convert waypoint origin to offset from ekf origin
    Vector2f origin_NE;
    if (!_origin.get_vector_xy_from_origin_NE(origin_NE)) {
        return;
    }
    // convert from cm to meters
    origin_NE *= 0.01f;

    // use _track_scalar_dt to slow down S-Curve time to prevent target moving too far in front of vehicle
    Vector2f curr_target_vel = _pos_control.get_desired_velocity();
    float track_scaler_dt = 1.0f;
    if (is_positive(curr_target_vel.length())) {
        Vector2f track_direction = curr_target_vel.normalized();
        const float track_error = _pos_control.get_pos_error().tofloat().dot(track_direction);
        float track_velocity = curr_vel_NED.xy().dot(track_direction);
        // set time scaler to be consistent with the achievable vehicle speed with a 5% buffer for short term variation.
        const float time_scaler_dt_max = _overspeed_enabled ? AR_WPNAV_OVERSPEED_RATIO_MAX : 1.0f;
        track_scaler_dt = constrain_float(0.05f + (track_velocity - _pos_control.get_pos_p().kP() * track_error) / curr_target_vel.length(), 0.0f, time_scaler_dt_max);
    }
    // change s-curve time speed with a time constant of maximum acceleration / maximum jerk
    float track_scaler_tc = 1.0f;
    if (is_positive(_pos_control.get_jerk_max())) {
        track_scaler_tc = _pos_control.get_accel_max() / _pos_control.get_jerk_max();
    }
    _track_scalar_dt += (track_scaler_dt - _track_scalar_dt) * (dt / track_scaler_tc);

    // target position, velocity and acceleration from straight line or spline calculators
    Vector3f target_pos_3d_ftype{origin_NE.x, origin_NE.y, 0.0f};
    Vector3f target_vel, target_accel;

    // update target position, velocity and acceleration
    const float wp_radius = MAX(_radius, _turn_radius);
    bool s_finished = _scurve_this_leg.advance_target_along_track(_scurve_prev_leg, _scurve_next_leg, wp_radius, _pos_control.get_lat_accel_max(), _fast_waypoint, _track_scalar_dt * dt, target_pos_3d_ftype, target_vel, target_accel);

    // A promoted route remains the original frozen D0-D1 line.  If the rover
    // slid ahead of D0 while spinning, only the position target's along-track
    // component is advanced; velocity and acceleration stay from the S-curve.
    apply_forward_rejoin(origin_NE, target_pos_3d_ftype);

    // pass new target to the position controller
    init_pos_control_if_necessary();
    Vector2p target_pos_ptype{target_pos_3d_ftype.x, target_pos_3d_ftype.y};
    _pos_control.set_pos_vel_accel_target(target_pos_ptype, target_vel.xy(), target_accel.xy());

    // A planned differential-drive pivot may discard only the low-speed tail
    // of its stopping S-curve. Both the realtime body origin and the planned
    // target must already be inside the strict endpoint circle.
    if (!_reached_destination && (_exact_phase == ExactPivotPhase::Move)) {
        const uint32_t now_ms = AP_HAL::millis();
        Vector2f destination_ne_cm;
        if (!_destination.get_vector_xy_from_origin_NE(destination_ne_cm)) {
            enter_exact_fault(ExactPivotFault::Internal);
            return;
        }
        destination_ne_cm *= 0.01f;
        const float planned_distance_m = (target_pos_3d_ftype.xy() - destination_ne_cm).length();
        _exact_diag.plan_valid = isfinite(planned_distance_m) && isfinite(target_vel.length());
        _exact_diag.planned_distance_m = planned_distance_m;
        _exact_diag.planned_speed_mps = target_vel.length();
        _exact_diag.path_terminal = s_finished;
        const bool endpoint_captured = exact_endpoint_captured(current_loc,
                                                               planned_distance_m,
                                                               s_finished,
                                                               target_vel.length());
        if (endpoint_captured) {
            _exact_diag.spin_source = 'A';
            mark_exact_pivot_diag_event(DiagEarlyCaptureA);
            _exact_turn_direction = select_exact_turn_direction();
            const bool pivot_activated = _pivot.activate_planned(_exact_turn_direction);
            switch (AR_WPNavDifferential::resolve_move(true, pivot_activated)) {
            case AR_WPNavDifferential::MoveAction::Continue:
                break;
            case AR_WPNavDifferential::MoveAction::BeginSpin:
                enter_exact_spin();
                break;
            case AR_WPNavDifferential::MoveAction::Fault:
                enter_exact_fault(ExactPivotFault::Internal);
                break;
            }
            return;
        }

        if (s_finished) {
            // A long ordinary Move is not timed. The bounded transaction starts
            // only when the path is terminal and a local capture is required.
            start_exact_pivot_timeout(now_ms);
            if (!start_exact_capture_path(current_loc) &&
                (_exact_phase != ExactPivotPhase::Fault)) {
                enter_exact_fault(ExactPivotFault::Internal);
            }
        }
        return;
    }

    // check if we've reached an ordinary waypoint
    if (!_reached_destination && s_finished) {
        // "fast" waypoints are complete once the intermediate point reaches the destination
        if (_fast_waypoint) {
            _reached_destination = true;
        } else {
            // regular waypoints also require the vehicle to be within the waypoint radius or past the "finish line"
            const bool near_wp = current_loc.get_distance(_destination) <= _radius;
            const bool past_wp = current_loc.past_interval_finish_line(_origin, _destination);
            _reached_destination = near_wp || past_wp;
        }
    }
}

// update psc input shaping navigation controller
void AR_WPNav::update_psc_input_shaping(float dt)
{
    // convert destination location to offset from EKF origin (in meters)
    Vector2f pos_target_cm;
    if (!_destination.get_vector_xy_from_origin_NE(pos_target_cm)) {
        return;
    }

    // initialise position controller if not called recently
    init_pos_control_if_necessary();

    // convert to meters and update target
    const Vector2p pos_target = pos_target_cm.topostype() * 0.01;
    _pos_control.input_pos_target(pos_target, dt);

    // update reached_destination
    if (!_reached_destination) {
        // calculate position difference between destination and position controller input shaped target
        Vector2p pos_target_diff = pos_target - _pos_control.get_pos_target();
        // vehicle has reached destination when the target is within 1cm of the destination and vehicle is within waypoint radius
        _reached_destination = (pos_target_diff.length_squared() < sq(0.01)) && (_pos_control.get_pos_error().length_squared() < sq(_radius));
    }
}

// update distance from vehicle's current position to destination
void AR_WPNav::update_distance_and_bearing_to_destination()
{
    // if no current location leave distance unchanged
    Location current_loc;
    if (!_orig_and_dest_valid || !AP::ahrs().get_location(current_loc)) {
        _distance_to_destination = 0.0f;
        _wp_bearing_cd = 0.0f;
        return;
    }
    _distance_to_destination = current_loc.get_distance(_destination);
    _wp_bearing_cd = current_loc.get_bearing_to(_destination);
}

// calculate steering and speed to drive along line from origin to destination waypoint
void AR_WPNav::update_steering_and_speed(const Location &current_loc, float dt)
{
    _cross_track_error = calc_crosstrack_error(current_loc);

    if (_exact_phase == ExactPivotPhase::Spin) {
        const uint32_t now_ms = AP_HAL::millis();
        if (exact_pivot_timed_out(now_ms)) {
            enter_exact_fault(ExactPivotFault::Timeout);
            return;
        }

        // Position safety owns completion. A short excursion only inhibits
        // completion; a continuous excursion becomes a visible fault. Never
        // reconstruct a realtime P-to-D0 point target from Spin.
        if (!exact_spin_position_within_limit(current_loc)) {
            _pivot.reset_completion();
            if (is_positive(_pivot_drift)) {
                if (!_exact_drift_active) {
                    _exact_drift_active = true;
                    _exact_drift_start_ms = now_ms;
                }
                if ((now_ms - _exact_drift_start_ms) < AR_WPNAV_EXACT_DRIFT_MS) {
                    _desired_speed_limited = 0.0f;
                    _desired_lat_accel = 0.0f;
                    _desired_turn_rate_rads = _pivot.active() ?
                        _pivot.get_turn_rate_rads(_exact_pivot_heading_cd * 0.01f) : 0.0f;
                    return;
                }
            }
            _exact_drift_active = false;
            _exact_drift_start_ms = 0U;
            enter_exact_fault(ExactPivotFault::RecoveryBounds);
            return;
        }
        _exact_drift_active = false;
        _exact_drift_start_ms = 0U;

        _desired_heading_cd = get_pivot_target_heading_cd();
        const bool controller_active_before_update = _pivot.active();
        float requested_turn_rate_rads = 0.0f;
        bool completion_edge = false;

        if (controller_active_before_update) {
            float yaw_reset_delta;
            const uint32_t yaw_reset_ms = AP::ahrs().getLastYawResetAngle(yaw_reset_delta);
            if ((yaw_reset_ms != 0U) && (yaw_reset_ms != _last_yaw_reset_ms)) {
                _last_yaw_reset_ms = yaw_reset_ms;
                _pivot.handle_yaw_reset();
            }

            const float exit_angle_deg = constrain_float(_pivot_exit, 0.0f, 45.0f);
            if (is_positive(exit_angle_deg)) {
                // Keep the locked-heading controller live up to the atomic
                // boundary.  Moving its zero-rate point out to PIVOT_EXIT can
                // stop the rover just outside that boundary and recreate a
                // navigation-layer wait.
                requested_turn_rate_rads = _pivot.get_turn_rate_rads(_desired_heading_cd * 0.01f);
                completion_edge = get_pivot_heading_error_deg() <= exit_angle_deg;
            } else {
                requested_turn_rate_rads = _pivot.get_turn_rate_rads(_desired_heading_cd * 0.01f);
                const float yaw_rate_rads = AP::ahrs().get_yaw_rate_earth();
                if (!isfinite(yaw_rate_rads)) {
                    enter_exact_fault(ExactPivotFault::Estimator);
                    return;
                }
                completion_edge = _pivot.update_completion(_desired_heading_cd * 0.01f,
                                                           yaw_rate_rads,
                                                           now_ms);
            }
        }

        const auto output = AR_WPNavDifferential::resolve_spin(controller_active_before_update,
                                                               completion_edge,
                                                               requested_turn_rate_rads);
        _desired_speed_limited = 0.0f;
        _desired_turn_rate_rads = output.turn_rate_rads;
        _desired_lat_accel = 0.0f;

        switch (output.action) {
        case AR_WPNavDifferential::SpinAction::Continue:
            break;
        case AR_WPNavDifferential::SpinAction::Complete:
            if (!promote_exact_preview(current_loc)) {
                enter_exact_fault(ExactPivotFault::Internal);
                break;
            }
            // Promotion and Path output happen in this very navigation cycle.
            advance_wp_target_along_track(current_loc, dt);
            update_path_outputs(dt);
            break;
        case AR_WPNavDifferential::SpinAction::Fault:
            enter_exact_fault(ExactPivotFault::Internal);
            break;
        }
        return;
    }

    if (_exact_phase == ExactPivotPhase::Fault) {
        _desired_speed_limited = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _desired_lat_accel = 0.0f;
        return;
    }

    // Preserve the established entry policy for ordinary AUTO/Guided/RTL
    // pivots.  Only an explicitly planned Exact transition owns a direct
    // endpoint-to-SPIN handoff; an interrupted or dynamic target must first
    // finish the normal navigation-layer deceleration.
    if (_pivot.active()) {
        _desired_heading_cd = get_pivot_target_heading_cd();
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;

        if (!is_zero(_desired_speed_limited)) {
            _desired_turn_rate_rads = 0.0f;
            _pivot.reset_completion();
            return;
        }

        _desired_turn_rate_rads = _pivot.get_turn_rate_rads(_desired_heading_cd * 0.01f);
        const float yaw_rate_rads = AP::ahrs().get_yaw_rate_earth();
        const bool completion_edge = isfinite(yaw_rate_rads) &&
                                     _pivot.update_completion(_desired_heading_cd * 0.01f,
                                                              yaw_rate_rads,
                                                              AP_HAL::millis());
        if (completion_edge) {
            _desired_turn_rate_rads = 0.0f;
            _pivot_handoff = true;
            _pivot_heading_valid = false;
            if (_exact_phase == ExactPivotPhase::Move) {
                // This was the initial entry alignment. Its timeout must not
                // remain armed over the following, potentially long, MOVE.
                _exact_timeout_active = false;
                _exact_timeout_start_ms = 0U;
            }
        } else if (!isfinite(yaw_rate_rads)) {
            _desired_turn_rate_rads = 0.0f;
        }
        return;
    }

    // update position controller only while translation owns the vehicle
    _pos_control.set_reversed(_reversed);
    update_path_outputs(dt);
}

// settor to allow vehicle code to provide turn related param values to this library (should be updated regularly)
void AR_WPNav::set_turn_params(float turn_radius, bool pivot_possible)
{
    _turn_radius = pivot_possible ? 0.0 : turn_radius;
    _pivot.enable(pivot_possible);
}

// return the absolute heading error used by the pivot controller
float AR_WPNav::get_pivot_heading_error_deg() const
{
    const float target_heading_cd = get_pivot_target_heading_cd();
    return fabsf(wrap_180(target_heading_cd * 0.01f - AP::ahrs().yaw_sensor * 0.01f));
}

AR_WPNav::MotionPrimitive AR_WPNav::get_motion_primitive() const
{
    if ((_exact_phase != ExactPivotPhase::None) &&
        _exact_position_loss_active) {
        return MotionPrimitive::Hold;
    }

    // Direct common-throttle zero is reserved for an explicitly planned Exact
    // SPIN.  Ordinary AUTO/Guided/RTL pivots stay on the established Path
    // throttle controller even after their speed target reaches zero.
    if (_exact_phase == ExactPivotPhase::Spin) {
        return MotionPrimitive::Spin;
    }
    if (_exact_phase == ExactPivotPhase::Fault) {
        return MotionPrimitive::Hold;
    }
    return MotionPrimitive::Path;
}

AR_WPNav::ExactPivotDiagSnapshot AR_WPNav::get_exact_pivot_diag_snapshot() const
{
    ExactPivotDiagSnapshot snapshot;
    snapshot.phase = exact_pivot_phase_to_diag(_exact_phase);
    snapshot.primitive = static_cast<uint8_t>(get_motion_primitive());
    snapshot.fault = static_cast<uint8_t>(_exact_fault);
    snapshot.fault_from_phase = _exact_diag.fault_from_phase;
    snapshot.transition_flags = _exact_diag.transition_flags;
    snapshot.transition_sequence = _exact_diag.transition_sequence;
    snapshot.leg_id = _exact_diag.leg_id;
    snapshot.handoff_generation = _handoff_generation;
    snapshot.acknowledged_generation = _exact_diag.acknowledged_generation;
    snapshot.endpoint_distance_m = _exact_diag.endpoint_distance_m;
    snapshot.planned_distance_m = _exact_diag.planned_distance_m;
    snapshot.planned_speed_mps = _exact_diag.planned_speed_mps;
    snapshot.desired_speed_mps = _desired_speed_limited;
    snapshot.desired_turn_rate_rads = _desired_turn_rate_rads;
    snapshot.xtrack_error_m = _cross_track_error;
    snapshot.target_heading_deg = _exact_diag.target_heading_deg;
    snapshot.spin_source = _exact_diag.spin_source;
    snapshot.turn_direction = _exact_turn_direction;
    snapshot.oa_cancel_from_phase = _exact_diag.oa_cancel_from_phase;
    snapshot.oa_cancel_reason = _exact_diag.oa_cancel_reason;
    snapshot.capture = _exact_diag.capture;
    snapshot.handoff = _exact_diag.handoff;

    uint16_t state_flags = 0U;
    if (_exact_phase != ExactPivotPhase::None) {
        state_flags |= DiagStateExactActive;
    }
    if (_exact_diag.endpoint_valid) {
        state_flags |= DiagStateEndpointValid;
    }
    if (_exact_diag.plan_valid) {
        state_flags |= DiagStatePlanValid;
    }
    if (_exact_diag.path_terminal) {
        state_flags |= DiagStatePathTerminal;
    }
    if (_completion_event_pending) {
        state_flags |= DiagStateCompletion;
    }
    if (_pivot.active()) {
        state_flags |= DiagStatePivotActive;
    }
    if ((_exact_phase == ExactPivotPhase::CapturePath) &&
        _exact_capture_path.valid) {
        state_flags |= DiagStateCaptureActive;
        if (_exact_diag.capture.valid && (_exact_diag.capture.segment == 1U)) {
            state_flags |= DiagStateCaptureArc;
        }
    }
    if (_rejoin_active) {
        state_flags |= DiagStateRejoinActive;
    }
    if (_heading_handoff.active) {
        state_flags |= DiagStateHandoffActive;
    }
    if (_exact_timeout_active) {
        state_flags |= DiagStateTimeoutActive;
    }
    if (_exact_position_loss_active) {
        state_flags |= DiagStatePositionLoss;
    }
    if (_exact_drift_active) {
        state_flags |= DiagStateDriftActive;
    }
    if (_exact_capture_violation_start_ms != 0U) {
        state_flags |= DiagStateCaptureViolation;
    }
    if (_reversed) {
        state_flags |= DiagStateReversed;
    }

    const float yaw_rad = AP::ahrs().get_yaw();
    if (((_exact_diag.transition_flags & DiagHandoffDone) != 0U) &&
        _exact_diag.handoff.valid &&
        isfinite(_exact_diag.handoff.heading_error_deg)) {
        // Preserve the signed error from the handoff completion edge even if
        // an Exact ACK in the same mode cycle has already cached the following
        // endpoint's pivot heading.
        snapshot.yaw_error_deg = _exact_diag.handoff.heading_error_deg;
        state_flags |= DiagStateYawErrorValid;
    } else if (_heading_handoff.active && isfinite(yaw_rad)) {
        snapshot.yaw_error_deg = wrap_180(_heading_handoff.heading_cd * 0.01f -
                                          degrees(yaw_rad));
        state_flags |= DiagStateYawErrorValid;
    } else if (_exact_diag.target_heading_valid && isfinite(yaw_rad)) {
        snapshot.yaw_error_deg = wrap_180(_exact_diag.target_heading_deg -
                                          degrees(yaw_rad));
        state_flags |= DiagStateYawErrorValid;
    }
    snapshot.state_flags = state_flags;
    return snapshot;
}

void AR_WPNav::ack_exact_pivot_diag(uint32_t transition_sequence)
{
    if ((transition_sequence != 0U) &&
        (transition_sequence == _exact_diag.transition_sequence)) {
        _exact_diag.transition_flags = 0U;
    }
}

uint8_t AR_WPNav::exact_pivot_phase_to_diag(ExactPivotPhase phase)
{
    switch (phase) {
    case ExactPivotPhase::None:
        return static_cast<uint8_t>(ExactPivotDiagPhase::None);
    case ExactPivotPhase::Move:
        return static_cast<uint8_t>(ExactPivotDiagPhase::Move);
    case ExactPivotPhase::CapturePath:
        return static_cast<uint8_t>(ExactPivotDiagPhase::CapturePath);
    case ExactPivotPhase::Spin:
        return static_cast<uint8_t>(ExactPivotDiagPhase::Spin);
    case ExactPivotPhase::PromotedMove:
        return static_cast<uint8_t>(ExactPivotDiagPhase::PromotedMove);
    case ExactPivotPhase::Fault:
        return static_cast<uint8_t>(ExactPivotDiagPhase::Fault);
    }
    return static_cast<uint8_t>(ExactPivotDiagPhase::Fault);
}

void AR_WPNav::mark_exact_pivot_diag_event(uint16_t event)
{
    if (event == 0U) {
        return;
    }
    _exact_diag.transition_flags |= event;
    ++_exact_diag.transition_sequence;
    if (_exact_diag.transition_sequence == 0U) {
        ++_exact_diag.transition_sequence;
    }
}

void AR_WPNav::start_exact_pivot_diag_leg(uint16_t event)
{
    ++_exact_diag.leg_id;
    if (_exact_diag.leg_id == 0U) {
        ++_exact_diag.leg_id;
    }
    _exact_diag.endpoint_valid = false;
    _exact_diag.plan_valid = false;
    _exact_diag.path_terminal = false;
    _exact_diag.target_heading_valid = isfinite(_exact_pivot_heading_cd);
    _exact_diag.endpoint_distance_m = 0.0f;
    _exact_diag.planned_distance_m = 0.0f;
    _exact_diag.planned_speed_mps = 0.0f;
    _exact_diag.target_heading_deg = _exact_pivot_heading_cd * 0.01f;
    _exact_diag.spin_source = 0U;
    _exact_diag.fault_from_phase = 0U;
    _exact_diag.oa_cancel_from_phase = 0U;
    _exact_diag.oa_cancel_reason = 0U;
    _exact_diag.capture = ExactPivotCaptureDiag{};
    if (event == DiagNewExact) {
        _exact_diag.handoff = ExactPivotHandoffDiag{};
    }
    mark_exact_pivot_diag_event(event);
}

void AR_WPNav::update_exact_pivot_diag_endpoint(const Location &current_loc)
{
    const float endpoint_distance_m = current_loc.get_distance(_destination);
    _exact_diag.endpoint_valid = isfinite(endpoint_distance_m);
    if (_exact_diag.endpoint_valid) {
        _exact_diag.endpoint_distance_m = endpoint_distance_m;
    }
}

void AR_WPNav::note_exact_pivot_oa_cancel(uint8_t from_phase,
                                         ExactPivotOACancelReason reason)
{
    _exact_diag.oa_cancel_from_phase = from_phase;
    _exact_diag.oa_cancel_reason = static_cast<uint8_t>(reason);
    mark_exact_pivot_diag_event(DiagOACancel);
}

// return the pivot target heading, applying reverse at time of use
float AR_WPNav::get_pivot_target_heading_cd() const
{
    if ((_exact_phase == ExactPivotPhase::Spin) ||
        (_exact_phase == ExactPivotPhase::PromotedMove) ||
        (_exact_phase == ExactPivotPhase::Fault)) {
        return _exact_pivot_heading_cd;
    }
    if (_pivot.active() && _pivot_heading_valid) {
        return _pivot_heading_cd;
    }
    const float forward_heading_cd = oa_wp_bearing_cd();
    return _reversed ? wrap_360_cd(forward_heading_cd + 18000.0f) : forward_heading_cd;
}

void AR_WPNav::clear_exact_pivot()
{
    _exact_phase = ExactPivotPhase::None;
    _exact_fault = ExactPivotFault::None;
    _exact_next_destination = Location();
    _exact_frozen_origin = Location();
    _exact_pivot_heading_cd = 0.0f;
    _last_yaw_reset_ms = 0U;
    _exact_position_loss_active = false;
    _exact_position_loss_start_ms = 0U;
    _exact_drift_active = false;
    _exact_drift_start_ms = 0U;
    _exact_timeout_active = false;
    _exact_timeout_start_ms = 0U;
    _exact_reversed = false;
    _exact_turn_direction = 0;
    _exact_capture_attempted = false;
    clear_exact_capture_path();
    _completion_event_pending = false;
    _completed_destination = Location();
    _active_destination = Location();
    clear_promoted_path_constraints();
}

void AR_WPNav::clear_exact_pivot_after_ack()
{
    _exact_phase = ExactPivotPhase::None;
    _exact_fault = ExactPivotFault::None;
    _exact_next_destination = Location();
    _exact_frozen_origin = Location();
    _exact_pivot_heading_cd = 0.0f;
    _last_yaw_reset_ms = 0U;
    _exact_position_loss_active = false;
    _exact_position_loss_start_ms = 0U;
    _exact_drift_active = false;
    _exact_drift_start_ms = 0U;
    _exact_timeout_active = false;
    _exact_timeout_start_ms = 0U;
    _exact_reversed = false;
    _exact_turn_direction = 0;
    _exact_capture_attempted = false;
    clear_exact_capture_path();
    _completed_destination = Location();
    _active_destination = Location();
    // ACK only attaches the following preview.  The promoted active Path keeps
    // running and is not reinitialised.
}

bool AR_WPNav::atomic_handoff_ack_matches(const Location &destination,
                                          uint32_t &generation) const
{
    if (!_completion_event_pending ||
        !_active_destination.initialised() ||
        !destination.same_latlon_as(_active_destination)) {
        return false;
    }
    generation = _handoff_generation;
    return generation != 0U;
}

bool AR_WPNav::attach_ordinary_next_leg(const Location &destination,
                                        const Location &next_destination)
{
    _scurve_next_leg.init();
    _next_destination = next_destination;
    _fast_waypoint = false;
    _pivot_at_next_wp = false;

    if (!next_destination.initialised()) {
        return true;
    }

    const float next_wp_yaw_change = get_corner_angle(_origin,
                                                       destination,
                                                       next_destination);
    _pivot_at_next_wp = _pivot.would_activate(next_wp_yaw_change);
    if (_pivot_at_next_wp) {
        return true;
    }

    Vector2f destination_ne_cm;
    Vector2f next_destination_ne_cm;
    if (!destination.get_vector_xy_from_origin_NE(destination_ne_cm) ||
        !next_destination.get_vector_xy_from_origin_NE(next_destination_ne_cm)) {
        return false;
    }
    destination_ne_cm *= 0.01f;
    next_destination_ne_cm *= 0.01f;
    _scurve_next_leg.calculate_track(Vector3f{destination_ne_cm.x, destination_ne_cm.y, 0.0f},
                                     Vector3f{next_destination_ne_cm.x, next_destination_ne_cm.y, 0.0f},
                                     _pos_control.get_speed_max(),
                                     _pos_control.get_speed_max(),
                                     _pos_control.get_speed_max(),
                                     _pos_control.get_accel_max(),
                                     _pos_control.get_accel_max(),
                                     AR_WPNAV_SNAP_MAX,
                                     _pos_control.get_jerk_max());
    _fast_waypoint = true;
    return true;
}

bool AR_WPNav::exact_endpoint_captured(const Location &current_loc,
                                       float planned_distance_m,
                                       bool path_terminal,
                                       float planned_speed_mps) const
{
    const float capture_radius_m = constrain_float(_pivot_radius, 0.05f, 2.0f);
    return AR_WPNavDifferential::endpoint_captured({
        path_terminal,
        planned_speed_mps,
        static_cast<float>(current_loc.get_distance(_destination)),
        planned_distance_m,
        capture_radius_m,
        constrain_float(_pivot_capture_speed, 0.0f, 0.5f),
    });
}

float AR_WPNav::get_exact_capture_speed_max(float radius_m) const
{
    const float speed_max_mps = _pos_control.get_speed_max();
    const float lat_accel_max_mpss = _pos_control.get_lat_accel_max();
    const float pivot_turn_rate_max_deg = MAX(_pivot.get_rate_max(), 0.0f);
    const float atc_turn_rate_max_deg = _atc.get_steer_rate_max();
    const float lat_turn_rate_max_rads =
        _atc.get_turn_rate_from_lat_accel(lat_accel_max_mpss,
                                          AR_WPNAV_CAPTURE_MAX_SPEED);
    if (!isfinite(radius_m) || !is_positive(radius_m) ||
        !isfinite(speed_max_mps) || !is_positive(speed_max_mps) ||
        !isfinite(lat_accel_max_mpss) || !is_positive(lat_accel_max_mpss) ||
        !isfinite(pivot_turn_rate_max_deg) ||
        !isfinite(atc_turn_rate_max_deg) ||
        !isfinite(lat_turn_rate_max_rads) || !is_positive(lat_turn_rate_max_rads)) {
        return 0.0f;
    }

    const float physical_lat_speed_max_mps = safe_sqrt(lat_accel_max_mpss * radius_m);
    // Capture runs below the attitude controller's minimum steering-conversion
    // speed. Limit v/R to the turn rate that its public lateral-acceleration
    // conversion can actually request, while retaining the physical v^2/R cap.
    const float atc_lat_speed_max_mps = lat_turn_rate_max_rads * radius_m;
    if (!isfinite(physical_lat_speed_max_mps) || !is_positive(physical_lat_speed_max_mps) ||
        !isfinite(atc_lat_speed_max_mps) || !is_positive(atc_lat_speed_max_mps)) {
        return 0.0f;
    }

    float capture_speed_max_mps = MIN(MIN(speed_max_mps, AR_WPNAV_CAPTURE_MAX_SPEED),
                                      MIN(physical_lat_speed_max_mps,
                                          atc_lat_speed_max_mps));

    float effective_turn_rate_max_deg = 0.0f;
    if (is_positive(pivot_turn_rate_max_deg)) {
        effective_turn_rate_max_deg = pivot_turn_rate_max_deg;
    }
    if (is_positive(atc_turn_rate_max_deg)) {
        effective_turn_rate_max_deg = is_positive(effective_turn_rate_max_deg) ?
            MIN(effective_turn_rate_max_deg, atc_turn_rate_max_deg) :
            atc_turn_rate_max_deg;
    }
    if (is_positive(effective_turn_rate_max_deg)) {
        const float rate_speed_max_mps = radians(effective_turn_rate_max_deg) * radius_m;
        if (!isfinite(rate_speed_max_mps) || !is_positive(rate_speed_max_mps)) {
            return 0.0f;
        }
        capture_speed_max_mps = MIN(capture_speed_max_mps, rate_speed_max_mps);
    }

    return (isfinite(capture_speed_max_mps) && is_positive(capture_speed_max_mps)) ?
        capture_speed_max_mps : 0.0f;
}

float AR_WPNav::exact_crosstrack_error(const Location &current_loc) const
{
    const Location &frozen_origin = _exact_frozen_origin.initialised() ?
        _exact_frozen_origin : _origin;
    const Vector2f track = frozen_origin.get_distance_NE(_destination);
    if (track.length() < 1.0e-6f) {
        return current_loc.get_distance_NE(_destination).length();
    }
    return frozen_origin.get_distance_NE(current_loc) % track.normalized();
}

bool AR_WPNav::exact_spin_position_within_limit(const Location &current_loc) const
{
    const float configured_drift_m = constrain_float(_pivot_drift, 0.0f, 3.0f);
    const float limit_m = is_positive(configured_drift_m) ? configured_drift_m :
        (constrain_float(_pivot_radius, 0.05f, 2.0f) * AR_WPNAV_PIVOT_RELEASE_RATIO);
    return (current_loc.get_distance(_destination) <= limit_m) &&
           (fabsf(exact_crosstrack_error(current_loc)) <= limit_m);
}

bool AR_WPNav::exact_pivot_timed_out(uint32_t now_ms) const
{
    const float timeout_s = constrain_float(_pivot_timeout, 0.0f, 60.0f);
    return _exact_timeout_active && is_positive(timeout_s) &&
           ((now_ms - _exact_timeout_start_ms) >= uint32_t(timeout_s * 1000.0f));
}

void AR_WPNav::start_exact_pivot_timeout(uint32_t now_ms)
{
    if (!_exact_timeout_active) {
        _exact_timeout_active = true;
        // Zero is a valid boot-time timestamp; the active flag owns validity.
        _exact_timeout_start_ms = now_ms;
    }
}

void AR_WPNav::enter_exact_spin()
{
    const ExactPivotPhase previous_phase = _exact_phase;
    clear_promoted_path_constraints();
    clear_exact_capture_path();
    _exact_phase = ExactPivotPhase::Spin;
    _reached_destination = false;
    float yaw_reset_delta;
    _last_yaw_reset_ms = AP::ahrs().getLastYawResetAngle(yaw_reset_delta);
    _exact_position_loss_active = false;
    _exact_position_loss_start_ms = 0U;
    _exact_drift_active = false;
    _exact_drift_start_ms = 0U;
    _exact_capture_violation_start_ms = 0U;
    start_exact_pivot_timeout(AP_HAL::millis());
    _exact_diag.spin_source = (previous_phase == ExactPivotPhase::CapturePath) ? 'B' : 'A';
    mark_exact_pivot_diag_event(DiagEnterSpin);
}

void AR_WPNav::enter_exact_fault(ExactPivotFault reason)
{
    _exact_diag.fault_from_phase = exact_pivot_phase_to_diag(_exact_phase);
    _pivot.deactivate();
    _exact_phase = ExactPivotPhase::Fault;
    _exact_fault = reason;
    _reached_destination = false;
    _completion_event_pending = false;
    _completed_destination = Location();
    _active_destination = Location();
    _desired_speed_limited = 0.0f;
    _desired_turn_rate_rads = 0.0f;
    _desired_lat_accel = 0.0f;
    _exact_position_loss_active = false;
    _exact_position_loss_start_ms = 0U;
    _exact_drift_active = false;
    _exact_drift_start_ms = 0U;
    clear_promoted_path_constraints();
    clear_exact_capture_path();
    mark_exact_pivot_diag_event(DiagFault);
}

int8_t AR_WPNav::select_exact_turn_direction() const
{
    const float target_yaw_rad = radians(_exact_pivot_heading_cd * 0.01f);
    const float yaw_error_rad = wrap_PI(target_yaw_rad - AP::ahrs().get_yaw());
    if (fabsf(fabsf(yaw_error_rad) - M_PI) > radians(1.0f) &&
        !is_zero(yaw_error_rad)) {
        return is_negative(yaw_error_rad) ? -1 : 1;
    }

    const Vector2f incoming = _exact_frozen_origin.get_distance_NE(_destination);
    const Vector2f outgoing = _destination.get_distance_NE(_exact_next_destination);
    const float corner_cross = incoming % outgoing;
    if (!is_zero(corner_cross)) {
        // Positive mathematical rotation in NE is a clockwise/right turn.
        return is_negative(corner_cross) ? -1 : 1;
    }
    // Make a collinear 180 degree tie deterministic.
    return -1;
}

void AR_WPNav::clear_exact_capture_path()
{
    _exact_capture_path = ExactCapturePath{};
    _exact_capture_scurve.init();
    _exact_capture_scurve_aux.init();
    _exact_capture_violation_start_ms = 0U;
}

bool AR_WPNav::start_exact_capture_path(const Location &current_loc)
{
    if (_exact_capture_attempted) {
        enter_exact_fault(ExactPivotFault::RecoveryBounds);
        return false;
    }
    _exact_capture_attempted = true;

    const float capture_radius_m = constrain_float(_pivot_radius, 0.05f, 2.0f);
    const float drift_limit_m = constrain_float(_pivot_drift, 0.0f, 3.0f);
    const float realtime_distance_m = current_loc.get_distance(_destination);
    if (_exact_reversed || !is_positive(drift_limit_m)) {
        enter_exact_fault(ExactPivotFault::RecoveryInfeasible);
        return false;
    }
    if (!isfinite(realtime_distance_m) ||
        (realtime_distance_m <= capture_radius_m) ||
        (realtime_distance_m > drift_limit_m)) {
        enter_exact_fault(ExactPivotFault::RecoveryBounds);
        return false;
    }

    Vector2f current_ne_cm;
    Vector2f destination_ne_cm;
    if (!current_loc.get_vector_xy_from_origin_NE(current_ne_cm) ||
        !_destination.get_vector_xy_from_origin_NE(destination_ne_cm) ||
        !isfinite(AP::ahrs().get_yaw())) {
        enter_exact_fault(ExactPivotFault::Estimator);
        return false;
    }
    const Vector2f current_ne = current_ne_cm * 0.01f;
    const Vector2f destination_ne = destination_ne_cm * 0.01f;
    const float start_heading_rad = AP::ahrs().get_yaw();
    const float target_heading_rad = radians(_exact_pivot_heading_cd * 0.01f);
    const float inner_radius_m = capture_radius_m * AR_WPNAV_CAPTURE_INNER_RATIO;
    const float max_path_length_m = 2.0f * drift_limit_m;
    const float angle_tolerance_rad = radians(0.5f);
    constexpr float distance_tolerance_m = 0.005f;
    const float radius_ratios[] = {0.5f, 0.375f, 0.25f};

    const Vector2f incoming = _exact_frozen_origin.get_distance_NE(_destination);
    const Vector2f outgoing = _destination.get_distance_NE(_exact_next_destination);
    const float corner_cross = incoming % outgoing;
    const int8_t tie_direction = is_zero(corner_cross) ? -1 :
        (is_negative(corner_cross) ? -1 : 1);

    ExactCapturePath best_path;
    float best_length_m = 1.0e30f;
    bool found = false;

    for (const float radius_ratio : radius_ratios) {
        const float turn_radius_m = capture_radius_m * radius_ratio;
        ExactCapturePath radius_best;
        float radius_best_length_m = 1.0e30f;
        bool radius_found = false;

        for (int8_t turn_direction = -1; turn_direction <= 1; turn_direction += 2) {
            const float available_turn_rad = wrap_2PI(turn_direction *
                                                      (target_heading_rad - start_heading_rad));
            if (available_turn_rad > (M_PI + angle_tolerance_rad)) {
                continue;
            }

            const Vector2f heading_unit{cosf(start_heading_rad), sinf(start_heading_rad)};
            Vector2f normal_unit = heading_unit;
            normal_unit.rotate(turn_direction * M_PI_2);
            const Vector2f center_ne = current_ne + normal_unit * turn_radius_m;
            const Vector2f center_to_destination = destination_ne - center_ne;
            const float center_distance_m = center_to_destination.length();
            if (!is_positive(center_distance_m)) {
                continue;
            }

            const float tangent_offsets[] = {
                turn_radius_m - inner_radius_m,
                turn_radius_m + inner_radius_m,
            };
            for (const float tangent_offset_m : tangent_offsets) {
                if (fabsf(tangent_offset_m) >= center_distance_m) {
                    continue;
                }

                const float straight_length_m = safe_sqrt(sq(center_distance_m) -
                                                          sq(tangent_offset_m));
                if (!is_positive(straight_length_m)) {
                    continue;
                }
                const float line_heading_rad = center_to_destination.angle() +
                    turn_direction * atan2f(tangent_offset_m, straight_length_m);
                const Vector2f line_unit{cosf(line_heading_rad), sinf(line_heading_rad)};
                Vector2f line_normal = line_unit;
                line_normal.rotate(M_PI_2);
                const Vector2f tangent_ne = center_ne -
                    line_normal * (turn_direction * turn_radius_m);
                const Vector2f end_ne = tangent_ne + line_unit * straight_length_m;
                if (fabsf((end_ne - destination_ne).length() - inner_radius_m) >
                    distance_tolerance_m) {
                    continue;
                }

                const float arc_angle_rad = wrap_2PI(turn_direction *
                                                     (line_heading_rad - start_heading_rad));
                if ((arc_angle_rad > (available_turn_rad + angle_tolerance_rad)) ||
                    (arc_angle_rad > (M_PI + angle_tolerance_rad))) {
                    continue;
                }
                const float arc_length_m = turn_radius_m * arc_angle_rad;
                const float total_length_m = arc_length_m + straight_length_m;
                if (total_length_m > (max_path_length_m + distance_tolerance_m)) {
                    continue;
                }

                Vector2f start_radial_unit = current_ne - center_ne;
                start_radial_unit /= turn_radius_m;
                Vector2f far_radial_unit = center_ne - destination_ne;
                far_radial_unit /= center_distance_m;
                const float far_arc_angle_rad = wrap_2PI(turn_direction *
                    (far_radial_unit.angle() - start_radial_unit.angle()));
                float max_arc_distance_m = MAX(realtime_distance_m,
                                                (tangent_ne - destination_ne).length());
                if (far_arc_angle_rad <= (arc_angle_rad + angle_tolerance_rad)) {
                    max_arc_distance_m = MAX(max_arc_distance_m,
                                             center_distance_m + turn_radius_m);
                }
                const float max_path_distance_m = MAX(max_arc_distance_m,
                    MAX((tangent_ne - destination_ne).length(),
                        (end_ne - destination_ne).length()));
                if (max_path_distance_m > (drift_limit_m + distance_tolerance_m)) {
                    continue;
                }

                ExactCapturePath candidate;
                candidate.center_from_destination = center_ne - destination_ne;
                candidate.start_radial_unit = start_radial_unit;
                candidate.tangent_from_destination = tangent_ne - destination_ne;
                candidate.line_unit = line_unit;
                candidate.radius_m = turn_radius_m;
                candidate.arc_angle_rad = arc_angle_rad;
                candidate.arc_length_m = arc_length_m;
                candidate.total_length_m = total_length_m;
                candidate.turn_direction = turn_direction;
                candidate.valid = true;

                const bool shorter = total_length_m < (radius_best_length_m - distance_tolerance_m);
                const bool tied_and_preferred =
                    fabsf(total_length_m - radius_best_length_m) <= distance_tolerance_m &&
                    turn_direction == tie_direction;
                if (!radius_found || shorter || tied_and_preferred) {
                    radius_best = candidate;
                    radius_best_length_m = total_length_m;
                    radius_found = true;
                }
            }
        }

        if (radius_found) {
            best_path = radius_best;
            best_length_m = radius_best_length_m;
            found = true;
            break;
        }
    }

    if (!found || !is_positive(best_length_m)) {
        enter_exact_fault(ExactPivotFault::RecoveryInfeasible);
        return false;
    }

    const float capture_speed_mps = get_exact_capture_speed_max(best_path.radius_m);
    if (!is_positive(capture_speed_mps) ||
        !is_positive(_pos_control.get_accel_max()) ||
        !is_positive(_pos_control.get_jerk_max())) {
        enter_exact_fault(ExactPivotFault::RecoveryBounds);
        return false;
    }

    clear_exact_capture_path();
    _exact_capture_path = best_path;
    _exact_capture_scurve.calculate_track(Vector3f{},
                                          Vector3f{best_path.total_length_m, 0.0f, 0.0f},
                                          capture_speed_mps,
                                          capture_speed_mps,
                                          capture_speed_mps,
                                          _pos_control.get_accel_max(),
                                          _pos_control.get_accel_max(),
                                          AR_WPNAV_SNAP_MAX,
                                          _pos_control.get_jerk_max());
    if (_exact_capture_scurve.finished()) {
        enter_exact_fault(ExactPivotFault::Internal);
        return false;
    }
    _exact_capture_scurve.set_origin_speed_max(0.0f);
    _exact_capture_scurve.set_destination_speed_max(0.0f);
    _exact_turn_direction = best_path.turn_direction;
    _pivot.deactivate();
    clear_heading_handoff();
    _exact_phase = ExactPivotPhase::CapturePath;
    _track_scalar_dt = 1.0f;
    _reached_destination = false;
    _exact_capture_violation_start_ms = 0U;
    start_exact_pivot_timeout(AP_HAL::millis());
    _exact_diag.spin_source = 'B';
    _exact_diag.capture = ExactPivotCaptureDiag{};
    _exact_diag.capture.valid = true;
    _exact_diag.capture.segment = 1U;
    _exact_diag.capture.direction = best_path.turn_direction;
    _exact_diag.capture.length_m = best_path.total_length_m;
    _exact_diag.capture.radius_m = best_path.radius_m;
    _exact_diag.capture.endpoint_distance_m = realtime_distance_m;
    mark_exact_pivot_diag_event(DiagCapturePathB);
    return true;
}

void AR_WPNav::update_exact_capture_path(const Location &current_loc, float dt)
{
    if ((_exact_phase != ExactPivotPhase::CapturePath) ||
        !_exact_capture_path.valid) {
        enter_exact_fault(ExactPivotFault::Internal);
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (exact_pivot_timed_out(now_ms)) {
        enter_exact_fault(ExactPivotFault::Timeout);
        return;
    }

    Vector2f destination_ne_cm;
    if (!_destination.get_vector_xy_from_origin_NE(destination_ne_cm)) {
        enter_exact_fault(ExactPivotFault::Estimator);
        return;
    }
    const Vector2f destination_ne = destination_ne_cm * 0.01f;
    const Vector2f capture_center_ne = destination_ne +
        _exact_capture_path.center_from_destination;
    const Vector2f capture_tangent_ne = destination_ne +
        _exact_capture_path.tangent_from_destination;

    // Reuse the ordinary path time scaler so the private scalar target cannot
    // finish while the physical rover is still lagging behind it.
    const Vector2f current_target_vel = _pos_control.get_desired_velocity();
    float track_scaler_dt = 1.0f;
    if (is_positive(current_target_vel.length())) {
        const Vector2f track_direction = current_target_vel.normalized();
        const float track_error = _pos_control.get_pos_error().tofloat().dot(track_direction);
        Vector3f current_vel_ned;
        if (AP::ahrs().get_velocity_NED(current_vel_ned)) {
            const float track_velocity = current_vel_ned.xy().dot(track_direction);
            track_scaler_dt = constrain_float(
                0.05f + (track_velocity - _pos_control.get_pos_p().kP() * track_error) /
                current_target_vel.length(),
                0.0f,
                1.0f);
        }
    }
    float track_scaler_tc = 1.0f;
    if (is_positive(_pos_control.get_jerk_max())) {
        track_scaler_tc = _pos_control.get_accel_max() / _pos_control.get_jerk_max();
    }
    _track_scalar_dt += (track_scaler_dt - _track_scalar_dt) *
                        (dt / MAX(track_scaler_tc, dt));

    Vector3f scalar_pos;
    Vector3f scalar_vel;
    Vector3f scalar_accel;
    const bool path_finished = _exact_capture_scurve.advance_target_along_track(
        _exact_capture_scurve_aux,
        _exact_capture_scurve_aux,
        0.0f,
        _pos_control.get_lat_accel_max(),
        false,
        _track_scalar_dt * dt,
        scalar_pos,
        scalar_vel,
        scalar_accel);

    const float path_distance_m = constrain_float(scalar_pos.x,
                                                  0.0f,
                                                  _exact_capture_path.total_length_m);
    const float path_speed_mps = MAX(scalar_vel.x, 0.0f);
    const float path_accel_mpss = scalar_accel.x;
    Vector2f target_ne;
    Vector2f target_vel_ne;
    Vector2f target_accel_ne;
    Vector2f target_heading_unit;
    bool target_on_arc = path_distance_m < _exact_capture_path.arc_length_m;

    if (target_on_arc) {
        Vector2f radial_unit = _exact_capture_path.start_radial_unit;
        radial_unit.rotate(_exact_capture_path.turn_direction *
                           (path_distance_m / _exact_capture_path.radius_m));
        target_heading_unit = radial_unit;
        target_heading_unit.rotate(_exact_capture_path.turn_direction * M_PI_2);
        target_ne = capture_center_ne +
                    radial_unit * _exact_capture_path.radius_m;
        target_vel_ne = target_heading_unit * path_speed_mps;
        // AR_AttitudeControl applies its own minimum-speed conversion from
        // lateral acceleration to turn rate. Invert that public conversion so
        // the low-speed capture arc still requests the geometric yaw rate v/R
        // without duplicating the controller's private speed floor here.
        const float unit_turn_rate_rads =
            _atc.get_turn_rate_from_lat_accel(1.0f, path_speed_mps);
        if (!isfinite(unit_turn_rate_rads) || !is_positive(unit_turn_rate_rads)) {
            enter_exact_fault(ExactPivotFault::Internal);
            return;
        }
        const float curvature_accel_mpss =
            (path_speed_mps / _exact_capture_path.radius_m) / unit_turn_rate_rads;
        if (!isfinite(curvature_accel_mpss)) {
            enter_exact_fault(ExactPivotFault::Internal);
            return;
        }
        target_accel_ne = target_heading_unit * path_accel_mpss -
                          radial_unit * curvature_accel_mpss;
    } else {
        target_heading_unit = _exact_capture_path.line_unit;
        target_ne = capture_tangent_ne +
                    target_heading_unit * (path_distance_m -
                                            _exact_capture_path.arc_length_m);
        target_vel_ne = target_heading_unit * path_speed_mps;
        target_accel_ne = target_heading_unit * path_accel_mpss;
    }

    init_pos_control_if_necessary();
    _pos_control.set_reversed(false);
    _pos_control.set_pos_vel_accel_target(Vector2p{target_ne.x, target_ne.y},
                                          target_vel_ne,
                                          target_accel_ne);
    update_path_outputs(dt);
    _desired_heading_cd = wrap_360_cd(degrees(target_heading_unit.angle()) * 100.0f);

    const float realtime_distance_m = current_loc.get_distance(_destination);
    float diag_tracking_error_m = NAN;
    Vector2f diag_current_ne_cm;
    if (current_loc.get_vector_xy_from_origin_NE(diag_current_ne_cm)) {
        const Vector2f diag_current_ne = diag_current_ne_cm * 0.01f;
        if (target_on_arc) {
            diag_tracking_error_m = fabsf((diag_current_ne - capture_center_ne).length() -
                                          _exact_capture_path.radius_m);
        } else {
            diag_tracking_error_m = fabsf((diag_current_ne - capture_tangent_ne) %
                                          _exact_capture_path.line_unit);
        }
    }
    _exact_diag.capture.valid = true;
    _exact_diag.capture.segment = target_on_arc ? 1U : 2U;
    _exact_diag.capture.direction = _exact_capture_path.turn_direction;
    _exact_diag.capture.progress_m = path_distance_m;
    _exact_diag.capture.length_m = _exact_capture_path.total_length_m;
    _exact_diag.capture.radius_m = _exact_capture_path.radius_m;
    _exact_diag.capture.target_speed_mps = path_speed_mps;
    _exact_diag.capture.tracking_error_m = diag_tracking_error_m;
    _exact_diag.capture.heading_error_deg =
        wrap_180(degrees(target_heading_unit.angle()) -
                 AP::ahrs().yaw_sensor * 0.01f);
    _exact_diag.capture.endpoint_distance_m = realtime_distance_m;
    const float capture_radius_m = constrain_float(_pivot_radius, 0.05f, 2.0f);
    if (isfinite(realtime_distance_m) &&
        (realtime_distance_m <= capture_radius_m)) {
        if (!_pivot.activate_planned(_exact_turn_direction)) {
            enter_exact_fault(ExactPivotFault::Internal);
            return;
        }
        enter_exact_spin();
        update_steering_and_speed(current_loc, dt);
        return;
    }

    Vector2f current_ne_cm;
    if (!current_loc.get_vector_xy_from_origin_NE(current_ne_cm)) {
        enter_exact_fault(ExactPivotFault::Estimator);
        return;
    }
    const Vector2f current_ne = current_ne_cm * 0.01f;

    if (path_finished) {
        enter_exact_fault(ExactPivotFault::RecoveryBounds);
        return;
    }

    float tracking_error_m;
    if (target_on_arc) {
        tracking_error_m = fabsf((current_ne - capture_center_ne).length() -
                                 _exact_capture_path.radius_m);
    } else {
        tracking_error_m = fabsf((current_ne - capture_tangent_ne) %
                                 _exact_capture_path.line_unit);
    }
    _cross_track_error = tracking_error_m;
    const float drift_limit_m = constrain_float(_pivot_drift, 0.0f, 3.0f);
    const float heading_error_deg = fabsf(wrap_180(
        degrees(target_heading_unit.angle()) - AP::ahrs().yaw_sensor * 0.01f));
    if ((tracking_error_m > AR_WPNAV_CAPTURE_XTRACK_MAX) ||
        ((current_ne - destination_ne).length() > drift_limit_m) ||
        (!target_on_arc && (heading_error_deg > AR_WPNAV_CAPTURE_HEADING_MAX_DEG)) ||
        (_desired_speed_limited < -0.01f)) {
        enter_exact_fault(ExactPivotFault::RecoveryBounds);
        return;
    }

    const bool opposite_turn_requested = target_on_arc &&
        (_desired_turn_rate_rads * _exact_capture_path.turn_direction < -radians(1.0f));
    if (opposite_turn_requested) {
        _desired_turn_rate_rads = 0.0f;
        if (_exact_capture_violation_start_ms == 0U) {
            _exact_capture_violation_start_ms = MAX(now_ms, 1U);
        } else if ((now_ms - _exact_capture_violation_start_ms) >=
                   AR_WPNAV_CAPTURE_VIOLATION_MS) {
            enter_exact_fault(ExactPivotFault::RecoveryBounds);
        }
    } else {
        _exact_capture_violation_start_ms = 0U;
    }
}

bool AR_WPNav::promote_exact_preview(const Location &current_loc)
{
    if ((_exact_phase != ExactPivotPhase::Spin) ||
        !_exact_next_destination.initialised() ||
        _scurve_next_leg.finished()) {
        return false;
    }

    const Location completed_destination = _destination;
    const Location promoted_destination = _exact_next_destination;

    Vector2f origin_ne_cm;
    Vector2f destination_ne_cm;
    if (!completed_destination.get_vector_xy_from_origin_NE(origin_ne_cm) ||
        !promoted_destination.get_vector_xy_from_origin_NE(destination_ne_cm)) {
        return false;
    }
    origin_ne_cm *= 0.01f;
    destination_ne_cm *= 0.01f;
    const Vector2f frozen_track = destination_ne_cm - origin_ne_cm;
    const float frozen_length_m = frozen_track.length();
    if (!is_positive(frozen_length_m)) {
        return false;
    }

    _pivot.deactivate();
    _origin = completed_destination;
    _destination = promoted_destination;
    _scurve_prev_leg.init();
    _scurve_this_leg = _scurve_next_leg;
    // The promoted Path starts with a small planned forward velocity instead
    // of a second zero-speed phase. Translation belongs to the Path immediately;
    // yaw is handed to path steering continuously while that translation runs.
    _scurve_this_leg.set_origin_speed_max(MAX(_atc.get_stop_speed(), AR_WPNAV_SPEED_MIN));
    _scurve_next_leg.init();
    _next_destination = Location();
    _track_scalar_dt = 1.0f;
    _fast_waypoint = false;
    _pivot_at_next_wp = false;
    _reached_destination = false;
    _orig_and_dest_valid = true;
    _nav_control_type = NavControllerType::NAV_SCURVE;
    _exact_phase = ExactPivotPhase::PromotedMove;
    _completed_destination = completed_destination;
    _active_destination = promoted_destination;
    _completion_event_pending = true;
    ++_handoff_generation;
    if (_handoff_generation == 0U) {
        ++_handoff_generation;
    }
    _exact_timeout_active = false;
    _exact_timeout_start_ms = 0U;
    _exact_position_loss_active = false;
    _exact_position_loss_start_ms = 0U;
    _exact_drift_active = false;
    _exact_drift_start_ms = 0U;

    const Vector2f current_from_origin = completed_destination.get_distance_NE(current_loc);
    const Vector2f frozen_unit = frozen_track / frozen_length_m;
    const float vehicle_along_m = constrain_float(current_from_origin.dot(frozen_unit),
                                                   0.0f,
                                                   frozen_length_m);

    _rejoin_active = false;
    _rejoin_floor_m = 0.0f;
    _rejoin_unit.zero();
    if (is_positive(_pivot_rejoin)) {
        _rejoin_unit = frozen_unit;
        // Absorb only displacement that has already happened during SPIN.  The
        // old +lead behavior jumped the target forward even when the rover was
        // exactly at D0, creating the repeated hook visible at every endpoint.
        _rejoin_floor_m = MIN(vehicle_along_m,
                              constrain_float(_pivot_rejoin, 0.0f, 1.0f));
        _rejoin_active = is_positive(_rejoin_floor_m);
    }

    clear_heading_handoff();
    const float remaining_length_m = frozen_length_m - vehicle_along_m;
    const float configured_blend_m = constrain_float(_pivot_blend, 0.0f, 2.0f);
    const float blend_distance_m = MIN(configured_blend_m,
                                       remaining_length_m * 0.25f);
    if (is_positive(blend_distance_m)) {
        _heading_handoff.origin = completed_destination;
        _heading_handoff.track_unit = frozen_unit;
        _heading_handoff.track_length_m = frozen_length_m;
        _heading_handoff.start_along_m = vehicle_along_m;
        _heading_handoff.blend_distance_m = blend_distance_m;
        _heading_handoff.heading_cd = _exact_pivot_heading_cd;
        _heading_handoff.generation = _handoff_generation;
        _heading_handoff.active = true;
    }

    _exact_diag.handoff = ExactPivotHandoffDiag{};
    _exact_diag.handoff.valid = true;
    _exact_diag.handoff.along_m = vehicle_along_m;
    _exact_diag.handoff.rejoin_m = _rejoin_floor_m;
    _exact_diag.handoff.blend_m = blend_distance_m;
    _exact_diag.handoff.heading_rate_rads = _desired_turn_rate_rads;
    _exact_diag.handoff.output_rate_rads = _desired_turn_rate_rads;

    // Drop velocity-controller memory from the stopped incoming leg while
    // preserving the attitude/rate controller used continuously through Spin.
    _pos_control.get_vel_pid().reset_I();
    _pos_control.get_vel_pid().reset_filter();
    update_distance_and_bearing_to_destination();
    mark_exact_pivot_diag_event(DiagPromotePath);
    return true;
}

void AR_WPNav::apply_forward_rejoin(const Vector2f &origin_ne_m,
                                    Vector3f &target_pos)
{
    if (!_rejoin_active) {
        return;
    }

    const Vector2f target_from_origin{target_pos.x - origin_ne_m.x,
                                      target_pos.y - origin_ne_m.y};
    const float scurve_along_m = target_from_origin.dot(_rejoin_unit);
    if (_exact_diag.handoff.valid) {
        _exact_diag.handoff.along_m = scurve_along_m;
        _exact_diag.handoff.rejoin_m = _rejoin_floor_m;
    }
    if (scurve_along_m >= _rejoin_floor_m) {
        _rejoin_active = false;
        _rejoin_floor_m = 0.0f;
        return;
    }

    const Vector2f adjusted_target = origin_ne_m + (_rejoin_unit * _rejoin_floor_m);
    target_pos.x = adjusted_target.x;
    target_pos.y = adjusted_target.y;
}

void AR_WPNav::update_path_outputs(float dt)
{
    _pos_control.set_reversed(_reversed);
    _pos_control.update(dt);
    _desired_speed_limited = _pos_control.get_desired_speed();
    _desired_turn_rate_rads = _pos_control.get_desired_turn_rate_rads();
    _desired_lat_accel = _pos_control.get_desired_lat_accel();
    apply_heading_handoff();
}

void AR_WPNav::clear_heading_handoff()
{
    _heading_handoff = HeadingHandoff{};
}

void AR_WPNav::clear_promoted_path_constraints()
{
    _rejoin_active = false;
    _rejoin_floor_m = 0.0f;
    _rejoin_unit.zero();
    clear_heading_handoff();
}

void AR_WPNav::apply_heading_handoff()
{
    if (!_heading_handoff.active) {
        return;
    }

    Location current_loc;
    if (!AP::ahrs().get_location(current_loc) ||
        !isfinite(AP::ahrs().get_yaw())) {
        return;
    }

    const float along_m = constrain_float(
        _heading_handoff.origin.get_distance_NE(current_loc).dot(_heading_handoff.track_unit),
        _heading_handoff.start_along_m,
        _heading_handoff.track_length_m);
    const float progress_m = along_m - _heading_handoff.start_along_m;
    const float distance_ratio = constrain_float(progress_m /
                                                  _heading_handoff.blend_distance_m,
                                                  0.0f,
                                                  1.0f);
    const float heading_error_deg = wrap_180(_heading_handoff.heading_cd * 0.01f -
                                             AP::ahrs().yaw_sensor * 0.01f);
    const float exit_error_deg = MAX(constrain_float(_pivot_exit, 0.0f, 45.0f),
                                     5.0f);
    constexpr float release_error_deg = 1.0f;
    const float heading_ratio = constrain_float((exit_error_deg - fabsf(heading_error_deg)) /
                                                (exit_error_deg - release_error_deg),
                                                0.0f,
                                                1.0f);
    const float blend_input = MIN(distance_ratio, heading_ratio);
    const float path_weight = sq(blend_input) * (3.0f - 2.0f * blend_input);
    const float heading_turn_rate_rads = _atc.get_turn_rate_from_heading(
        radians(_heading_handoff.heading_cd * 0.01f),
        radians(_pivot.get_rate_max()));
    const float raw_path_turn_rate_rads = _desired_turn_rate_rads;
    float path_turn_rate_rads = raw_path_turn_rate_rads;
    if ((fabsf(heading_error_deg) > 5.0f) &&
        (heading_turn_rate_rads * path_turn_rate_rads < 0.0f)) {
        path_turn_rate_rads = 0.0f;
    }

    _desired_heading_cd = _heading_handoff.heading_cd;
    _desired_turn_rate_rads = (1.0f - path_weight) * heading_turn_rate_rads +
                              path_weight * path_turn_rate_rads;
    _exact_diag.handoff.valid = true;
    _exact_diag.handoff.along_m = along_m;
    _exact_diag.handoff.rejoin_m = _rejoin_floor_m;
    _exact_diag.handoff.blend_m = _heading_handoff.blend_distance_m;
    _exact_diag.handoff.distance_ratio = distance_ratio;
    _exact_diag.handoff.heading_ratio = heading_ratio;
    _exact_diag.handoff.path_weight = path_weight;
    _exact_diag.handoff.heading_error_deg = heading_error_deg;
    _exact_diag.handoff.heading_rate_rads = heading_turn_rate_rads;
    _exact_diag.handoff.path_rate_rads = raw_path_turn_rate_rads;
    _exact_diag.handoff.output_rate_rads = _desired_turn_rate_rads;

    if ((distance_ratio >= 1.0f) &&
        (fabsf(heading_error_deg) <= release_error_deg)) {
        mark_exact_pivot_diag_event(DiagHandoffDone);
        clear_heading_handoff();
    }
}

bool AR_WPNav::calculate_stopping_scurve(const Location &origin, const Location &destination, SCurve &scurve)
{
    Vector2f origin_NE;
    Vector2f destination_NE;
    if (!origin.get_vector_xy_from_origin_NE(origin_NE) ||
        !destination.get_vector_xy_from_origin_NE(destination_NE)) {
        return false;
    }
    origin_NE *= 0.01f;
    destination_NE *= 0.01f;
    if ((destination_NE - origin_NE).is_zero()) {
        return false;
    }
    scurve.calculate_track(Vector3f{origin_NE.x, origin_NE.y, 0.0f},
                           Vector3f{destination_NE.x, destination_NE.y, 0.0f},
                           _pos_control.get_speed_max(),
                           _pos_control.get_speed_max(),
                           _pos_control.get_speed_max(),
                           _pos_control.get_accel_max(),
                           _pos_control.get_accel_max(),
                           AR_WPNAV_SNAP_MAX,
                           _pos_control.get_jerk_max());
    scurve.set_origin_speed_max(0.0f);
    scurve.set_destination_speed_max(0.0f);
    return true;
}

// calculate the crosstrack error
float AR_WPNav::calc_crosstrack_error(const Location& current_loc) const
{
    if (!_orig_and_dest_valid) {
        return 0.0f;
    }

    // get object avoidance adjusted origin and destination
    const Location &orig = get_oa_origin();
    const Location &dest = get_oa_destination();

    // calculate the NE position of destination relative to origin
    Vector2f dest_from_origin = orig.get_distance_NE(dest);

    // return distance to destination if length of track is very small
    if (dest_from_origin.length() < 1.0e-6f) {
        return current_loc.get_distance_NE(dest).length();
    }

    // convert to a vector indicating direction only
    dest_from_origin.normalize();

    // calculate the NE position of the vehicle relative to origin
    const Vector2f veh_from_origin = orig.get_distance_NE(current_loc);

    // calculate distance to target track, for reporting
    return veh_from_origin % dest_from_origin;
}

// calculate yaw change at next waypoint in degrees
// returns zero if the angle cannot be calculated because some points are on top of others
float AR_WPNav::get_corner_angle(const Location& loc1, const Location& loc2, const Location& loc3) const
{
    // sanity check
    if (!loc1.initialised() || !loc2.initialised() || !loc3.initialised()) {
        return 0;
    }
    const float loc1_to_loc2_deg = loc1.get_bearing_to(loc2) * 0.01;
    const float loc2_to_loc3_deg = loc2.get_bearing_to(loc3) * 0.01;
    const float diff_yaw_deg = wrap_180(loc2_to_loc3_deg - loc1_to_loc2_deg);
    return diff_yaw_deg;
}

// helper function to initialise position controller if it hasn't been called recently
// this should be called before updating the position controller with new targets but after the EKF has a good position estimate
void AR_WPNav::init_pos_control_if_necessary()
{
    // initialise position controller if not called recently
    if (!_pos_control.is_active()) {
        if (!_pos_control.init()) {
            // this should never fail because we should always have a valid position estimate at this point
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return;
        }
    }
}

// set origin and destination to stopping point
bool AR_WPNav::set_origin_and_destination_to_stopping_point()
{
    // initialise origin and destination to stopping point
    Location stopping_loc;
    if (!get_stopping_location(stopping_loc)) {
        return false;
    }
    _origin = _destination = stopping_loc;
    _orig_and_dest_valid = true;
    return true;
}

// check for changes in _base_speed_max or _nudge_speed_max
// updates position controller limits and recalculate scurve path if required
void AR_WPNav::update_speed_max()
{
    const float speed_max = MAX(_base_speed_max, _nudge_speed_max);

    // ignore calls that do not change the speed
    if (is_equal(speed_max, _pos_control.get_speed_max())) {
        return;
    }

    // protect against rapid updates
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_speed_update_ms < AR_WPNAV_SPEED_UPDATE_MIN_MS) {
        return;
    }
    _last_speed_update_ms = now_ms;

    // update position controller max speed
    _pos_control.set_limits(speed_max, _pos_control.get_accel_max(), _pos_control.get_lat_accel_max(), _pos_control.get_jerk_max());

    // change track speed
    _scurve_this_leg.set_speed_max(_pos_control.get_speed_max(), _pos_control.get_speed_max(), _pos_control.get_speed_max());
    _scurve_next_leg.set_speed_max(_pos_control.get_speed_max(), _pos_control.get_speed_max(), _pos_control.get_speed_max());
    if (_exact_capture_path.valid) {
        const float capture_speed_mps =
            get_exact_capture_speed_max(_exact_capture_path.radius_m);
        if (is_positive(capture_speed_mps)) {
            _exact_capture_scurve.set_speed_max(capture_speed_mps,
                                                capture_speed_mps,
                                                capture_speed_mps);
        }
    }
}
