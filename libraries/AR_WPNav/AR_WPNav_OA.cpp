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
#include "AR_WPNav_OA.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>

extern const AP_HAL::HAL& hal;

// update navigation
void AR_WPNav_OA::update(float dt)
{
#if AP_OAPATHPLANNER_ENABLED
    constexpr uint16_t diag_constraint_mask = DiagStateRejoinActive |
                                              DiagStateHandoffActive;
    // Fixed-geometry owners such as Patrol may explicitly bypass obstacle
    // replanning.  AUTO exact-pivot transactions remain subject to OA and are
    // cancelled by the normal setters below if an alternate path is required.
    if (is_exact_pivot_sequence_active() && _exact_pivot_oa_bypass) {
        AR_WPNav::update(dt);
        return;
    }

    // exit immediately if no current location, origin or destination
    Location current_loc;
    float speed;
    if (!hal.util->get_soft_armed() || !is_destination_valid() || !AP::ahrs().get_location(current_loc) || !_atc.get_forward_speed(speed)) {
        // The Exact state machine has phase-specific estimator requirements and
        // owns its transient Hold, 500ms input-loss fault and pivot timeout.
        // In particular, an in-place Spin deliberately does not require a
        // forward-speed estimate. Let the parent process those states instead
        // of indefinitely stopping here ahead of its health gates.
        if (is_exact_pivot_sequence_active()) {
            AR_WPNav::update(dt);
            return;
        }
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _oa_active = false;
        return;
    }

    // run path planning around obstacles
    bool stop_vehicle = false;

    // backup _origin, _destination and _next_destination when not doing oa
    if (!_oa_active) {
        _origin_oabak = _origin;
        _destination_oabak = _destination;
        _next_destination_oabak = _next_destination;
    }

    AP_OAPathPlanner *oa = AP_OAPathPlanner::get_singleton();
    if (oa != nullptr) {
        Location oa_origin_new, oa_destination_new, oa_next_destination_new;
        AP_OAPathPlanner::OAPathPlannerUsed path_planner_used;
        bool dest_to_next_dest_clear;
        const AP_OAPathPlanner::OA_RetState oa_retstate = oa->mission_avoidance(current_loc,
                                                                                _origin_oabak,
                                                                                _destination_oabak,
                                                                                _next_destination_oabak,
                                                                                oa_origin_new,
                                                                                oa_destination_new,
                                                                                oa_next_destination_new,
                                                                                dest_to_next_dest_clear,
                                                                                path_planner_used);

        switch (oa_retstate) {

        case AP_OAPathPlanner::OA_NOT_REQUIRED:
            if (_oa_active) {
                // object avoidance has become inactive so reset target to original destination
                clear_promoted_path_constraints();
                if (!AR_WPNav::set_desired_location(_destination_oabak)) {
                    // this should never happen because we should have an EKF origin and the destination must be valid
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    stop_vehicle = true;
                }
                _oa_active = false;
                // ToDo: handle "if (oa->get_options() & AP_OAPathPlanner::OA_OPTION_WP_RESET)"
            }
            break;

        case AP_OAPathPlanner::OA_PROCESSING:
        case AP_OAPathPlanner::OA_ERROR:
            // during processing or in case of error, slow vehicle to a stop
            // This also clears a promoted-path handoff that survived an
            // ordinary mission ACK after the Exact phase returned to None.
            {
                const bool replacing_exact = is_exact_pivot_sequence_active();
                const ExactPivotDiagSnapshot diag_before = get_exact_pivot_diag_snapshot();
                const bool cancelling_constraints =
                    (diag_before.state_flags & diag_constraint_mask) != 0U;
                clear_promoted_path_constraints();
                if (replacing_exact) {
                    // OA owns this wait. Do not leave an Exact SPIN or completion
                    // transaction frozen behind the early return below because its
                    // timeout and health state would never advance. Restore the
                    // ordinary mission waypoint contract; OA can replan it on a
                    // later successful cycle.
                    if (!AR_WPNav::set_desired_location(_destination_oabak)) {
                        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    } else {
                        note_exact_pivot_oa_cancel(
                            diag_before.phase,
                            oa_retstate == AP_OAPathPlanner::OA_PROCESSING ?
                                ExactPivotOACancelReason::Processing :
                                ExactPivotOACancelReason::Error);
                    }
                } else if (cancelling_constraints) {
                    note_exact_pivot_oa_cancel(
                        diag_before.phase,
                        oa_retstate == AP_OAPathPlanner::OA_PROCESSING ?
                            ExactPivotOACancelReason::Processing :
                            ExactPivotOACancelReason::Error);
                }
                stop_vehicle = true;
                _oa_active = false;
            }
            break;

        case AP_OAPathPlanner::OA_SUCCESS:
            // handling of returned destination depends upon path planner used
            switch (path_planner_used) {

            case AP_OAPathPlanner::OAPathPlannerUsed::None:
            case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerVertical:
                // this should never happen.  this means the path planner has returned success but has returned an invalid planner
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                {
                    const bool replacing_exact = is_exact_pivot_sequence_active();
                    const ExactPivotDiagSnapshot diag_before = get_exact_pivot_diag_snapshot();
                    const bool cancelling_constraints =
                        (diag_before.state_flags & diag_constraint_mask) != 0U;
                    clear_promoted_path_constraints();
                    if (replacing_exact) {
                        if (!AR_WPNav::set_desired_location(_destination_oabak)) {
                            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                        } else {
                            note_exact_pivot_oa_cancel(
                                diag_before.phase,
                                ExactPivotOACancelReason::InvalidPlanner);
                        }
                    } else if (cancelling_constraints) {
                        note_exact_pivot_oa_cancel(
                            diag_before.phase,
                            ExactPivotOACancelReason::InvalidPlanner);
                    }
                    _oa_active = false;
                    stop_vehicle = true;
                }
                break;

            case AP_OAPathPlanner::OAPathPlannerUsed::Dijkstras:
                // Dijkstra's.  Action is only needed if path planner has just became active or the target destination's lat or lon has changed
                if (!_oa_active || !oa_destination_new.same_latlon_as(_oa_destination)) {
                    const bool replacing_exact = is_exact_pivot_sequence_active();
                    const ExactPivotDiagSnapshot diag_before = get_exact_pivot_diag_snapshot();
                    const bool cancelling_constraints =
                        (diag_before.state_flags & diag_constraint_mask) != 0U;
                    clear_promoted_path_constraints();
                    if (AR_WPNav::set_desired_location(oa_destination_new)) {
                        // if new target set successfully, update oa state and destination
                        _oa_active = true;
                        _oa_origin = oa_origin_new;
                        _oa_destination = oa_destination_new;
                        if (replacing_exact || cancelling_constraints) {
                            note_exact_pivot_oa_cancel(
                                diag_before.phase,
                                ExactPivotOACancelReason::Dijkstra);
                        }
                    } else {
                        // this should never happen
                        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                        stop_vehicle = true;
                    }
                }
                break;

            case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerHorizontal: {
                // BendyRuler.  Action is only needed if path planner has just became active or the target destination's lat or lon has changed
                if (!_oa_active || !oa_destination_new.same_latlon_as(_oa_destination)) {
                    const bool replacing_exact = is_exact_pivot_sequence_active();
                    const ExactPivotDiagSnapshot diag_before = get_exact_pivot_diag_snapshot();
                    const bool cancelling_constraints =
                        (diag_before.state_flags & diag_constraint_mask) != 0U;
                    clear_promoted_path_constraints();
                    if (AR_WPNav::set_desired_location_expect_fast_update(oa_destination_new)) {
                        // if new target set successfully, update oa state and destination
                        _oa_active = true;
                        _oa_origin = oa_origin_new;
                        _oa_destination = oa_destination_new;
                        if (replacing_exact || cancelling_constraints) {
                            note_exact_pivot_oa_cancel(
                                diag_before.phase,
                                ExactPivotOACancelReason::BendyRuler);
                        }
                    } else {
                        // this should never happen
                        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                        stop_vehicle = true;
                    }
                }
            }
            break;

            } // switch (path_planner_used) {
        } // switch (oa_retstate) {
    } // if (oa != nullptr) {

    update_oa_distance_and_bearing_to_destination();

    // handle stopping vehicle if avoidance has failed
    if (stop_vehicle) {
        // decelerate to speed to zero and set turn rate to zero
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        return;
    }
#endif  // AP_OAPATHPLANNER_ENABLED

    // call parent update
    AR_WPNav::update(dt);
}

// set desired location and (optionally) next_destination
// next_destination should be provided if known to allow smooth cornering
bool AR_WPNav_OA::set_desired_location(const Location& destination, Location next_destination)
{
    _exact_pivot_oa_bypass = false;
    const bool ret = AR_WPNav::set_desired_location(destination, next_destination);

    if (ret) {
        // disable object avoidance, it will be re-enabled (if necessary) on next update
        _oa_active = false;
    }

    return ret;
}

bool AR_WPNav_OA::set_desired_location_exact_pivot(const Location &destination, const Location &next_destination)
{
    const bool ret = AR_WPNav::set_desired_location_exact_pivot(destination, next_destination);
    if (ret) {
        _oa_active = false;
    }
    return ret;
}

// true if vehicle has reached desired location. defaults to true because this is normally used by missions and we do not want the mission to become stuck
bool AR_WPNav_OA::reached_destination() const
{
    // object avoidance should always be deactivated before reaching final destination
    if (_oa_active) {
        return false;
    }

    return AR_WPNav::reached_destination();
}

// get object avoidance adjusted origin. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
const Location &AR_WPNav_OA::get_oa_origin() const
{
    if (_oa_active) {
        return _oa_origin;
    }

    return _origin;
}

// get object avoidance adjusted destination. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
const Location &AR_WPNav_OA::get_oa_destination() const
{
    if (_oa_active) {
        return _oa_destination;
    }

    return AR_WPNav::get_oa_destination();
}

// update distance from vehicle's current position to destination
void AR_WPNav_OA::update_oa_distance_and_bearing_to_destination()
{
    // update OA adjusted values
    Location current_loc;
    if (_oa_active && AP::ahrs().get_location(current_loc)) {
        _oa_distance_to_destination = current_loc.get_distance(_oa_destination);
        _oa_wp_bearing_cd = current_loc.get_bearing_to(_oa_destination);
    } else {
        _oa_distance_to_destination = AR_WPNav::get_distance_to_destination();
        _oa_wp_bearing_cd = AR_WPNav::wp_bearing_cd();
    }
}
