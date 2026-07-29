/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Math/AP_Math.h>

#include "AR_WPNav.h"

extern const AP_HAL::HAL& hal;

#define AR_WPNAV_TIMEOUT_MS             100
#define AR_WPNAV_SPEED_DEFAULT          2.0f
#define AR_WPNAV_SPEED_MIN              0.05f
#define AR_WPNAV_SPEED_UPDATE_MIN_MS    500
#define AR_WPNAV_RADIUS_DEFAULT         2.0f
#define AR_WPNAV_OVERSPEED_RATIO_MAX    5.0f
#define AR_WPNAV_SNAP_MAX               15.0f
#define AR_WPNAV_ACCEL_MAX              20.0f
#define AR_WPNAV_PIVOT_RADIUS_DEFAULT   0.2f
#define AR_WPNAV_PIVOT_DRIFT_DEFAULT    0.5f
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
    // @Description: Waypoint acceleration. If zero then ATC_ACCEL_MAX is used
    // @Units: m/s/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ACCEL", 9, AR_WPNav, _accel_max, 0),

    // @Param: JERK
    // @DisplayName: Waypoint jerk
    // @Description: Waypoint jerk (change in acceleration). If zero then jerk is same as acceleration
    // @Units: m/s/s/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("JERK", 10, AR_WPNav, _jerk_max, 0),

    // @Param: PIVOT_RADIUS
    // @DisplayName: Patrol endpoint radius
    // @Description: Maximum realtime distance from a Patrol theoretical endpoint before Drive may change directly to Spin
    // @Units: m
    // @Range: 0.05 2.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("PIVOT_RADIUS", 11, AR_WPNav, _pivot_radius, AR_WPNAV_PIVOT_RADIUS_DEFAULT),

    // Index 12 was PIVOT_EXIT. It is permanently reserved for storage compatibility.

    // @Param: PIVOT_DRIFT
    // @DisplayName: Patrol spin drift allowance
    // @Description: Additional realtime position drift allowed around a Patrol endpoint during an in-place Spin
    // @Units: m
    // @Range: 0 3
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("PIVOT_DRIFT", 13, AR_WPNav, _pivot_drift, AR_WPNAV_PIVOT_DRIFT_DEFAULT),

    // Indices 14 through 17 were PIVOT_REJOIN, PIVOT_TOUT,
    // PIVOT_CAPSPD and PIVOT_BLEND. They remain permanently reserved.

    AP_GROUPEND
};

AR_WPNav::AR_WPNav(AR_AttitudeControl& atc, AR_PosControl &pos_control) :
    _pivot(atc),
    _atc(atc),
    _pos_control(pos_control)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AR_WPNav::init(float speed_max)
{
    _base_speed_max = is_positive(speed_max) ? speed_max : _speed_max;
    _base_speed_max = MAX(AR_WPNAV_SPEED_MIN, _base_speed_max);

    float atc_accel_max = MIN(_atc.get_accel_max(), _atc.get_decel_max());
    if (!is_positive(atc_accel_max)) {
        atc_accel_max = AR_WPNAV_ACCEL_MAX;
    }
    const float accel_max = is_positive(_accel_max) ?
        MIN(_accel_max, atc_accel_max) : atc_accel_max;
    const float jerk_max = is_positive(_jerk_max) ? _jerk_max : accel_max;

    _pos_control.set_limits(_base_speed_max,
                            accel_max,
                            _atc.get_turn_lat_accel_max(),
                            jerk_max);

    _scurve_prev_leg.init();
    _scurve_this_leg.init();
    _scurve_next_leg.init();
    _track_scalar_dt = 1.0f;
    clear_stopping_line_state();

    _reached_destination = false;
    _fast_waypoint = false;

    _pivot.deactivate();
    _planned_pivot_active = false;
    _pivot_heading_valid = false;
    _pivot_at_next_wp = false;
    _last_yaw_reset_ms = 0U;

    _orig_and_dest_valid = false;
    set_origin_and_destination_to_stopping_point();

    set_nudge_speed_max(0.0f);
}

void AR_WPNav::update(float dt)
{
    Location current_loc;
    float speed;
    if (!hal.util->get_soft_armed() ||
        !_orig_and_dest_valid ||
        !AP::ahrs().get_location(current_loc) ||
        !_atc.get_forward_speed(speed)) {
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _cross_track_error = 0.0f;
        return;
    }

    if (!is_active()) {
        _desired_speed_limited = speed;
    }
    _last_update_ms = AP_HAL::millis();

    update_distance_and_bearing_to_destination();
    update_speed_max();

    // Patrol owns a planned Spin directly and does not advance WPNav while it
    // is active. Keep this guard so an accidental update cannot move the path.
    if (!_pivot.active() && !_planned_pivot_active) {
        switch (_nav_control_type) {
        case NavControllerType::NAV_SCURVE:
            advance_wp_target_along_track(current_loc, dt);
            break;
        case NavControllerType::NAV_PSC_INPUT_SHAPING:
            update_psc_input_shaping(dt);
            break;
        }
    }

    update_steering_and_speed(current_loc, dt);
}

void AR_WPNav::set_reversed(bool reversed)
{
    if (_reversed == reversed) {
        return;
    }
    clear_stopping_line_progress();
    _reversed = reversed;
}

bool AR_WPNav::set_speed_max(float speed_max)
{
    if (speed_max < AR_WPNAV_SPEED_MIN) {
        return false;
    }
    _base_speed_max = speed_max;
    return true;
}

void AR_WPNav::set_nudge_speed_max(float nudge_speed_max)
{
    _nudge_speed_max = nudge_speed_max;
}

bool AR_WPNav::set_desired_location(const Location& destination,
                                    Location next_destination)
{
    const bool continue_from_previous_fast =
        _fast_waypoint && !_scurve_next_leg.finished();
    const bool force_entry_pivot = _pivot_at_next_wp;

    if (!is_active() ||
        !_reached_destination ||
        (_nav_control_type != NavControllerType::NAV_SCURVE)) {
        if (!set_origin_and_destination_to_stopping_point()) {
            return false;
        }
        _scurve_prev_leg.init();
        _scurve_this_leg.init();
        _scurve_next_leg.init();
    }

    _scurve_prev_leg = _scurve_this_leg;
    _origin = _destination;
    _destination = destination;
    _orig_and_dest_valid = true;
    _reached_destination = false;

    update_distance_and_bearing_to_destination();

    Vector2f origin_NE;
    Vector2f destination_NE;
    if (!_origin.get_vector_xy_from_origin_NE(origin_NE) ||
        !_destination.get_vector_xy_from_origin_NE(destination_NE)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    origin_NE *= 0.01f;
    destination_NE *= 0.01f;

    if (continue_from_previous_fast) {
        _scurve_this_leg = _scurve_next_leg;
    } else {
        _track_scalar_dt = 1.0f;
        _scurve_this_leg.calculate_track(
            Vector3p{origin_NE.x, origin_NE.y, 0.0f},
            Vector3p{destination_NE.x, destination_NE.y, 0.0f},
            _pos_control.get_speed_max(),
            _pos_control.get_speed_max(),
            _pos_control.get_speed_max(),
            _pos_control.get_accel_max(),
            _pos_control.get_accel_max(),
            AR_WPNAV_SNAP_MAX,
            _pos_control.get_jerk_max());
    }

    _scurve_next_leg.init();
    _fast_waypoint = false;
    _pivot_at_next_wp = false;
    _next_destination = next_destination;
    if (next_destination.initialised()) {
        const float next_wp_yaw_change =
            get_corner_angle(_origin, destination, next_destination);
        _pivot_at_next_wp = _pivot.would_activate(next_wp_yaw_change);
        if (!_pivot_at_next_wp) {
            Vector2f next_destination_NE;
            if (!next_destination.get_vector_xy_from_origin_NE(next_destination_NE)) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return false;
            }
            next_destination_NE *= 0.01f;
            _scurve_next_leg.calculate_track(
                Vector3p{destination_NE.x, destination_NE.y, 0.0f},
                Vector3p{next_destination_NE.x, next_destination_NE.y, 0.0f},
                _pos_control.get_speed_max(),
                _pos_control.get_speed_max(),
                _pos_control.get_speed_max(),
                _pos_control.get_accel_max(),
                _pos_control.get_accel_max(),
                AR_WPNAV_SNAP_MAX,
                _pos_control.get_jerk_max());
            _fast_waypoint = true;
        }
    }

    _pivot.deactivate();
    _planned_pivot_active = false;
    _pivot_heading_valid = false;
    clear_stopping_line_state();
    _nav_control_type = NavControllerType::NAV_SCURVE;

    if (!continue_from_previous_fast) {
        check_pivot_activation(force_entry_pivot);
    }

    update_distance_and_bearing_to_destination();
    return true;
}

bool AR_WPNav::set_desired_location_stopping_from_origin(
    const Location &origin,
    const Location &destination,
    float speed_max_mps)
{
    if (!origin.initialised() ||
        !destination.initialised() ||
        !isfinite(speed_max_mps) ||
        is_negative(speed_max_mps) ||
        origin.same_latlon_as(destination)) {
        return false;
    }

    SCurve stopping_leg;
    if (!calculate_stopping_scurve(origin,
                                   destination,
                                   stopping_leg,
                                   speed_max_mps)) {
        return false;
    }

    _pivot.deactivate();
    _planned_pivot_active = false;
    clear_stopping_line_progress();

    Location current_loc;
    if (AP::ahrs().get_location(current_loc)) {
        const Vector2f track = origin.get_distance_NE(destination);
        const float track_length_m = track.length();
        if (is_positive(track_length_m)) {
            const Vector2f track_unit = track / track_length_m;
            const float along_m =
                origin.get_distance_NE(current_loc).dot(track_unit);
            if (isfinite(along_m) && is_positive(along_m)) {
                _stopping_line_progress_unit = track_unit;
                _stopping_line_progress_floor_m =
                    constrain_float(along_m, 0.0f, track_length_m);
                _stopping_line_progress_active =
                    is_positive(_stopping_line_progress_floor_m);
            }
        }
    }

    _scurve_prev_leg.init();
    _scurve_this_leg = stopping_leg;
    _scurve_next_leg.init();
    _track_scalar_dt = 1.0f;
    _origin = origin;
    _destination = destination;
    _next_destination = Location();
    _orig_and_dest_valid = true;
    _reached_destination = false;
    _fast_waypoint = false;
    _pivot_at_next_wp = false;
    _pivot_heading_valid = false;
    _nav_control_type = NavControllerType::NAV_SCURVE;
    _stopping_line_active = true;
    _stopping_line_speed_max_mps = speed_max_mps;
    _path_terminal = false;
    _path_target_speed_mps = 0.0f;
    update_distance_and_bearing_to_destination();
    return true;
}

void AR_WPNav::cancel_stopping_line()
{
    cancel_planned_pivot();
    if (!_stopping_line_active) {
        clear_stopping_line_progress();
        return;
    }

    clear_stopping_line_state();
    _scurve_prev_leg.init();
    _scurve_this_leg.init();
    _scurve_next_leg.init();
    _orig_and_dest_valid = false;
    _reached_destination = false;
    set_navigation_outputs_zero();
}

bool AR_WPNav::start_planned_pivot(float target_heading_cd,
                                   int8_t preferred_direction)
{
    if (!isfinite(target_heading_cd) ||
        ((preferred_direction != -1) &&
         (preferred_direction != 0) &&
         (preferred_direction != 1)) ||
        !_pivot.available()) {
        return false;
    }

    _pivot.deactivate();
    _planned_pivot_active = false;
    _planned_pivot_heading_cd = wrap_360_cd(target_heading_cd);
    if (!_pivot.activate_planned(preferred_direction)) {
        return false;
    }

    _planned_pivot_active = true;
    set_navigation_outputs_zero();
    return true;
}

AR_WPNav::PlannedPivotResult AR_WPNav::update_planned_pivot(float &turn_rate_rads)
{
    turn_rate_rads = 0.0f;
    if (!_planned_pivot_active || !_pivot.active()) {
        return PlannedPivotResult::Fault;
    }

    const float yaw_rad = AP::ahrs().get_yaw();
    const float yaw_rate_rads = AP::ahrs().get_yaw_rate_earth();
    if (!isfinite(yaw_rad) || !isfinite(yaw_rate_rads)) {
        return PlannedPivotResult::Fault;
    }

    float yaw_reset_delta;
    const uint32_t yaw_reset_ms =
        AP::ahrs().getLastYawResetAngle(yaw_reset_delta);
    if ((yaw_reset_ms != 0U) && (yaw_reset_ms != _last_yaw_reset_ms)) {
        _last_yaw_reset_ms = yaw_reset_ms;
        _pivot.handle_yaw_reset();
    }

    _desired_heading_cd = _planned_pivot_heading_cd;
    turn_rate_rads =
        _pivot.get_turn_rate_rads(_planned_pivot_heading_cd * 0.01f);
    if (!isfinite(turn_rate_rads)) {
        turn_rate_rads = 0.0f;
        return PlannedPivotResult::Fault;
    }

    _desired_speed_limited = 0.0f;
    _desired_lat_accel = 0.0f;
    if (!_pivot.update_completion(
            _planned_pivot_heading_cd * 0.01f,
            yaw_rate_rads,
            AP_HAL::millis(),
            AR_PivotTurn::CompletionDelayPolicy::NoDelay)) {
        _desired_turn_rate_rads = turn_rate_rads;
        return PlannedPivotResult::Running;
    }

    _planned_pivot_active = false;
    turn_rate_rads = 0.0f;
    _desired_turn_rate_rads = 0.0f;
    return PlannedPivotResult::Complete;
}

void AR_WPNav::cancel_planned_pivot()
{
    if (_planned_pivot_active) {
        _pivot.deactivate();
    }
    _planned_pivot_active = false;
    _desired_turn_rate_rads = 0.0f;
}

float AR_WPNav::get_leg_speed_max(float requested_speed_max_mps) const
{
    const float controller_speed_max_mps = _pos_control.get_speed_max();
    if (!isfinite(controller_speed_max_mps) ||
        !is_positive(controller_speed_max_mps)) {
        return 0.0f;
    }
    if (!is_positive(requested_speed_max_mps)) {
        return controller_speed_max_mps;
    }
    return MIN(controller_speed_max_mps, requested_speed_max_mps);
}

bool AR_WPNav::set_desired_location_to_stopping_location()
{
    Location stopping_loc;
    if (!get_stopping_location(stopping_loc)) {
        return false;
    }
    return set_desired_location(stopping_loc);
}

bool AR_WPNav::set_desired_location_NED(const Vector3f& destination)
{
    Location destination_ned;
    if (!AP::ahrs().get_origin(destination_ned)) {
        return false;
    }
    destination_ned.offset(destination.x, destination.y);
    return set_desired_location(destination_ned);
}

bool AR_WPNav::set_desired_location_NED(const Vector3f &destination,
                                        const Vector3f &next_destination)
{
    Location dest_loc;
    Location next_dest_loc;
    if (!AP::ahrs().get_origin(dest_loc)) {
        return false;
    }
    next_dest_loc = dest_loc;
    dest_loc.offset(destination.x, destination.y);
    next_dest_loc.offset(next_destination.x, next_destination.y);
    return set_desired_location(dest_loc, next_dest_loc);
}

bool AR_WPNav::set_desired_location_expect_fast_update(
    const Location &destination)
{
    if (!is_active() ||
        (_nav_control_type != NavControllerType::NAV_PSC_INPUT_SHAPING)) {
        if (!set_origin_and_destination_to_stopping_point()) {
            return false;
        }
    }

    _origin = _destination;
    _destination = destination;
    _next_destination = Location();
    _orig_and_dest_valid = true;
    _reached_destination = false;
    update_distance_and_bearing_to_destination();

    _pivot.deactivate();
    _planned_pivot_active = false;
    _pivot_heading_valid = false;
    clear_stopping_line_state();

    check_pivot_activation();

    _nav_control_type = NavControllerType::NAV_PSC_INPUT_SHAPING;
    return true;
}

bool AR_WPNav::get_stopping_location(Location& stopping_loc)
{
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return false;
    }

    const Vector2f velocity = AP::ahrs().groundspeed_vector();
    const float speed = velocity.length();
    if (!is_positive(speed)) {
        stopping_loc = current_loc;
        return true;
    }

    const float stopping_dist = _atc.get_stopping_distance(speed);
    const Vector2f stopping_offset = velocity.normalized() * stopping_dist;
    stopping_loc = current_loc;
    stopping_loc.offset(stopping_offset.x, stopping_offset.y);
    return true;
}

bool AR_WPNav::is_active() const
{
    return (AP_HAL::millis() - _last_update_ms) < AR_WPNAV_TIMEOUT_MS;
}

void AR_WPNav::advance_wp_target_along_track(const Location &current_loc,
                                             float dt)
{
    Vector2f curr_pos_NE;
    Vector3f curr_vel_NED;
    if (!AP::ahrs().get_relative_position_NE_origin(curr_pos_NE) ||
        !AP::ahrs().get_velocity_NED(curr_vel_NED)) {
        return;
    }

    Vector2f origin_NE;
    if (!_origin.get_vector_xy_from_origin_NE(origin_NE)) {
        return;
    }
    origin_NE *= 0.01f;

    const Vector2f curr_target_vel = _pos_control.get_desired_velocity();
    float track_scaler_dt = 1.0f;
    if (is_positive(curr_target_vel.length())) {
        const Vector2f track_direction = curr_target_vel.normalized();
        const float track_error =
            _pos_control.get_pos_error().tofloat().dot(track_direction);
        const float track_velocity =
            curr_vel_NED.xy().dot(track_direction);
        const float track_scaler_dt_max = _overspeed_enabled ?
            AR_WPNAV_OVERSPEED_RATIO_MAX : 1.0f;
        track_scaler_dt = constrain_float(
            0.05f +
                (track_velocity -
                 _pos_control.get_pos_p().kP() * track_error) /
                    curr_target_vel.length(),
            0.0f,
            track_scaler_dt_max);
    }

    float track_scaler_tc = 1.0f;
    if (is_positive(_pos_control.get_jerk_max())) {
        track_scaler_tc =
            _pos_control.get_accel_max() / _pos_control.get_jerk_max();
    }
    _track_scalar_dt +=
        (track_scaler_dt - _track_scalar_dt) *
        (dt / MAX(track_scaler_tc, dt));

    Vector3p target_pos_3d{origin_NE.x, origin_NE.y, 0.0f};
    Vector3f target_vel;
    Vector3f target_accel;

    const float wp_radius = _stopping_line_active ?
        0.0f : MAX(_radius, _turn_radius);
    const bool s_finished = _scurve_this_leg.advance_target_along_track(
        _scurve_prev_leg,
        _scurve_next_leg,
        wp_radius,
        _pos_control.get_lat_accel_max(),
        _fast_waypoint,
        _track_scalar_dt * dt,
        target_pos_3d,
        target_vel,
        target_accel);

    if (_stopping_line_active) {
        // A terminal S-curve has zero feed-forward velocity. AR_PosControl
        // normally interprets that as a full stop and disables lateral
        // steering, which can strand a skid-steer rover just outside the
        // physical endpoint radius. Keep the original theoretical line and
        // derive the forward marker from existing controller policy: position
        // P converts remaining along-line distance to speed, ATC_STOP_SPEED
        // supplies the usable low-speed floor, and the current leg limit caps
        // the result. This is still the original Drive leg; it does not create
        // a capture path or replan from realtime position.
        if (s_finished &&
            (current_loc.get_distance(_destination) > get_pivot_radius())) {
            const Vector2f track = _origin.get_distance_NE(_destination);
            const float track_length_m = track.length();
            if (isfinite(track_length_m) && is_positive(track_length_m)) {
                const Vector2f track_unit = track / track_length_m;
                const Vector2f remaining =
                    current_loc.get_distance_NE(_destination);
                const float signed_remaining_along_m =
                    remaining.dot(track_unit);
                const float along_overrun_m = -signed_remaining_along_m;
                if (isfinite(along_overrun_m) &&
                    (along_overrun_m <= get_pivot_radius())) {
                    const float remaining_along_m =
                        MAX(signed_remaining_along_m, 0.0f);
                    const float leg_speed_max_mps =
                        get_leg_speed_max(_stopping_line_speed_max_mps);
                    const float guidance_speed_mps = MIN(
                        leg_speed_max_mps,
                        MAX(_atc.get_stop_speed(),
                            _pos_control.get_pos_p().kP() * remaining_along_m));
                    if (isfinite(guidance_speed_mps) &&
                        is_positive(guidance_speed_mps)) {
                        target_vel.xy() = track_unit * guidance_speed_mps;
                    }
                }
            }
        }
        _path_terminal = s_finished;
        _path_target_speed_mps = target_vel.length();
        apply_stopping_line_progress_floor(origin_NE, target_pos_3d);
    }

    init_pos_control_if_necessary();
    _pos_control.set_pos_vel_accel_target(target_pos_3d.xy(),
                                          target_vel.xy(),
                                          target_accel.xy());

    if (!_reached_destination && s_finished) {
        if (_fast_waypoint) {
            _reached_destination = true;
        } else {
            const bool near_wp =
                current_loc.get_distance(_destination) <= _radius;
            const bool past_wp =
                current_loc.past_interval_finish_line(_origin, _destination);
            _reached_destination = near_wp || past_wp;
        }
    }
}

void AR_WPNav::update_psc_input_shaping(float dt)
{
    Vector2f pos_target_cm;
    if (!_destination.get_vector_xy_from_origin_NE(pos_target_cm)) {
        return;
    }

    init_pos_control_if_necessary();
    const Vector2p pos_target = pos_target_cm.topostype() * 0.01;
    _pos_control.input_pos_target(pos_target, dt);

    if (!_reached_destination) {
        const Vector2p pos_target_diff =
            pos_target - _pos_control.get_pos_target();
        _reached_destination =
            (pos_target_diff.length_squared() < sq(0.01)) &&
            (_pos_control.get_pos_error().length_squared() < sq(_radius));
    }
}

void AR_WPNav::update_distance_and_bearing_to_destination()
{
    Location current_loc;
    if (!_orig_and_dest_valid || !AP::ahrs().get_location(current_loc)) {
        _distance_to_destination = 0.0f;
        _wp_bearing_cd = 0.0f;
        return;
    }
    _distance_to_destination = current_loc.get_distance(_destination);
    _wp_bearing_cd = current_loc.get_bearing_to(_destination);
}

void AR_WPNav::update_steering_and_speed(const Location &current_loc,
                                         float dt)
{
    _cross_track_error = calc_crosstrack_error(current_loc);

    if (_planned_pivot_active) {
        set_navigation_outputs_zero();
        return;
    }

    if (_pivot.active()) {
        _desired_heading_cd = get_pivot_target_heading_cd();
        _desired_speed_limited =
            _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;

        if (!is_zero(_desired_speed_limited)) {
            _desired_turn_rate_rads = 0.0f;
            _pivot.reset_completion();
            return;
        }

        _desired_turn_rate_rads =
            _pivot.get_turn_rate_rads(_desired_heading_cd * 0.01f);
        // Ordinary AUTO/Guided/RTL pivots keep the native heading and
        // configured PIVOT_DELAY completion policy. Patrol alone uses the
        // explicit planned-pivot completion gate with NoDelay and yaw rate.
        _pivot.check_activation(_desired_heading_cd * 0.01f);
        if (!_pivot.active()) {
            _desired_turn_rate_rads = 0.0f;
            _pivot_heading_valid = false;
        }
        return;
    }

    _pos_control.set_reversed(_reversed);
    _pos_control.update(dt);
    _desired_speed_limited = _pos_control.get_desired_speed();
    _desired_turn_rate_rads = _pos_control.get_desired_turn_rate_rads();
    _desired_lat_accel = _pos_control.get_desired_lat_accel();
}

void AR_WPNav::check_pivot_activation(bool force_active)
{
    const float pivot_heading_cd = _reversed ?
        wrap_360_cd(wp_bearing_cd() + 18000.0f) : wp_bearing_cd();
    _pivot.check_activation(pivot_heading_cd * 0.01f, force_active);
    if (_pivot.active()) {
        _pivot_heading_cd = pivot_heading_cd;
        _pivot_heading_valid = true;
    }
}

void AR_WPNav::set_turn_params(float turn_radius, bool pivot_possible)
{
    _turn_radius = pivot_possible ? 0.0f : turn_radius;
    _pivot.enable(pivot_possible);
}

float AR_WPNav::get_pivot_target_heading_cd() const
{
    if (_pivot.active() && _pivot_heading_valid) {
        return _pivot_heading_cd;
    }
    const float forward_heading_cd = wp_bearing_cd();
    return _reversed ?
        wrap_360_cd(forward_heading_cd + 18000.0f) : forward_heading_cd;
}

bool AR_WPNav::calculate_stopping_scurve(const Location &origin,
                                         const Location &destination,
                                         SCurve &scurve,
                                         float speed_max_mps)
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

    const float effective_speed_max_mps = get_leg_speed_max(speed_max_mps);
    if (!is_positive(effective_speed_max_mps)) {
        return false;
    }

    scurve.calculate_track(
        Vector3p{origin_NE.x, origin_NE.y, 0.0f},
        Vector3p{destination_NE.x, destination_NE.y, 0.0f},
        effective_speed_max_mps,
        effective_speed_max_mps,
        effective_speed_max_mps,
        _pos_control.get_accel_max(),
        _pos_control.get_accel_max(),
        AR_WPNAV_SNAP_MAX,
        _pos_control.get_jerk_max());
    scurve.set_origin_speed_max(0.0f);
    scurve.set_destination_speed_max(0.0f);
    return true;
}

void AR_WPNav::apply_stopping_line_progress_floor(
    const Vector2f &origin_ne_m,
    Vector3p &target_pos)
{
    if (!_stopping_line_progress_active) {
        return;
    }

    const Vector2f target_from_origin =
        target_pos.xy().tofloat() - origin_ne_m;
    const float scurve_along_m =
        target_from_origin.dot(_stopping_line_progress_unit);
    if (!isfinite(scurve_along_m) ||
        (scurve_along_m >= _stopping_line_progress_floor_m)) {
        clear_stopping_line_progress();
        return;
    }

    const Vector2f adjusted_target =
        origin_ne_m +
        (_stopping_line_progress_unit * _stopping_line_progress_floor_m);
    target_pos.xy() = adjusted_target.topostype();
}

void AR_WPNav::clear_stopping_line_progress()
{
    _stopping_line_progress_unit.zero();
    _stopping_line_progress_floor_m = 0.0f;
    _stopping_line_progress_active = false;
}

void AR_WPNav::clear_stopping_line_state()
{
    _stopping_line_active = false;
    _stopping_line_speed_max_mps = 0.0f;
    _path_terminal = false;
    _path_target_speed_mps = 0.0f;
    clear_stopping_line_progress();
}

float AR_WPNav::calc_crosstrack_error(const Location& current_loc) const
{
    if (!_orig_and_dest_valid) {
        return 0.0f;
    }

    const Location &orig = get_oa_origin();
    const Location &dest = get_oa_destination();
    Vector2f dest_from_origin = orig.get_distance_NE(dest);
    if (dest_from_origin.length() < 1.0e-6f) {
        return current_loc.get_distance_NE(dest).length();
    }
    dest_from_origin.normalize();
    const Vector2f veh_from_origin = orig.get_distance_NE(current_loc);
    return veh_from_origin % dest_from_origin;
}

float AR_WPNav::get_corner_angle(const Location& loc1,
                                 const Location& loc2,
                                 const Location& loc3) const
{
    if (!loc1.initialised() || !loc2.initialised() || !loc3.initialised()) {
        return 0.0f;
    }
    const float loc1_to_loc2_deg = loc1.get_bearing_to(loc2) * 0.01f;
    const float loc2_to_loc3_deg = loc2.get_bearing_to(loc3) * 0.01f;
    return wrap_180(loc2_to_loc3_deg - loc1_to_loc2_deg);
}

void AR_WPNav::init_pos_control_if_necessary()
{
    if (!_pos_control.is_active() && !_pos_control.init()) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
}

bool AR_WPNav::set_origin_and_destination_to_stopping_point()
{
    Location stopping_loc;
    if (!get_stopping_location(stopping_loc)) {
        return false;
    }
    _origin = _destination = stopping_loc;
    _orig_and_dest_valid = true;
    return true;
}

void AR_WPNav::update_speed_max()
{
    const float speed_max = MAX(_base_speed_max, _nudge_speed_max);
    if (is_equal(speed_max, _pos_control.get_speed_max())) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_speed_update_ms) < AR_WPNAV_SPEED_UPDATE_MIN_MS) {
        return;
    }
    _last_speed_update_ms = now_ms;

    _pos_control.set_limits(speed_max,
                            _pos_control.get_accel_max(),
                            _pos_control.get_lat_accel_max(),
                            _pos_control.get_jerk_max());

    const float this_speed_max_mps = _stopping_line_active ?
        get_leg_speed_max(_stopping_line_speed_max_mps) :
        _pos_control.get_speed_max();
    const float next_speed_max_mps = _pos_control.get_speed_max();
    _scurve_this_leg.set_speed_max(this_speed_max_mps,
                                   this_speed_max_mps,
                                   this_speed_max_mps);
    _scurve_next_leg.set_speed_max(next_speed_max_mps,
                                   next_speed_max_mps,
                                   next_speed_max_mps);
}
