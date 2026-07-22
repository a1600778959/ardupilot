#include "Rover.h"

namespace {
constexpr float patrol_dist_min = 0.0f;
constexpr float patrol_dist_max = 10.0f;
constexpr float patrol_dist_step = 0.5f;
}

const AP_Param::GroupInfo ModePatrol::var_info[] = {
    // @Param: DIST
    // @DisplayName: Patrol spacing
    // @Description: Fixed lateral spacing from the manually saved patrol A-B line.
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("DIST", 1, ModePatrol, _dist, 5.0f),

    // @Param: PIVOT_TOUT
    // @DisplayName: Deprecated Patrol pivot timeout
    // @Description: Deprecated and ignored. Patrol exact-pivot timeout is controlled by WP_PIVOT_TOUT. This parameter is retained for storage compatibility.
    // @Units: s
    // @Range: 0 60
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PIVOT_TOUT", 2, ModePatrol, _pivot_timeout_s, 15.0f),

    AP_GROUPEND
};

ModePatrol::ModePatrol() :
    Mode(),
    _point_count(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModePatrol::_enter()
{
    // Patrol geometry is always planned for forward differential-drive travel.
    // Clear any reverse state inherited from the previous mode before WPNav
    // creates its first heading or entry pivot.
    set_reversed(false);
    g2.wp_nav.init();

    if (_point_count < 2) {
        _route.reset();
        gcs().send_text(MAV_SEVERITY_NOTICE, "Patrol: save A/B points");
        return true;
    }

    if (_route.navigating()) {
        if (!resume_patrol()) {
            stop_vehicle();
        }
        return true;
    }

    if (!start_patrol()) {
        stop_vehicle();
    }
    return true;
}

void ModePatrol::_exit()
{
    stop_vehicle();
}

void ModePatrol::save_point()
{
    if (rover.control_mode != this) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Patrol: switch to patrol mode");
        return;
    }

    if (!rover.have_position) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Patrol: position required");
        return;
    }

    if (_point_count >= 2) {
        clear_points();
    }

    if (_point_count == 0) {
        _point_a = rover.current_loc;
        _point_count = 1;
        _route.reset();
        gcs().send_text(MAV_SEVERITY_NOTICE, "Patrol: saved point A");
        return;
    }

    _point_b = rover.current_loc;
    _point_count = 2;
    gcs().send_text(MAV_SEVERITY_NOTICE, "Patrol: saved point B");
    if (!start_patrol()) {
        stop_vehicle();
    }
}

void ModePatrol::clear_points()
{
    if (rover.control_mode == this) {
        // A/B recapture replaces Patrol's complete navigation transaction.
        // Cancel a running Exact preview before the first new point is stored,
        // otherwise the new route setter correctly rejects it as a mutation of
        // the still-active old transaction.
        g2.wp_nav.init();
    }
    _point_a = Location();
    _point_b = Location();
    _line_start = Location();
    _line_end = Location();
    _point_count = 0;
    _route.reset();
    _preview_valid = false;
    _planning_spacing_m = get_spacing_m();
    _planning_spacing_valid = true;
}

void ModePatrol::update()
{
    // Patrol geometry is forward-only.  Reassert this every cycle so a runtime
    // reverse command cannot invalidate the current MOVE-to-SPIN transaction.
    Mode::set_reversed(false);

    if (_route.waiting_for_points()) {
        float desired_steering, desired_throttle;
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
        desired_steering = 4500.0f * input_expo(desired_steering / 4500.0f, g2.manual_steering_expo);
        g2.motors.set_throttle(desired_throttle);
        g2.motors.set_steering(desired_steering);
        return;
    }

    if (_route.in_fault()) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    if (g2.wp_nav.reached_destination() && !advance_to_next_target()) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    navigate_to_waypoint();
    if (g2.wp_nav.exact_pivot_failed()) {
        const AR_WPNav::ExactPivotFault pivot_fault = g2.wp_nav.get_exact_pivot_fault();
        if (pivot_fault == AR_WPNav::ExactPivotFault::RecoveryInfeasible) {
            set_fault("Patrol: pivot recovery infeasible, hold");
        } else if (pivot_fault == AR_WPNav::ExactPivotFault::RecoveryBounds) {
            set_fault("Patrol: pivot recovery bounds, hold");
        } else if (pivot_fault == AR_WPNav::ExactPivotFault::Estimator) {
            set_fault("Patrol: pivot estimator fault, hold");
        } else if (pivot_fault == AR_WPNav::ExactPivotFault::Timeout) {
            set_fault("Patrol: pivot timeout, hold");
        } else {
            set_fault("Patrol: pivot controller fault, hold");
        }
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }
}

bool ModePatrol::start_patrol()
{
    // A new route has no preview transaction to preserve, so the current
    // parameter becomes its planning spacing immediately.
    _planning_spacing_m = get_spacing_m();
    _planning_spacing_valid = true;
    _preview_valid = false;

    const ModePatrolRoute::Target target = _route.start_target();
    Location line_start;
    Location line_end;
    if (!make_offset_leg(target.line_index, line_start, line_end)) {
        set_fault("Patrol: invalid A/B line");
        return false;
    }

    if (!set_target(target, line_start, line_start, line_end)) {
        set_fault("Patrol: failed to set start");
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Patrol: line %u spacing %.1fm",
                    (unsigned)_route.line_index(),
                    (double)get_spacing_m());
    return true;
}

bool ModePatrol::resume_patrol()
{
    if (!_planning_spacing_valid) {
        _planning_spacing_m = get_spacing_m();
        _planning_spacing_valid = true;
    }
    ModePatrolRoute::Target target {};
    if (!_route.resume_target(target)) {
        set_fault("Patrol: invalid resume state");
        return false;
    }

    const Location &destination = (target.phase == ModePatrolRoute::Phase::LineStart) ? _line_start : _line_end;
    if (!set_target(target, destination, _line_start, _line_end)) {
        set_fault("Patrol: failed to resume");
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Patrol: resume line %u", (unsigned)_route.line_index());
    return true;
}

bool ModePatrol::make_offset_leg(const uint16_t line_index, Location &start, Location &end)
{
    if (line_index == 0) {
        return false;
    }

    const Vector2f ab = _point_a.get_distance_NE(_point_b);
    const float ab_len = ab.length();
    if (ab_len < 1.0f) {
        return false;
    }

    const float spacing = _planning_spacing_m * line_index;
    const Vector2f normal {-ab.y / ab_len, ab.x / ab_len};
    const Vector2f offset = normal * spacing;

    const bool start_from_b = (line_index % 2) == 1;
    start = start_from_b ? _point_b : _point_a;
    end = start_from_b ? _point_a : _point_b;
    start.offset(offset.x, offset.y);
    end.offset(offset.x, offset.y);
    return true;
}

bool ModePatrol::set_target(const ModePatrolRoute::Target &target,
                            const Location &destination,
                            const Location &line_start,
                            const Location &line_end)
{
    // Reassert the forward-only contract before creating every transaction.
    Mode::set_reversed(false);

    // Refresh differential-drive pivot capability synchronously so Patrol cannot fault
    // merely because the one-second propagation task has not run since boot.
    g2.wp_nav.set_turn_params(g2.turn_radius, g2.motors.have_skid_steering());

    ModePatrolRoute::Target next_target {};
    Location next_destination;
    Location next_line_start;
    Location next_line_end;
    bool line_changed;
    if (!get_next_distinct_target(target,
                                  destination,
                                  line_start,
                                  line_end,
                                  next_target,
                                  next_destination,
                                  next_line_start,
                                  next_line_end,
                                  line_changed)) {
        return false;
    }

    // Patrol owns immutable A/B-derived geometry for the whole exact-pivot
    // transaction.  Keep OA from replacing either frozen leg with a dynamic
    // destination before WPNav acknowledges/promotes it.
    g2.wp_nav.set_exact_pivot_oa_bypass(true);
    if (!g2.wp_nav.set_desired_location_exact_pivot(destination, next_destination)) {
        return false;
    }
    _distance_to_destination = g2.wp_nav.get_distance_to_destination();
    _reached_destination = false;

    // Commit the route and its already planned successor atomically.  Handoff
    // consumes these exact locations instead of deriving them again from a
    // possibly changed PTRL_DIST value.
    _line_start = line_start;
    _line_end = line_end;
    _preview_target = next_target;
    _preview_destination = next_destination;
    _preview_line_start = next_line_start;
    _preview_line_end = next_line_end;
    _preview_line_changed = line_changed;
    _preview_spacing_m = _planning_spacing_m;
    _preview_valid = true;
    _route.commit(target);
    return true;
}

bool ModePatrol::get_target_geometry(const ModePatrolRoute::Target &target,
                                     const Location &current_line_start,
                                     const Location &current_line_end,
                                     Location &destination,
                                     Location &line_start,
                                     Location &line_end)
{
    line_start = current_line_start;
    line_end = current_line_end;
    if (target.starts_new_line && !make_offset_leg(target.line_index, line_start, line_end)) {
        return false;
    }
    destination = (target.phase == ModePatrolRoute::Phase::LineStart) ? line_start : line_end;
    return destination.initialised();
}

bool ModePatrol::get_next_distinct_target(const ModePatrolRoute::Target &current_target,
                                          const Location &current_destination,
                                          const Location &current_line_start,
                                          const Location &current_line_end,
                                          ModePatrolRoute::Target &next_target,
                                          Location &next_destination,
                                          Location &next_line_start,
                                          Location &next_line_end,
                                          bool &line_changed)
{
    ModePatrolRoute::Target candidate = current_target;
    Location candidate_line_start = current_line_start;
    Location candidate_line_end = current_line_end;
    line_changed = false;

    ModePatrolRoute::Target following {};
    while (ModePatrolRoute::target_after(candidate, following)) {
        candidate = following;
        if (!get_target_geometry(candidate,
                                 candidate_line_start,
                                 candidate_line_end,
                                 next_destination,
                                 next_line_start,
                                 next_line_end)) {
            return false;
        }

        line_changed |= candidate.starts_new_line;
        candidate_line_start = next_line_start;
        candidate_line_end = next_line_end;
        if (!current_destination.same_latlon_as(next_destination)) {
            next_target = candidate;
            return true;
        }
    }

    return false;
}

bool ModePatrol::advance_to_next_target()
{
    if (!_preview_valid) {
        set_fault("Patrol: missing route preview");
        return false;
    }

    const ModePatrolRoute::Target target = _preview_target;
    const Location destination = _preview_destination;
    const Location line_start = _preview_line_start;
    const Location line_end = _preview_line_end;
    const bool line_changed = _preview_line_changed;
    const float target_spacing_m = _preview_spacing_m;

    // The copied target is exactly the leg already previewed by WPNav.  A
    // pending parameter change is promoted only now, so it affects the future
    // preview built inside set_target(), never the leg being handed off.
    _planning_spacing_m = get_spacing_m();
    _planning_spacing_valid = true;

    // Mark the copied preview consumed only for the duration of the ACK. A
    // failure restores its validity, so Patrol cannot silently lose the route;
    // success atomically installs the newly generated successor in set_target().
    _preview_valid = false;
    if (!set_target(target, destination, line_start, line_end)) {
        _preview_valid = true;
        set_fault(target.starts_new_line ? "Patrol: failed to set next line" : "Patrol: failed to set line end");
        return false;
    }

    if (line_changed) {
        gcs().send_text(MAV_SEVERITY_INFO, "Patrol: line %u spacing %.1fm",
                        (unsigned)_route.line_index(),
                        (double)target_spacing_m);
    }
    return true;
}

float ModePatrol::get_spacing_m() const
{
    return constrain_float(_dist.get(), patrol_dist_min, patrol_dist_max);
}

void ModePatrol::adjust_spacing(int8_t direction)
{
    if (direction == 0) {
        return;
    }

    const int8_t step_direction = direction > 0 ? 1 : -1;
    const float new_spacing = constrain_float(get_spacing_m() + (patrol_dist_step * step_direction),
                                              patrol_dist_min,
                                              patrol_dist_max);
    _dist.set_and_save_ifchanged(new_spacing);
    if (!_route.navigating()) {
        _planning_spacing_m = new_spacing;
        _planning_spacing_valid = true;
    }
    gcs().send_text(MAV_SEVERITY_INFO,
                    _route.navigating() ? "Patrol: queued spacing %.1fm" : "Patrol: spacing %.1fm",
                    (double)new_spacing);
}

void ModePatrol::set_fault(const char *message)
{
    if (_route.set_fault()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "%s", message);
    }
}

float ModePatrol::wp_bearing() const
{
    if (!_route.navigating()) {
        return 0.0f;
    }
    return g2.wp_nav.wp_bearing_cd() * 0.01f;
}

float ModePatrol::nav_bearing() const
{
    if (!_route.navigating()) {
        return 0.0f;
    }
    return g2.wp_nav.nav_bearing_cd() * 0.01f;
}

float ModePatrol::crosstrack_error() const
{
    if (!_route.navigating()) {
        return 0.0f;
    }
    return g2.wp_nav.crosstrack_error();
}

float ModePatrol::get_desired_lat_accel() const
{
    if (!_route.navigating()) {
        return 0.0f;
    }
    return g2.wp_nav.get_lat_accel();
}

bool ModePatrol::get_desired_location(Location& destination) const
{
    if (!_route.navigating()) {
        return false;
    }
    if (!g2.wp_nav.is_destination_valid()) {
        return false;
    }
    destination = g2.wp_nav.get_oa_destination();
    return true;
}

bool ModePatrol::set_desired_speed(float speed)
{
    return g2.wp_nav.set_speed_max(speed);
}
