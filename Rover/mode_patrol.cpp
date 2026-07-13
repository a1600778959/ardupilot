#include "Rover.h"

namespace {
constexpr float patrol_dist_min = 0.0f;
constexpr float patrol_dist_max = 10.0f;
constexpr float patrol_dist_step = 0.5f;
constexpr float patrol_pivot_accuracy_deg = 5.0f;
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
    // @DisplayName: Patrol pivot progress timeout
    // @Description: Maximum time without meaningful heading improvement during a Patrol pivot turn. Zero disables the watchdog.
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
    g2.wp_nav.init();
    _pivot_watchdog.reset();

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
    _pivot_watchdog.reset();
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
    _point_a = Location();
    _point_b = Location();
    _line_start = Location();
    _line_end = Location();
    _point_count = 0;
    _route.reset();
    _pivot_watchdog.reset();
}

void ModePatrol::update()
{
    if (_route.waiting_for_points()) {
        float desired_steering, desired_throttle;
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
        desired_steering = 4500.0f * input_expo(desired_steering / 4500.0f, g2.manual_steering_expo);
        g2.motors.set_throttle(desired_throttle);
        g2.motors.set_steering(desired_steering);
        return;
    }

    if (_route.in_fault()) {
        stop_vehicle();
        return;
    }

    if (g2.wp_nav.reached_destination() && !advance_to_next_target()) {
        stop_vehicle();
        return;
    }

    navigate_to_waypoint();
    if (pivot_stalled()) {
        set_fault("Patrol: pivot stalled");
        stop_vehicle();
    }
}

bool ModePatrol::start_patrol()
{
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

    const float spacing = get_spacing_m() * line_index;
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
    // Patrol endpoints must be reached strictly; do not provide next_destination
    // because that enables fast-waypoint smoothing and cuts line endpoints.
    if (!Mode::set_desired_location(destination)) {
        return false;
    }

    _line_start = line_start;
    _line_end = line_end;
    _route.commit(target);
    _pivot_watchdog.reset();
    return true;
}

bool ModePatrol::advance_to_next_target()
{
    ModePatrolRoute::Target target {};
    if (!_route.next_target(target)) {
        set_fault("Patrol: route exhausted");
        return false;
    }

    Location line_start = _line_start;
    Location line_end = _line_end;
    if (target.starts_new_line && !make_offset_leg(target.line_index, line_start, line_end)) {
        set_fault("Patrol: invalid next line");
        return false;
    }

    const Location &destination = (target.phase == ModePatrolRoute::Phase::LineStart) ? line_start : line_end;
    if (!set_target(target, destination, line_start, line_end)) {
        set_fault(target.starts_new_line ? "Patrol: failed to set next line" : "Patrol: failed to set line end");
        return false;
    }

    if (target.starts_new_line) {
        gcs().send_text(MAV_SEVERITY_INFO, "Patrol: line %u spacing %.1fm",
                        (unsigned)_route.line_index(),
                        (double)get_spacing_m());
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
    gcs().send_text(MAV_SEVERITY_INFO, "Patrol: next spacing %.1fm", (double)get_spacing_m());
}

bool ModePatrol::pivot_stalled()
{
    const float timeout_s = constrain_float(_pivot_timeout_s.get(), 0.0f, 60.0f);
    if (!hal.util->get_soft_armed() ||
        !is_positive(timeout_s) ||
        !g2.wp_nav.is_pivot_active() ||
        !is_zero(g2.wp_nav.get_speed())) {
        _pivot_watchdog.reset();
        return false;
    }

    const float heading_error_deg = g2.wp_nav.get_pivot_heading_error_deg();
    // AR_PivotTurn only starts its normal completion delay below 5 degrees.
    if (!isfinite(heading_error_deg) || (heading_error_deg < patrol_pivot_accuracy_deg)) {
        _pivot_watchdog.reset();
        return false;
    }

    const uint32_t timeout_ms = static_cast<uint32_t>(timeout_s * 1000.0f);
    return _pivot_watchdog.update(true, heading_error_deg, AP_HAL::millis(), timeout_ms);
}

void ModePatrol::set_fault(const char *message)
{
    if (_route.set_fault()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "%s", message);
    }
    _pivot_watchdog.reset();
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
