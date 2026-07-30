#include "Rover.h"

namespace {
constexpr float patrol_dist_min = 0.1f;
constexpr float patrol_dist_max = 10.0f;
constexpr float patrol_dist_step = 0.5f;
constexpr float patrol_transition_speed_min = 0.05f;
constexpr float patrol_transition_speed_max = 2.0f;
constexpr float patrol_transition_speed_default = 0.5f;
constexpr float patrol_ab_length_min = 1.0f;
constexpr uint32_t patrol_dynamic_log_period_ms = 100U;
constexpr uint32_t patrol_geometry_log_period_ms = 1000U;
constexpr float patrol_path_speed_stopped_mps = 0.02f;
constexpr float patrol_yaw_rate_stopped_rads = radians(5.0f);
constexpr uint32_t patrol_boundary_violation_ms = 500U;

const char *spacing_error_message(float spacing_m)
{
    return is_positive(spacing_m) ?
        "Patrol: spacing must be 0.1-10m" :
        "Patrol: spacing must be > 0";
}

bool boundary_violation_elapsed(uint32_t &start_ms, uint32_t now_ms)
{
    if (start_ms == 0U) {
        start_ms = MAX(now_ms, 1U);
        return false;
    }
    return (now_ms - start_ms) >= patrol_boundary_violation_ms;
}
}

const AP_Param::GroupInfo ModePatrol::var_info[] = {
    // @Param: DIST
    // @DisplayName: Patrol spacing
    // @Description: Lateral distance from the first patrol line to the saved A-B baseline and, while running, the distance between the current planned line and the following line
    // @Units: m
    // @Range: 0.1 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("DIST", 1, ModePatrol, _dist, 5.0f),

    // Index 2 was PIVOT_TOUT. It is permanently reserved for storage
    // compatibility and must not be reused for a different Patrol parameter.

    // @Param: TRANS_SPD
    // @DisplayName: Patrol transition speed
    // @Description: Maximum speed for the initial transit to the first patrol line and for perpendicular transitions between adjacent patrol lines
    // @Units: m/s
    // @Range: 0.05 2.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("TRANS_SPD", 3, ModePatrol, _transition_speed, patrol_transition_speed_default),

    // @Param: MAX_RUNS
    // @DisplayName: Maximum patrol runs
    // @Description: Maximum number of completed work lines before Patrol stops. Zero allows unlimited operation
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX_RUNS", 4, ModePatrol, _max_runs, 0),

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
    // Patrol geometry is forward-only. Clear reverse before any theoretical
    // route or turn direction is calculated.
    set_reversed(false);

    if (_point_count < 2) {
        g2.wp_nav.init();
        reset_drive_boundary_monitors();
        _motion_state = MotionState::WaitingForPoints;
        _fault_reason = FaultReason::None;
        _fault_latched = false;
        _recapture_rejection_reported = false;
        gcs().send_text(MAV_SEVERITY_NOTICE, "Patrol: save A/B points");
        return true;
    }

    const float configured_spacing_m = _dist.get();
    if (!spacing_is_valid(configured_spacing_m)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "%s",
                        spacing_error_message(configured_spacing_m));
        return false;
    }

    g2.wp_nav.init();
    g2.wp_nav.set_turn_params(g2.turn_radius, g2.motors.have_skid_steering());
    _queued_spacing_m = configured_spacing_m;
    _last_parameter_spacing_m = configured_spacing_m;

    if ((_motion_state == MotionState::Hold) && _fault_latched) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Patrol: fault latched, disarm to clear");
        write_patrol_log(LogEvent::Hold, true);
        return true;
    }

    if (_motion_state == MotionState::Completed) {
        gcs().send_text(MAV_SEVERITY_NOTICE,
                        "Patrol: completed %u runs",
                        (unsigned)_active_target.line_index);
        write_patrol_log(LogEvent::Complete, true);
        return true;
    }

    const bool saved_route = (_active_target.line_index != 0U) && _next_valid;
    const bool started = saved_route ? resume_patrol() : start_patrol();
    if (!started) {
        stop_vehicle();
    }
    return true;
}

void ModePatrol::_exit()
{
    g2.wp_nav.cancel_stopping_line();
    reset_drive_boundary_monitors();
    _spin_pivot_started = false;
    if (_motion_state != MotionState::WaitingForPoints) {
        write_patrol_log(LogEvent::Release, true);
    }
    stop_vehicle();
}

void ModePatrol::save_point()
{
    if (rover.control_mode != this) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Patrol: switch to patrol mode");
        return;
    }

    if ((_motion_state != MotionState::WaitingForPoints) || (_point_count >= 2)) {
        if (!_recapture_rejection_reported) {
            _recapture_rejection_reported = true;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "Patrol: stop/disarm before recapture");
            write_patrol_log(LogEvent::RecaptureRejected, true);
        }
        return;
    }

    if (!rover.have_position || !rover.current_loc.initialised()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Patrol: position required");
        return;
    }

    if (_point_count == 0) {
        _point_a = rover.current_loc;
        _point_count = 1;
        _motion_state = MotionState::WaitingForPoints;
        _fault_reason = FaultReason::None;
        _fault_latched = false;
        _recapture_rejection_reported = false;
        gcs().send_text(MAV_SEVERITY_NOTICE, "Patrol: saved point A");
        write_patrol_log(LogEvent::SaveA, true);
        return;
    }

    _point_b = rover.current_loc;
    _point_count = 2;
    gcs().send_text(MAV_SEVERITY_NOTICE, "Patrol: saved point B");
    write_patrol_log(LogEvent::SaveB, true);

    const float configured_spacing_m = _dist.get();
    if (!spacing_is_valid(configured_spacing_m)) {
        set_fault(spacing_error_message(configured_spacing_m),
                  FaultReason::Configuration);
        return;
    }

    if (!start_patrol() && (_motion_state != MotionState::Hold)) {
        set_fault("Patrol: failed to start", FaultReason::Internal);
    }
}

void ModePatrol::clear_points()
{
    const bool had_state = (_point_count != 0) ||
                           (_motion_state != MotionState::WaitingForPoints);
    g2.wp_nav.cancel_stopping_line();

    _point_a = Location();
    _point_b = Location();
    _active_target = ModePatrolRoute::Target{};
    _active_origin = Location();
    _active_destination = Location();
    _active_offset_m = 0.0f;
    _active_spacing_m = 0.0f;
    _next_target = ModePatrolRoute::Target{};
    _next_destination = Location();
    _next_line_start = Location();
    _next_line_end = Location();
    _next_offset_m = 0.0f;
    _next_spacing_m = 0.0f;
    _next_valid = false;
    _point_count = 0;
    _motion_state = MotionState::WaitingForPoints;
    _fault_reason = FaultReason::None;
    _fault_latched = false;
    reset_drive_boundary_monitors();
    _spin_drift_violation_start_ms = 0U;
    _spin_purpose = SpinPurpose::TurnToNextLeg;
    _spin_pivot_started = false;
    _spin_anchor = Location();
    _recapture_rejection_reported = false;
    _last_heartbeat_ms = 0U;
    _last_geometry_log_ms = 0U;
    _last_parameter_spacing_m = _dist.get();
    _queued_spacing_m = spacing_is_valid(_last_parameter_spacing_m) ?
        _last_parameter_spacing_m : 0.0f;

    if (had_state) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Patrol: points cleared");
        // Record the post-clear state so the log proves that the route and its
        // theoretical endpoints no longer remain active.
        write_patrol_log(LogEvent::Clear, true);
    }
}

void ModePatrol::update()
{
    // Patrol geometry and turn direction are always forward and theoretical.
    Mode::set_reversed(false);

    if ((_point_count >= 2) && !refresh_spacing_queue(true)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        write_patrol_heartbeat();
        return;
    }

    switch (_motion_state) {
    case MotionState::WaitingForPoints: {
        float desired_steering;
        float desired_throttle;
        get_pilot_desired_steering_and_throttle(desired_steering,
                                                desired_throttle);
        desired_steering = 4500.0f * input_expo(desired_steering / 4500.0f,
                                                g2.manual_steering_expo);
        g2.motors.set_throttle(desired_throttle);
        g2.motors.set_steering(desired_steering);
        return;
    }
    case MotionState::Hold:
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        break;
    case MotionState::Completed:
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        break;
    case MotionState::Drive:
        update_drive();
        break;
    case MotionState::Spin:
        update_spin();
        break;
    default:
        hold("Patrol: invalid runtime state, hold", FaultReason::Internal, true);
        break;
    }
    write_patrol_heartbeat();
}

void ModePatrol::update_drive()
{
    Location current_loc;
    Vector3f velocity_ned;
    float forward_speed;
    const float yaw_rate_rads = AP::ahrs().get_yaw_rate_earth();
    if (!rover.have_position ||
        !AP::ahrs().get_location(current_loc) ||
        !AP::ahrs().get_velocity_NED(velocity_ned) ||
        !attitude_control.get_forward_speed(forward_speed) ||
        !isfinite(yaw_rate_rads)) {
        hold("Patrol: estimator unavailable, hold",
             FaultReason::Estimator,
             false);
        return;
    }

    // Patrol owns both translation and yaw.  OA and all pilot motion mixing are
    // bypassed by the explicit stopping-line contract.
    navigate_to_waypoint(false, false);
    _distance_to_destination = current_loc.get_distance(_active_destination);

    // Leave ordinary cross-track recovery to WPNav. Hold only when the error
    // exceeds the configured recovery envelope, steering saturates and a full
    // observation window shows no improvement.
    if (!isfinite(g2.wp_nav.crosstrack_error())) {
        hold("Patrol: cross-track estimator fault, hold",
             FaultReason::Estimator,
             false);
        return;
    }

    const Vector2f track =
        _active_origin.get_distance_NE(_active_destination);
    const float track_length_m = track.length();
    const Vector2f vehicle_from_origin =
        _active_origin.get_distance_NE(current_loc);
    if (!isfinite(track_length_m) ||
        (track_length_m < 0.01f) ||
        !isfinite(vehicle_from_origin.x) ||
        !isfinite(vehicle_from_origin.y)) {
        hold("Patrol: drive geometry invalid, hold",
             FaultReason::RouteGeometry,
             true);
        return;
    }

    const Vector2f track_unit = track / track_length_m;
    const float along_overrun_m =
        vehicle_from_origin.dot(track_unit) - track_length_m;
    const float xtrack_m = fabsf(vehicle_from_origin % track_unit);
    const float endpoint_radius_m = g2.wp_nav.get_pivot_radius();
    if (!isfinite(_distance_to_destination) ||
        !isfinite(along_overrun_m) || !isfinite(xtrack_m) ||
        !isfinite(endpoint_radius_m)) {
        hold("Patrol: drive boundary estimator fault, hold",
             FaultReason::Estimator,
             false);
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const bool terminal_miss =
        (_distance_to_destination > endpoint_radius_m) &&
        !is_negative(along_overrun_m);
    if (terminal_miss) {
        // A forward-only theoretical line cannot remove a lateral miss after
        // reaching the finish plane. Bound the estimator-noise edge, then
        // report the failure instead of remaining indefinitely in Drive with
        // a zero terminal velocity target.
        if (boundary_violation_elapsed(_terminal_miss_violation_start_ms,
                                       now_ms)) {
            hold("Patrol: past endpoint, hold",
                 FaultReason::PastFinishPlane,
                 false);
            return;
        }
        // PastFinishPlane owns the fault classification while this condition
        // is pending; do not let the same position become CrossTrack first.
        _xtrack_violation_start_ms = 0U;
        _xtrack_violation_start_m = 0.0f;
        _xtrack_steering_limited = false;
        return;
    }
    _terminal_miss_violation_start_ms = 0U;

    const float recovery_limit_m =
        MAX(endpoint_radius_m, g2.wp_nav.get_pivot_drift());
    if (!isfinite(recovery_limit_m)) {
        hold("Patrol: drive recovery limit invalid, hold",
             FaultReason::Estimator,
             false);
        return;
    }
    if (xtrack_m <= recovery_limit_m) {
        _xtrack_violation_start_ms = 0U;
        _xtrack_violation_start_m = 0.0f;
        _xtrack_steering_limited = false;
    } else {
        const bool steering_limited = g2.motors.limit.steer_left ||
                                      g2.motors.limit.steer_right;
        if (_xtrack_violation_start_ms == 0U) {
            _xtrack_violation_start_ms = MAX(now_ms, 1U);
            _xtrack_violation_start_m = xtrack_m;
            _xtrack_steering_limited = steering_limited;
        } else {
            _xtrack_steering_limited |= steering_limited;
            if ((now_ms - _xtrack_violation_start_ms) >=
                patrol_boundary_violation_ms) {
                if (_xtrack_steering_limited &&
                    (xtrack_m >= _xtrack_violation_start_m)) {
                    hold("Patrol: cross-track diverging, hold",
                         FaultReason::CrossTrack,
                         false);
                    return;
                }
                // The controller is recovering or still has steering
                // authority. Start a fresh observation window without
                // turning a recoverable cross-track error into a self-lock.
                _xtrack_violation_start_ms = MAX(now_ms, 1U);
                _xtrack_violation_start_m = xtrack_m;
                _xtrack_steering_limited = steering_limited;
            }
        }
    }

    if (!drive_completion_conditions_met(current_loc)) {
        return;
    }

    write_patrol_log(LogEvent::EndpointReached, true);
    if (run_limit_reached()) {
        complete_patrol();
        return;
    }
    if (!begin_spin()) {
        hold("Patrol: invalid turn geometry, hold",
             FaultReason::RouteGeometry,
             true);
        return;
    }
    // The first Spin output is produced on the same scheduler cycle as the
    // physical endpoint/stopped edge.
    update_spin();
}

void ModePatrol::complete_patrol()
{
    g2.wp_nav.cancel_stopping_line();
    reset_drive_boundary_monitors();
    _spin_pivot_started = false;
    _spin_anchor = Location();
    _motion_state = MotionState::Completed;
    _fault_reason = FaultReason::None;
    _fault_latched = false;
    g2.motors.set_throttle(0.0f);
    g2.motors.set_steering(0.0f);
    gcs().send_text(MAV_SEVERITY_NOTICE,
                    "Patrol: completed %u runs",
                    (unsigned)_active_target.line_index);
    write_patrol_log(LogEvent::Complete, true);
}

bool ModePatrol::drive_completion_conditions_met(const Location &current_loc) const
{
    if (!g2.wp_nav.path_terminal() ||
        !isfinite(g2.wp_nav.get_path_target_speed()) ||
        (g2.wp_nav.get_path_target_speed() > patrol_path_speed_stopped_mps) ||
        (current_loc.get_distance(_active_destination) > g2.wp_nav.get_pivot_radius())) {
        return false;
    }

    float forward_speed;
    Vector3f velocity_ned;
    const float yaw_rate_rads = AP::ahrs().get_yaw_rate_earth();
    if (!attitude_control.get_forward_speed(forward_speed) ||
        !AP::ahrs().get_velocity_NED(velocity_ned) ||
        !isfinite(forward_speed) ||
        !isfinite(velocity_ned.x) ||
        !isfinite(velocity_ned.y) ||
        !isfinite(yaw_rate_rads)) {
        return false;
    }

    const Vector2f track =
        _active_origin.get_distance_NE(_active_destination);
    const float track_length_m = track.length();
    if (!isfinite(track_length_m) || (track_length_m < 0.01f)) {
        return false;
    }
    const float track_speed_mps =
        velocity_ned.xy().dot(track / track_length_m);
    if (!isfinite(track_speed_mps)) {
        return false;
    }

    const float stop_speed_mps = MAX(attitude_control.get_stop_speed(),
                                     patrol_path_speed_stopped_mps);
    return (fabsf(forward_speed) <= stop_speed_mps) &&
           (fabsf(track_speed_mps) <= stop_speed_mps) &&
           (fabsf(yaw_rate_rads) <= patrol_yaw_rate_stopped_rads);
}

bool ModePatrol::calculate_planned_turn(float &heading_cd,
                                        int8_t &preferred_direction) const
{
    if (!_active_origin.initialised() ||
        !_active_destination.initialised() ||
        !_next_destination.initialised()) {
        return false;
    }

    const Vector2f incoming = _active_origin.get_distance_NE(_active_destination);
    const Vector2f outgoing =
        _active_destination.get_distance_NE(_next_destination);
    const float incoming_length_m = incoming.length();
    const float outgoing_length_m = outgoing.length();
    if (!isfinite(incoming_length_m) || !isfinite(outgoing_length_m) ||
        (incoming_length_m < 0.01f) || (outgoing_length_m < 0.01f)) {
        return false;
    }

    const float cross = (incoming / incoming_length_m) %
                        (outgoing / outgoing_length_m);
    if (!isfinite(cross)) {
        return false;
    }
    // North-East coordinates use a negative cross product for a left turn.
    // Collinear 180-degree geometry uses the same deterministic left tie-break.
    preferred_direction = is_positive(cross) ? 1 : -1;
    heading_cd = _active_destination.get_bearing_to(_next_destination);
    return isfinite(heading_cd);
}

bool ModePatrol::begin_spin()
{
    int8_t spin_direction;
    if (!calculate_planned_turn(_spin_heading_cd, spin_direction) ||
        !g2.wp_nav.start_planned_pivot(_spin_heading_cd, spin_direction)) {
        return false;
    }

    // Do not carry line-tracking integral into the in-place turn. A residual
    // steering I term can kick one side of a differential drive just as the
    // planned turn-rate ramp starts.
    attitude_control.get_steering_rate_pid().reset_I();
    reset_drive_boundary_monitors();
    _spin_purpose = SpinPurpose::TurnToNextLeg;
    _spin_pivot_started = true;
    _spin_anchor = _active_destination;
    _motion_state = MotionState::Spin;
    _spin_drift_violation_start_ms = 0U;
    g2.motors.set_throttle(0.0f);
    write_patrol_log(LogEvent::SpinStart, true);
    return true;
}

bool ModePatrol::begin_drive_alignment()
{
    if ((_active_target.leg_type !=
         ModePatrolRoute::LegType::InitialTransit) ||
        !_active_origin.initialised() ||
        !_active_destination.initialised()) {
        return false;
    }

    const Vector2f drive_vector =
        _active_origin.get_distance_NE(_active_destination);
    const float drive_length_m = drive_vector.length();
    Location current_loc;
    if (!isfinite(drive_length_m) ||
        (drive_length_m < 0.01f) ||
        !AP::ahrs().get_location(current_loc)) {
        return false;
    }

    _spin_heading_cd =
        current_loc.get_bearing_to(_active_destination);
    if (!isfinite(_spin_heading_cd)) {
        return false;
    }

    _spin_purpose = SpinPurpose::AlignDrive;
    reset_drive_boundary_monitors();
    _spin_pivot_started = false;
    _spin_anchor = current_loc;
    _motion_state = MotionState::Spin;
    _spin_drift_violation_start_ms = 0U;
    g2.motors.set_throttle(0.0f);
    g2.motors.set_steering(0.0f);
    gcs().send_text(MAV_SEVERITY_INFO,
                    "Patrol: align Initial line %u",
                    (unsigned)_active_target.line_index);
    write_patrol_log(LogEvent::SpinStart, true);
    return true;
}

void ModePatrol::update_spin()
{
    Location current_loc;
    const float yaw_rate_rads = AP::ahrs().get_yaw_rate_earth();
    if (!rover.have_position ||
        !AP::ahrs().get_location(current_loc) ||
        !isfinite(AP::ahrs().get_yaw()) ||
        !isfinite(yaw_rate_rads)) {
        hold("Patrol: spin estimator fault, hold",
             FaultReason::Estimator,
             false);
        return;
    }

    if (!_spin_anchor.initialised()) {
        hold("Patrol: spin anchor invalid, hold",
             FaultReason::RouteGeometry,
             true);
        return;
    }

    const float radius_m = g2.wp_nav.get_pivot_radius();
    const float allowed_distance_m = radius_m +
        MAX(g2.wp_nav.get_pivot_drift(), 0.5f * radius_m);
    const float distance_m = current_loc.get_distance(_spin_anchor);
    const uint32_t now_ms = AP_HAL::millis();
    if (!isfinite(distance_m)) {
        hold("Patrol: spin position fault, hold",
             FaultReason::Estimator,
             false);
        return;
    }
    if (distance_m > allowed_distance_m) {
        if (boundary_violation_elapsed(_spin_drift_violation_start_ms,
                                       now_ms)) {
            hold("Patrol: spin drift limit, hold",
                 FaultReason::SpinDrift,
                 false);
            return;
        }
    } else {
        _spin_drift_violation_start_ms = 0U;
    }

    // Saving B may occur while the vehicle is still moving. Gate the start of
    // the alignment pivot once, then let the planned pivot run continuously.
    // Reapplying this gate after rotation starts turns GPS lever-arm velocity
    // into a repeated stop-turn-stop cycle.
    if ((_spin_purpose == SpinPurpose::AlignDrive) &&
        !_spin_pivot_started) {
        Vector3f velocity_ned;
        float forward_speed;
        if (!AP::ahrs().get_velocity_NED(velocity_ned) ||
            !attitude_control.get_forward_speed(forward_speed) ||
            !isfinite(forward_speed) ||
            !isfinite(velocity_ned.x) ||
            !isfinite(velocity_ned.y)) {
            hold("Patrol: align estimator fault, hold",
                 FaultReason::Estimator,
                 false);
            return;
        }
        const Vector2f track =
            current_loc.get_distance_NE(_active_destination);
        const float track_length_m = track.length();
        if (!isfinite(track_length_m) || (track_length_m < 0.01f)) {
            hold("Patrol: align route geometry invalid, hold",
                 FaultReason::RouteGeometry,
                 true);
            return;
        }
        const float track_speed_mps =
            velocity_ned.xy().dot(track / track_length_m);
        const float stop_speed_mps =
            MAX(attitude_control.get_stop_speed(),
                patrol_path_speed_stopped_mps);
        if ((fabsf(forward_speed) > stop_speed_mps) ||
            !isfinite(track_speed_mps) ||
            (fabsf(track_speed_mps) > stop_speed_mps) ||
            (fabsf(yaw_rate_rads) > patrol_yaw_rate_stopped_rads)) {
            g2.motors.set_throttle(0.0f);
            g2.motors.set_steering(0.0f);
            return;
        }

        _spin_heading_cd =
            current_loc.get_bearing_to(_active_destination);
        if (!isfinite(_spin_heading_cd) ||
            !g2.wp_nav.start_planned_pivot(_spin_heading_cd, 0)) {
            hold("Patrol: align controller start failed, hold",
                 FaultReason::Internal,
                 true);
            return;
        }
        attitude_control.get_steering_rate_pid().reset_I();
        _spin_pivot_started = true;
    }

    float turn_rate_rads = 0.0f;
    const AR_WPNav::PlannedPivotResult result =
        g2.wp_nav.update_planned_pivot(turn_rate_rads);
    g2.motors.set_throttle(0.0f);
    if (result == AR_WPNav::PlannedPivotResult::Running) {
        calc_steering_from_turn_rate(turn_rate_rads, false);
        return;
    }
    if (result == AR_WPNav::PlannedPivotResult::Fault) {
        g2.motors.set_steering(0.0f);
        hold("Patrol: spin controller fault, hold",
             FaultReason::Internal,
             true);
        return;
    }

    g2.motors.set_steering(0.0f);
    // Likewise, do not carry pivot integral into the next straight leg.
    attitude_control.get_steering_rate_pid().reset_I();
    write_patrol_log(LogEvent::SpinDone, true);
    if (_spin_purpose == SpinPurpose::AlignDrive) {
        const float initial_distance_m =
            current_loc.get_distance(_active_destination);
        if (!isfinite(initial_distance_m)) {
            hold("Patrol: aligned position invalid, hold",
                 FaultReason::Estimator,
                 false);
            return;
        }
        if (initial_distance_m < 0.01f) {
            write_patrol_log(LogEvent::EndpointReached, true);
            if (!begin_spin()) {
                hold("Patrol: aligned endpoint turn invalid, hold",
                     FaultReason::RouteGeometry,
                     true);
            }
            return;
        }

        // InitialTransit is the only non-A/B-derived leg. Install it after
        // alignment so Spin displacement cannot leave the new Drive offset
        // from a path frozen before the vehicle stopped.
        _active_origin = current_loc;
        if (!install_saved_drive()) {
            hold("Patrol: aligned drive install failed, hold",
                 FaultReason::Internal,
                 true);
            return;
        }
        start_installed_drive();
        // Start the aligned InitialTransit on the same scheduler cycle as the
        // planned pivot completion edge.
        update_drive();
        return;
    }
    if (!finish_spin_and_start_drive()) {
        hold("Patrol: failed to start next drive, hold",
             FaultReason::RouteGeometry,
             true);
    }
}

void ModePatrol::start_installed_drive()
{
    reset_drive_boundary_monitors();
    _spin_purpose = SpinPurpose::TurnToNextLeg;
    _spin_pivot_started = false;
    _spin_anchor = Location();
    _motion_state = MotionState::Drive;
    _fault_reason = FaultReason::None;
    _spin_drift_violation_start_ms = 0U;
    gcs().send_text(MAV_SEVERITY_INFO,
                    "Patrol: %s line %u sp %.2f off %.2f",
                    leg_name(_active_target.leg_type),
                    (unsigned)_active_target.line_index,
                    (double)_active_spacing_m,
                    (double)_active_offset_m);
    write_patrol_log(LogEvent::DriveStart, true);
}

void ModePatrol::reset_drive_boundary_monitors()
{
    _terminal_miss_violation_start_ms = 0U;
    _xtrack_violation_start_ms = 0U;
    _xtrack_violation_start_m = 0.0f;
    _xtrack_steering_limited = false;
}

bool ModePatrol::finish_spin_and_start_drive()
{
    if (!advance_to_next_target()) {
        return false;
    }
    // set_target installed the next stopping line.  Run it immediately so the
    // Spin completion edge has no empty zero-output scheduler cycle.
    update_drive();
    return _motion_state != MotionState::Hold;
}

bool ModePatrol::start_patrol()
{
    const float spacing_m = _dist.get();
    if (!spacing_is_valid(spacing_m)) {
        set_fault(spacing_error_message(spacing_m),
                  FaultReason::Configuration);
        return false;
    }
    if (!_point_a.initialised() || !_point_b.initialised() ||
        (_point_a.get_distance(_point_b) < patrol_ab_length_min) ||
        !rover.current_loc.initialised()) {
        set_fault("Patrol: invalid A/B line", FaultReason::RouteGeometry);
        return false;
    }

    _queued_spacing_m = spacing_m;
    _last_parameter_spacing_m = spacing_m;
    _next_valid = false;
    _fault_reason = FaultReason::None;
    _fault_latched = false;
    _recapture_rejection_reported = false;

    g2.wp_nav.init();
    g2.wp_nav.set_turn_params(g2.turn_radius, g2.motors.have_skid_steering());
    Mode::set_reversed(false);

    const ModePatrolRoute::Target target = ModePatrolRoute::start_target();
    Location line_start;
    Location line_end;
    if (!make_offset_leg(target.line_index,
                         spacing_m,
                         line_start,
                         line_end)) {
        set_fault("Patrol: invalid A/B line", FaultReason::RouteGeometry);
        return false;
    }

    TargetGeometry geometry;
    geometry.destination = line_start;
    geometry.line_start = line_start;
    geometry.line_end = line_end;
    geometry.offset_m = spacing_m;
    geometry.spacing_m = spacing_m;

    const bool needs_initial_alignment =
        rover.current_loc.get_distance(line_start) >= 0.01f;
    if (!set_target(target,
                    rover.current_loc,
                    geometry,
                    needs_initial_alignment)) {
        set_fault("Patrol: failed to set start", FaultReason::Internal);
        return false;
    }

    if (needs_initial_alignment && !begin_drive_alignment()) {
        set_fault("Patrol: failed to align start",
                  FaultReason::RouteGeometry);
        return false;
    }

    write_patrol_log(LogEvent::SpacingApplied,
                     true,
                     &target,
                     target.line_index,
                     spacing_m,
                     spacing_m);
    gcs().send_text(MAV_SEVERITY_INFO,
                    "Patrol: spacing %.2fm applied line %u",
                    (double)spacing_m,
                    (unsigned)target.line_index);
    return true;
}

bool ModePatrol::resume_patrol()
{
    if (!_next_valid ||
        !_active_origin.initialised() ||
        !_active_destination.initialised() ||
        !_next_destination.initialised() ||
        (_active_target.line_index == 0U)) {
        set_fault("Patrol: invalid resume state", FaultReason::Internal);
        return false;
    }

    Location current_loc;
    Vector3f velocity_ned;
    float forward_speed;
    const float yaw_rate_rads = AP::ahrs().get_yaw_rate_earth();
    if (!AP::ahrs().get_location(current_loc) ||
        !AP::ahrs().get_velocity_NED(velocity_ned) ||
        !attitude_control.get_forward_speed(forward_speed) ||
        !isfinite(yaw_rate_rads)) {
        hold("Patrol: resume estimator unavailable, hold",
             FaultReason::Estimator,
             false);
        return true;
    }

    const Vector2f track = _active_origin.get_distance_NE(_active_destination);
    const float track_length_m = track.length();
    if (!isfinite(track_length_m) || (track_length_m < 0.01f)) {
        set_fault("Patrol: resume route geometry invalid",
                  FaultReason::RouteGeometry);
        return false;
    }
    const Vector2f track_unit = track / track_length_m;
    const Vector2f vehicle_from_origin = _active_origin.get_distance_NE(current_loc);
    const float along_m = vehicle_from_origin.dot(track_unit);
    const float xtrack_m = fabsf(vehicle_from_origin % track_unit);
    const float projection_tolerance_m = 0.01f;
    if (!isfinite(along_m) || !isfinite(xtrack_m)) {
        hold("Patrol: resume estimator fault, hold",
             FaultReason::Estimator,
             false);
        return true;
    }

    const float stop_speed_mps = MAX(attitude_control.get_stop_speed(),
                                     patrol_path_speed_stopped_mps);
    const float track_speed_mps = velocity_ned.xy().dot(track_unit);
    const bool stopped_at_endpoint =
        (current_loc.get_distance(_active_destination) <= g2.wp_nav.get_pivot_radius()) &&
        (fabsf(forward_speed) <= stop_speed_mps) &&
        isfinite(track_speed_mps) &&
        (fabsf(track_speed_mps) <= stop_speed_mps) &&
        (fabsf(yaw_rate_rads) <= patrol_yaw_rate_stopped_rads);
    if (stopped_at_endpoint) {
        if (!begin_spin()) {
            set_fault("Patrol: resume turn geometry invalid",
                      FaultReason::RouteGeometry);
            return false;
        }
        gcs().send_text(MAV_SEVERITY_INFO,
                        "Patrol: resume Spin line %u",
                        (unsigned)_active_target.line_index);
        write_patrol_log(LogEvent::Resume, true);
        return true;
    }

    const float endpoint_radius_m = g2.wp_nav.get_pivot_radius();
    const float recovery_limit_m =
        MAX(endpoint_radius_m, g2.wp_nav.get_pivot_drift());
    const float distance_to_endpoint_m =
        current_loc.get_distance(_active_destination);
    if (!isfinite(endpoint_radius_m) || !isfinite(recovery_limit_m) ||
        !isfinite(distance_to_endpoint_m)) {
        hold("Patrol: resume boundary estimator fault, hold",
             FaultReason::Estimator,
             false);
        return true;
    }
    if ((distance_to_endpoint_m > endpoint_radius_m) &&
        (along_m >= track_length_m)) {
        hold("Patrol: past endpoint, hold",
             FaultReason::PastFinishPlane,
             false);
        return true;
    }
    if (xtrack_m > recovery_limit_m) {
        hold("Patrol: cross-track diverging, hold",
             FaultReason::CrossTrack,
             false);
        return true;
    }

    if (_active_target.leg_type ==
        ModePatrolRoute::LegType::InitialTransit) {
        // InitialTransit is not an A/B-derived work line. Re-anchor it at the
        // current body position and perform the same one-shot entry alignment
        // used by a fresh Patrol start.
        _active_origin = current_loc;
        if (!begin_drive_alignment()) {
            set_fault("Patrol: resume start alignment failed",
                      FaultReason::RouteGeometry);
            return false;
        }
        gcs().send_text(MAV_SEVERITY_INFO,
                        "Patrol: resume %s line %u off %.2fm",
                        leg_name(_active_target.leg_type),
                        (unsigned)_active_target.line_index,
                        (double)_active_offset_m);
        write_patrol_log(LogEvent::Resume, true);
        return true;
    }

    if ((along_m < -projection_tolerance_m) ||
        (along_m > (track_length_m + projection_tolerance_m))) {
        hold("Patrol: resume outside route segment, hold",
             FaultReason::PastFinishPlane,
             false);
        return true;
    }
    if (!install_saved_drive()) {
        set_fault("Patrol: resume drive install failed",
                  FaultReason::Internal);
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO,
                    "Patrol: resume %s line %u off %.2fm",
                    leg_name(_active_target.leg_type),
                    (unsigned)_active_target.line_index,
                    (double)_active_offset_m);
    write_patrol_log(LogEvent::Resume, true);
    return true;
}

bool ModePatrol::install_saved_drive()
{
    if (!g2.wp_nav.set_desired_location_stopping_from_origin(
            _active_origin,
            _active_destination,
            get_leg_speed_cap(_active_target.leg_type))) {
        return false;
    }
    _motion_state = MotionState::Drive;
    _fault_reason = FaultReason::None;
    reset_drive_boundary_monitors();
    _distance_to_destination = g2.wp_nav.get_distance_to_destination();
    return true;
}

bool ModePatrol::make_offset_leg(uint16_t line_index,
                                 float offset_m,
                                 Location &start,
                                 Location &end) const
{
    if ((line_index == 0U) || !isfinite(offset_m) || !is_positive(offset_m)) {
        return false;
    }

    const Vector2f ab = _point_a.get_distance_NE(_point_b);
    const float ab_len = ab.length();
    if (!isfinite(ab_len) || (ab_len < patrol_ab_length_min)) {
        return false;
    }

    const Vector2f normal{-ab.y / ab_len, ab.x / ab_len};
    const Vector2f offset = normal * offset_m;

    // Generate both endpoints in point A's single local NE frame. Offsetting
    // A and B independently uses two geodetic tangent points and can rotate
    // successive lines slightly. With one anchor every WorkLine vector is the
    // original AB vector (or its reverse) and spacing is a pure normal scalar.
    Location offset_a = _point_a;
    Location offset_b = _point_a;
    offset_a.offset(offset.x, offset.y);
    offset_b.offset(ab.x + offset.x, ab.y + offset.y);

    const bool start_from_b = (line_index % 2U) == 1U;
    start = start_from_b ? offset_b : offset_a;
    end = start_from_b ? offset_a : offset_b;
    return start.initialised() && end.initialised();
}

bool ModePatrol::set_target(const ModePatrolRoute::Target &target,
                            const Location &origin,
                            const TargetGeometry &geometry,
                            bool defer_drive_start)
{
    Mode::set_reversed(false);
    g2.wp_nav.set_turn_params(g2.turn_radius, g2.motors.have_skid_steering());

    ModePatrolRoute::Target next_target{};
    TargetGeometry next_geometry;
    bool line_generated = false;
    if (!get_next_distinct_target(target,
                                  geometry.destination,
                                  geometry,
                                  next_target,
                                  next_geometry,
                                  line_generated)) {
        return false;
    }

    const bool zero_length_initial =
        (target.leg_type == ModePatrolRoute::LegType::InitialTransit) &&
        (origin.get_distance(geometry.destination) < 0.01f);
    if (zero_length_initial) {
        float forward_speed;
        const float yaw_rate_rads = AP::ahrs().get_yaw_rate_earth();
        const float stop_speed_mps = MAX(attitude_control.get_stop_speed(),
                                         patrol_path_speed_stopped_mps);
        if (!attitude_control.get_forward_speed(forward_speed) ||
            !isfinite(yaw_rate_rads) ||
            (fabsf(forward_speed) > stop_speed_mps) ||
            (fabsf(yaw_rate_rads) > patrol_yaw_rate_stopped_rads)) {
            return false;
        }
    } else if (!defer_drive_start &&
               !g2.wp_nav.set_desired_location_stopping_from_origin(
                   origin,
                   geometry.destination,
                   get_leg_speed_cap(target.leg_type))) {
        return false;
    }

    _distance_to_destination = (zero_length_initial || defer_drive_start) ?
        origin.get_distance(geometry.destination) :
        g2.wp_nav.get_distance_to_destination();

    // Commit after WPNav accepted the current stopping line, or after the
    // caller explicitly deferred InitialTransit installation until alignment.
    // The next target is geometry for Spin, not a WPNav path cache.
    _active_target = target;
    _active_origin = origin;
    _active_destination = geometry.destination;
    _active_offset_m = geometry.offset_m;
    _active_spacing_m = geometry.spacing_m;
    _next_target = next_target;
    _next_destination = next_geometry.destination;
    _next_line_start = next_geometry.line_start;
    _next_line_end = next_geometry.line_end;
    _next_offset_m = next_geometry.offset_m;
    _next_spacing_m = next_geometry.spacing_m;
    _next_valid = true;
    _fault_reason = FaultReason::None;
    _fault_latched = false;
    reset_drive_boundary_monitors();
    _spin_drift_violation_start_ms = 0U;
    _spin_pivot_started = false;

    if (!defer_drive_start) {
        start_installed_drive();
    }

    if (zero_length_initial) {
        write_patrol_log(LogEvent::EndpointReached, true);
        if (!begin_spin()) {
            return false;
        }
    }

    if (line_generated) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "Patrol: spacing %.2fm applied line %u",
                        (double)next_geometry.spacing_m,
                        (unsigned)next_target.line_index);
        write_patrol_log(LogEvent::SpacingApplied,
                         true,
                         &next_target,
                         next_target.line_index,
                         next_geometry.spacing_m,
                         next_geometry.offset_m);
    }
    return true;
}

bool ModePatrol::get_target_geometry(
    const ModePatrolRoute::Target &target,
    const TargetGeometry &current_geometry,
    TargetGeometry &geometry)
{
    geometry = current_geometry;

    if (target.starts_new_line()) {
        if (!spacing_is_valid(_queued_spacing_m)) {
            return false;
        }
        geometry.spacing_m = _queued_spacing_m;
        geometry.offset_m = current_geometry.offset_m + geometry.spacing_m;
        if (!isfinite(geometry.offset_m) ||
            !make_offset_leg(target.line_index,
                             geometry.offset_m,
                             geometry.line_start,
                             geometry.line_end)) {
            return false;
        }
    }

    switch (target.leg_type) {
    case ModePatrolRoute::LegType::InitialTransit:
    case ModePatrolRoute::LegType::Transition:
        geometry.destination = geometry.line_start;
        break;
    case ModePatrolRoute::LegType::WorkLine:
        geometry.destination = geometry.line_end;
        break;
    case ModePatrolRoute::LegType::None:
        return false;
    }
    return geometry.destination.initialised();
}

bool ModePatrol::get_next_distinct_target(
    const ModePatrolRoute::Target &current_target,
    const Location &current_destination,
    const TargetGeometry &current_geometry,
    ModePatrolRoute::Target &next_target,
    TargetGeometry &next_geometry,
    bool &line_generated)
{
    ModePatrolRoute::Target candidate = current_target;
    TargetGeometry candidate_geometry = current_geometry;
    line_generated = false;

    ModePatrolRoute::Target following{};
    while (ModePatrolRoute::target_after(candidate, following)) {
        candidate = following;
        if (!get_target_geometry(candidate,
                                 candidate_geometry,
                                 next_geometry)) {
            return false;
        }

        line_generated |= candidate.starts_new_line();
        candidate_geometry = next_geometry;

        // With nonzero spacing this normally returns on the first candidate.
        // Keep the centimetre-quantisation guard without giving zero spacing a
        // hidden single-line semantic.
        if (!current_destination.same_latlon_as(next_geometry.destination)) {
            next_target = candidate;
            return true;
        }
    }
    return false;
}

bool ModePatrol::advance_to_next_target()
{
    if (!_next_valid) {
        set_fault("Patrol: missing next route target",
                  FaultReason::Internal);
        return false;
    }

    const ModePatrolRoute::Target target = _next_target;
    const Location origin = _active_destination;
    TargetGeometry geometry;
    geometry.destination = _next_destination;
    geometry.line_start = _next_line_start;
    geometry.line_end = _next_line_end;
    geometry.offset_m = _next_offset_m;
    geometry.spacing_m = _next_spacing_m;

    if (!set_target(target,
                    origin,
                    geometry)) {
        set_fault(target.starts_new_line() ?
                      "Patrol: failed to set transition" :
                      "Patrol: failed to set work line",
                  FaultReason::RouteGeometry);
        return false;
    }
    return true;
}

float ModePatrol::get_leg_speed_cap(ModePatrolRoute::LegType leg) const
{
    switch (leg) {
    case ModePatrolRoute::LegType::InitialTransit:
    case ModePatrolRoute::LegType::Transition:
        return constrain_float(_transition_speed.get(),
                               patrol_transition_speed_min,
                               patrol_transition_speed_max);
    case ModePatrolRoute::LegType::WorkLine:
    case ModePatrolRoute::LegType::None:
        return 0.0f;
    }
    return 0.0f;
}

bool ModePatrol::spacing_is_valid(float spacing_m) const
{
    return isfinite(spacing_m) &&
           (spacing_m >= patrol_dist_min) &&
           (spacing_m <= patrol_dist_max);
}

bool ModePatrol::run_limit_reached() const
{
    const int16_t max_runs = _max_runs.get();
    return (max_runs > 0) &&
           (_active_target.leg_type == ModePatrolRoute::LegType::WorkLine) &&
           (_active_target.line_index >= static_cast<uint16_t>(max_runs));
}

bool ModePatrol::refresh_spacing_queue(bool report_fault)
{
    const float configured_spacing_m = _dist.get();
    if (isfinite(_last_parameter_spacing_m) &&
        is_equal(configured_spacing_m, _last_parameter_spacing_m)) {
        return spacing_is_valid(configured_spacing_m);
    }
    _last_parameter_spacing_m = configured_spacing_m;

    if (!spacing_is_valid(configured_spacing_m)) {
        if (report_fault && (_point_count >= 2)) {
            set_fault(spacing_error_message(configured_spacing_m),
                      FaultReason::Configuration);
        }
        return false;
    }

    _queued_spacing_m = configured_spacing_m;
    const uint16_t applies_line = first_unplanned_line();
    if (_active_target.line_index != 0U) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "Patrol: queued %.2fm for line %u",
                        (double)configured_spacing_m,
                        (unsigned)applies_line);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "Patrol: spacing %.2fm",
                        (double)configured_spacing_m);
    }
    write_patrol_log(LogEvent::SpacingQueued,
                     true,
                     nullptr,
                     applies_line,
                     configured_spacing_m,
                     NAN);
    return true;
}

uint16_t ModePatrol::first_unplanned_line() const
{
    uint16_t last_planned_line = _active_target.line_index;
    if (_next_valid) {
        last_planned_line = MAX(last_planned_line,
                                _next_target.line_index);
    }
    return last_planned_line == UINT16_MAX ?
        UINT16_MAX : static_cast<uint16_t>(last_planned_line + 1U);
}

void ModePatrol::adjust_spacing(int8_t direction)
{
    if (direction == 0) {
        return;
    }

    const float configured_spacing_m = _dist.get();
    const float base_spacing_m = spacing_is_valid(configured_spacing_m) ?
        configured_spacing_m : patrol_dist_min;
    const float new_spacing_m = constrain_float(
        base_spacing_m + (patrol_dist_step * (direction > 0 ? 1.0f : -1.0f)),
        patrol_dist_min,
        patrol_dist_max);
    _dist.set_and_save_ifchanged(new_spacing_m);
    refresh_spacing_queue(false);
}

void ModePatrol::hold(const char *message,
                      FaultReason reason,
                      bool latched)
{
    const bool new_hold = (_motion_state != MotionState::Hold) ||
                          (_fault_reason != reason) ||
                          (latched && !_fault_latched);
    _motion_state = MotionState::Hold;
    _fault_reason = reason;
    _fault_latched |= latched;
    reset_drive_boundary_monitors();
    g2.wp_nav.cancel_planned_pivot();
    _spin_pivot_started = false;
    g2.motors.set_throttle(0.0f);
    g2.motors.set_steering(0.0f);
    if (!new_hold) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "%s", message);
    write_patrol_log(LogEvent::Hold, true);
}

void ModePatrol::set_fault(const char *message, FaultReason reason)
{
    hold(message, reason, true);
}

const char *ModePatrol::leg_name(ModePatrolRoute::LegType leg)
{
    switch (leg) {
    case ModePatrolRoute::LegType::InitialTransit:
        return "Initial";
    case ModePatrolRoute::LegType::WorkLine:
        return "Work";
    case ModePatrolRoute::LegType::Transition:
        return "Transition";
    case ModePatrolRoute::LegType::None:
        return "None";
    }
    return "Unknown";
}

void ModePatrol::write_patrol_log(LogEvent event,
                                  bool critical,
                                  const ModePatrolRoute::Target *target_override,
                                  uint16_t line_override,
                                  float spacing_override_m,
                                  float offset_override_m,
                                  bool write_geometry)
{
#if HAL_LOGGING_ENABLED
    if (!rover.should_log(MASK_LOG_NTUN)) {
        return;
    }

    LogSnapshot snapshot;
    snapshot.event = static_cast<uint8_t>(event);
    snapshot.state = static_cast<uint8_t>(_motion_state);
    snapshot.leg = static_cast<uint8_t>(_active_target.leg_type);
    snapshot.fault = static_cast<uint8_t>(_fault_reason);
    snapshot.line = _active_target.line_index;
    snapshot.geometry_line = _active_target.line_index;
    const bool patrol_active =
        _motion_state != MotionState::WaitingForPoints;
    snapshot.spacing_m = patrol_active ? _active_spacing_m : _queued_spacing_m;
    snapshot.offset_m = _active_offset_m;

    if (target_override != nullptr) {
        snapshot.leg = static_cast<uint8_t>(target_override->leg_type);
        snapshot.line = target_override->line_index;
    }
    if (line_override != 0U) {
        snapshot.line = line_override;
    }
    if (isfinite(spacing_override_m)) {
        snapshot.spacing_m = spacing_override_m;
    }
    if (isfinite(offset_override_m)) {
        snapshot.offset_m = offset_override_m;
    }

    if (patrol_active) {
        snapshot.xtrack_m = AP::logger().quiet_nanf();
        snapshot.distance_m = AP::logger().quiet_nanf();

        // Event records may refer to a queued or newly generated line while
        // PTRG continues to describe the active theoretical geometry. Only
        // attach live observations when PTRL refers to that active line.
        Location current_loc;
        if ((snapshot.line == snapshot.geometry_line) &&
            _active_origin.initialised() &&
            _active_destination.initialised() &&
            AP::ahrs().get_location(current_loc)) {
            const Vector2f track =
                _active_origin.get_distance_NE(_active_destination);
            const Vector2f vehicle_from_origin =
                _active_origin.get_distance_NE(current_loc);
            const float track_length_m = track.length();
            const float distance_m =
                current_loc.get_distance(_active_destination);
            if (isfinite(track_length_m) &&
                is_positive(track_length_m) &&
                isfinite(vehicle_from_origin.x) &&
                isfinite(vehicle_from_origin.y) &&
                isfinite(distance_m)) {
                snapshot.xtrack_m =
                    vehicle_from_origin % (track / track_length_m);
                snapshot.distance_m = distance_m;
                snapshot.flags |= LogFlagObservationValid;
            }
        }
    }

    if (_point_count >= 1) {
        snapshot.flags |= LogFlagPointAValid;
    }
    if (_point_count >= 2) {
        snapshot.flags |= LogFlagPointBValid;
    }
    if (_next_valid) {
        snapshot.flags |= LogFlagNextValid;
    }
    if (patrol_active) {
        snapshot.flags |= LogFlagMotionInputBlocked;
    }
    if ((_motion_state == MotionState::Drive) &&
        g2.wp_nav.path_terminal() &&
        (snapshot.distance_m > g2.wp_nav.get_pivot_radius())) {
        snapshot.flags |= LogFlagTerminalGuidance;
    }
    if (patrol_active &&
        !is_equal(_queued_spacing_m, _active_spacing_m)) {
        snapshot.flags |= LogFlagSpacingQueued;
    }

    snapshot.point_a = _point_a;
    snapshot.point_b = _point_b;
    snapshot.origin = _active_origin;
    snapshot.destination = _active_destination;
    snapshot.next_destination = _next_destination;
    rover.Log_Write_Patrol(snapshot, critical, write_geometry);
    const uint32_t now_ms = AP_HAL::millis();
    if (critical) {
        _last_heartbeat_ms = now_ms;
    }
    if (write_geometry) {
        _last_geometry_log_ms = now_ms;
    }
#else
    (void)event;
    (void)critical;
    (void)target_override;
    (void)line_override;
    (void)spacing_override_m;
    (void)offset_override_m;
    (void)write_geometry;
#endif
}

void ModePatrol::write_patrol_heartbeat()
{
    if (_motion_state == MotionState::WaitingForPoints) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_heartbeat_ms) < patrol_dynamic_log_period_ms) {
        return;
    }
    const bool write_geometry =
        (now_ms - _last_geometry_log_ms) >= patrol_geometry_log_period_ms;
    _last_heartbeat_ms = now_ms;
    write_patrol_log(LogEvent::Heartbeat,
                     false,
                     nullptr,
                     0U,
                     NAN,
                     NAN,
                     write_geometry);
}

float ModePatrol::wp_bearing() const
{
    return _motion_state == MotionState::Drive ?
        g2.wp_nav.wp_bearing_cd() * 0.01f : 0.0f;
}

float ModePatrol::nav_bearing() const
{
    if (_motion_state == MotionState::Spin) {
        return _spin_heading_cd * 0.01f;
    }
    return _motion_state == MotionState::Drive ?
        g2.wp_nav.nav_bearing_cd() * 0.01f : 0.0f;
}

float ModePatrol::crosstrack_error() const
{
    return _motion_state == MotionState::Drive ?
        g2.wp_nav.crosstrack_error() : 0.0f;
}

float ModePatrol::get_desired_lat_accel() const
{
    return _motion_state == MotionState::Drive ?
        g2.wp_nav.get_lat_accel() : 0.0f;
}

bool ModePatrol::get_desired_location(Location &destination) const
{
    if (((_motion_state != MotionState::Drive) &&
         (_motion_state != MotionState::Spin)) ||
        !g2.wp_nav.is_destination_valid()) {
        return false;
    }
    destination = g2.wp_nav.get_oa_destination();
    return true;
}

bool ModePatrol::set_desired_speed(float speed)
{
    return g2.wp_nav.set_speed_max(speed);
}
