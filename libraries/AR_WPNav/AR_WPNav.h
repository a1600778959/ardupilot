#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/SCurve.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <APM_Control/AR_PosControl.h>
#include <AC_Avoidance/AP_OAPathPlanner.h>
#include "AR_PivotTurn.h"
#include "AR_WPNav_Differential.h"

class AR_WPNav {
public:

    enum class MotionPrimitive : uint8_t {
        Path,
        Spin,
        Hold,
    };

    enum class ExactPivotFault : uint8_t {
        None,
        Estimator,
        Timeout,
        Internal,
        RecoveryInfeasible,
        RecoveryBounds,
    };

    // Stable numeric phase values exported to XPNV.Ph and STATUSTEXT.  Keep
    // this independent from the private controller enum so log decoding does
    // not depend on implementation ordering.
    enum class ExactPivotDiagPhase : uint8_t {
        None,
        Move,
        CapturePath,
        Spin,
        PromotedMove,
        Fault,
    };

    // Sticky diagnostic events.  Vehicle code acknowledges a snapshot by its
    // sequence number after it has written the corresponding log records and
    // STATUSTEXT messages.  Control code never consumes these flags.
    enum ExactPivotDiagTransition : uint16_t {
        DiagNewExact      = 1U << 0,
        DiagEarlyCaptureA = 1U << 1,
        DiagCapturePathB  = 1U << 2,
        DiagEnterSpin     = 1U << 3,
        DiagPromotePath   = 1U << 4,
        DiagAckOrdinary   = 1U << 5,
        DiagAckExact      = 1U << 6,
        DiagFault         = 1U << 7,
        DiagOACancel      = 1U << 8,
        DiagHandoffDone   = 1U << 9,
    };

    // Live state bits recorded in XPNV.Flg.
    enum ExactPivotDiagState : uint16_t {
        DiagStateExactActive      = 1U << 0,
        DiagStateEndpointValid    = 1U << 1,
        DiagStatePlanValid        = 1U << 2,
        DiagStatePathTerminal     = 1U << 3,
        DiagStateCompletion       = 1U << 4,
        DiagStatePivotActive      = 1U << 5,
        DiagStateCaptureActive    = 1U << 6,
        DiagStateCaptureArc       = 1U << 7,
        DiagStateRejoinActive     = 1U << 8,
        DiagStateHandoffActive    = 1U << 9,
        DiagStateTimeoutActive    = 1U << 10,
        DiagStatePositionLoss     = 1U << 11,
        DiagStateDriftActive      = 1U << 12,
        DiagStateCaptureViolation = 1U << 13,
        DiagStateReversed         = 1U << 14,
        DiagStateYawErrorValid    = 1U << 15,
    };

    enum class ExactPivotOACancelReason : uint8_t {
        None,
        Processing,
        Error,
        InvalidPlanner,
        Dijkstra,
        BendyRuler,
    };

    struct ExactPivotCaptureDiag {
        bool valid{false};
        uint8_t segment{0U};       // 0=None, 1=Arc, 2=Line
        int8_t direction{0};
        float progress_m{0.0f};
        float length_m{0.0f};
        float radius_m{0.0f};
        float target_speed_mps{0.0f};
        float tracking_error_m{0.0f};
        float heading_error_deg{0.0f};
        float endpoint_distance_m{0.0f};
    };

    struct ExactPivotHandoffDiag {
        bool valid{false};
        float along_m{0.0f};
        float rejoin_m{0.0f};
        float blend_m{0.0f};
        float distance_ratio{0.0f};
        float heading_ratio{0.0f};
        float path_weight{0.0f};
        float heading_error_deg{0.0f};
        float heading_rate_rads{0.0f};
        float path_rate_rads{0.0f};
        float output_rate_rads{0.0f};
    };

    struct ExactPivotDiagSnapshot {
        uint8_t phase{0U};
        uint8_t primitive{0U};
        uint8_t fault{0U};
        uint8_t fault_from_phase{0U};
        uint16_t transition_flags{0U};
        uint16_t state_flags{0U};
        uint32_t transition_sequence{0U};
        uint32_t leg_id{0U};
        uint32_t handoff_generation{0U};
        uint32_t acknowledged_generation{0U};
        float endpoint_distance_m{0.0f};
        float planned_distance_m{0.0f};
        float planned_speed_mps{0.0f};
        float desired_speed_mps{0.0f};
        float desired_turn_rate_rads{0.0f};
        float yaw_error_deg{0.0f};
        float xtrack_error_m{0.0f};
        float target_heading_deg{0.0f};
        uint8_t spin_source{0U};   // 'A' for early capture, 'B' for CapturePath
        int8_t turn_direction{0};
        uint8_t oa_cancel_from_phase{0U};
        uint8_t oa_cancel_reason{0U};
        ExactPivotCaptureDiag capture;
        ExactPivotHandoffDiag handoff;
    };

    // constructor
    AR_WPNav(AR_AttitudeControl& atc, AR_PosControl &pos_control);

    // initialise waypoint controller.  speed_max should be set to the maximum speed in m/s (or left at zero to use the default speed)
    void init(float speed_max = 0);

    // update navigation
    virtual void update(float dt);

    // get or set maximum speed in m/s
    // if set_speed_max is called in rapid succession changes in speed may be delayed by up to 0.5sec
    float get_speed_max() const { return _base_speed_max; }
    bool set_speed_max(float speed_max);

    // set speed nudge in m/s.  this will have no effect unless nudge_speed_max > speed_max
    // nudge_speed_max should always be positive regardless of whether the vehicle is travelling forward or reversing
    void set_nudge_speed_max(float nudge_speed_max);

    // execute the mission in reverse (i.e. drive backwards to destination)
    bool get_reversed() const { return _reversed; }
    void set_reversed(bool reversed);

    // get navigation outputs for speed (in m/s) and turn rate (in rad/sec)
    float get_speed() const { return _desired_speed_limited; }
    float get_turn_rate_rads() const { return _desired_turn_rate_rads; }

    // get desired lateral acceleration (for reporting purposes only because will be zero during pivot turns)
    float get_lat_accel() const { return _desired_lat_accel; }

    // set desired location and (optionally) next_destination
    // next_destination should be provided if known to allow smooth cornering
    virtual bool set_desired_location(const Location &destination, Location next_destination = Location()) WARN_IF_UNUSED;

    // set a stopping S-curve followed by a planned in-place turn toward next_destination
    virtual bool set_desired_location_exact_pivot(const Location &destination, const Location &next_destination) WARN_IF_UNUSED;

    // return true if the corner at destination would normally require a pivot
    bool would_pivot_at_destination(const Location &destination, const Location &next_destination) const;

    // set desired location to a reasonable stopping point, return true on success
    bool set_desired_location_to_stopping_location()  WARN_IF_UNUSED;

    // set desired location as offset from the EKF origin, return true on success
    bool set_desired_location_NED(const Vector3f& destination) WARN_IF_UNUSED;
    bool set_desired_location_NED(const Vector3f &destination, const Vector3f &next_destination) WARN_IF_UNUSED;

    // set desired location but expect the destination to be updated again in the near future
    // position controller input shaping will be used for navigation instead of scurves
    // Note: object avoidance is not supported if this method is used
    bool set_desired_location_expect_fast_update(const Location &destination) WARN_IF_UNUSED;

    // true if vehicle has reached desired location. defaults to true because this is normally used by missions and we do not want the mission to become stuck
    virtual bool reached_destination() const
    {
        if (_exact_phase == ExactPivotPhase::Fault) {
            return false;
        }
        return _completion_event_pending || _reached_destination;
    }

    // return distance (in meters) to destination
    float get_distance_to_destination() const { return _distance_to_destination; }

    // return true if destination is valid
    bool is_destination_valid() const { return _orig_and_dest_valid; }

    // get current destination. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    const Location &get_destination() const { return _destination; }

    // return heading (in centi-degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing_cd() const { return _wp_bearing_cd; }
    float nav_bearing_cd() const { return _desired_heading_cd; }
    float crosstrack_error() const { return _cross_track_error; }

    // get object avoidance adjusted origin. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    virtual const Location &get_oa_origin() const { return _origin; }

    // get object avoidance adjusted destination. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    virtual const Location &get_oa_destination() const { return get_destination(); }

    // return the heading (in centi-degrees) to the next waypoint accounting for OA, (used by sailboats)
    virtual float oa_wp_bearing_cd() const { return wp_bearing_cd(); }

    // settor to allow vehicle code to provide turn related param values to this library (should be updated regularly)
    void set_turn_params(float turn_radius, bool pivot_possible);

    // accessors for parameter values
    float get_default_speed() const { return _speed_max; }
    float get_default_accel() const { return _accel_max; }
    float get_default_jerk() const { return _jerk_max; }
    float get_radius() const { return _radius; }
    float get_pivot_rate() const { return _pivot.get_rate_max(); }

    // read-only pivot state for vehicle-specific safety monitoring
    bool is_pivot_active() const { return _pivot.active(); }
    bool is_exact_pivot_active() const { return _exact_phase == ExactPivotPhase::Spin; }
    bool is_exact_pivot_turn_in_progress() const
    {
        return (_exact_phase == ExactPivotPhase::Spin) ||
               (_exact_phase == ExactPivotPhase::CapturePath);
    }
    bool is_exact_pivot_sequence_active() const
    {
        return (_exact_phase != ExactPivotPhase::None) || _completion_event_pending;
    }
    bool exact_pivot_failed() const { return _exact_phase == ExactPivotPhase::Fault; }
    ExactPivotFault get_exact_pivot_fault() const { return _exact_fault; }
    MotionPrimitive get_motion_primitive() const;
    float get_pivot_heading_error_deg() const;

    // Return the current read-only diagnostic snapshot. Transition flags are a
    // cumulative sticky batch since the previous ACK; accompanying values are
    // the latest controller/cache state, not a per-event journal. Flags are
    // cleared only by an ACK for the exact sequence returned here.
    ExactPivotDiagSnapshot get_exact_pivot_diag_snapshot() const;
    void ack_exact_pivot_diag(uint32_t transition_sequence);

    // calculate stopping location using current position and attitude controller provided maximum deceleration
    // returns true on success, false on failure
    bool get_stopping_location(Location& stopping_loc) WARN_IF_UNUSED;

    // is_fast_waypoint returns true if vehicle will not stop at destination (e.g. set_desired_location provided a next_destination)
    bool is_fast_waypoint() const { return _fast_waypoint; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // true if update has been called recently
    bool is_active() const;

    // move target location along track from origin to destination using SCurves navigation
    void advance_wp_target_along_track(const Location &current_loc, float dt);

    // update psc input shaping navigation controller
    void update_psc_input_shaping(float dt);

    // update distance and bearing from vehicle's current position to destination
    void update_distance_and_bearing_to_destination();

    // calculate steering and speed to drive along line from origin to destination waypoint
    void update_steering_and_speed(const Location &current_loc, float dt);

    // calculate the crosstrack error (does not rely on L1 controller)
    float calc_crosstrack_error(const Location& current_loc) const;

    // calculate yaw change at next waypoint in degrees
    // returns zero if the angle cannot be calculated because some points are on top of others
    float get_corner_angle(const Location& loc1, const Location& loc2, const Location& loc3) const;

    // helper function to initialise position controller if it hasn't been called recently
    // this should be called before updating the position controller with new targets but after the EKF has a good position estimate
    void init_pos_control_if_necessary();

    // set origin and destination to stopping point
    bool set_origin_and_destination_to_stopping_point();

    // check for changes in _base_speed_max or _nudge_speed_max
    // updates position controller limits and recalculate scurve path if required
    void update_speed_max();

    // return the pivot target heading, applying reverse at time of use
    float get_pivot_target_heading_cd() const;

    // clear exact-pivot state when another navigation contract is selected
    void clear_exact_pivot();

    // clear exact-pivot transaction state after an ordinary handoff ACK while
    // preserving the promoted path and its forward-rejoin constraint
    void clear_exact_pivot_after_ack();

    // true when destination acknowledges the currently pending atomic handoff
    bool atomic_handoff_ack_matches(const Location &destination, uint32_t &generation) const;

    // attach an ordinary next-leg preview without modifying the active leg
    bool attach_ordinary_next_leg(const Location &destination, const Location &next_destination) WARN_IF_UNUSED;

    // exact differential-drive transition helpers
    bool exact_endpoint_captured(const Location &current_loc,
                                 float planned_distance_m,
                                 bool path_terminal,
                                 float planned_speed_mps) const;
    float exact_crosstrack_error(const Location &current_loc) const;
    bool exact_spin_position_within_limit(const Location &current_loc) const;
    bool exact_pivot_timed_out(uint32_t now_ms) const;
    void start_exact_pivot_timeout(uint32_t now_ms);
    void enter_exact_spin();
    void enter_exact_fault(ExactPivotFault reason);
    bool start_exact_capture_path(const Location &current_loc) WARN_IF_UNUSED;
    void update_exact_capture_path(const Location &current_loc, float dt);
    void clear_exact_capture_path();
    int8_t select_exact_turn_direction() const;
    bool promote_exact_preview(const Location &current_loc) WARN_IF_UNUSED;
    void apply_forward_rejoin(const Vector2f &origin_ne_m, Vector3f &target_pos);
    void update_path_outputs(float dt);
    void apply_heading_handoff();
    void clear_heading_handoff();
    void clear_promoted_path_constraints();

    // Called only after OA has successfully replaced an Exact navigation
    // contract.  from_phase must be captured before the ordinary setter clears
    // the state machine.
    void note_exact_pivot_oa_cancel(uint8_t from_phase,
                                    ExactPivotOACancelReason reason);

    // build a stopping S-curve between two locations
    bool calculate_stopping_scurve(const Location &origin, const Location &destination, SCurve &scurve) WARN_IF_UNUSED;

    // parameters
    AP_Float _speed_max;            // target speed between waypoints in m/s
    AP_Float _radius;               // distance in meters from a waypoint when we consider the waypoint has been reached
    AR_PivotTurn _pivot;            // pivot turn controller
    AP_Float _accel_max;            // max acceleration.  If zero then attitude controller's specified max accel is used
    AP_Float _jerk_max;             // max jerk (change in acceleration).  If zero then value is same as accel_max
    AP_Float _pivot_radius;          // exact endpoint capture radius before a planned pivot
    AP_Float _pivot_exit;            // heading error at which a planned pivot atomically promotes its next leg
    AP_Float _pivot_drift;           // position and cross-track error tolerated during a planned pivot
    AP_Float _pivot_rejoin;          // forward distance used to rejoin a promoted frozen route
    AP_Float _pivot_timeout;         // total planned CapturePath/Spin timeout
    AP_Float _pivot_capture_speed;   // planned speed below which the S-curve may hand off inside the exact endpoint circle
    AP_Float _pivot_blend;           // distance used to blend planned heading control into path steering

    // references
    AR_AttitudeControl& _atc;       // rover attitude control library
    AR_PosControl &_pos_control;    // rover position control library

    // scurve
    SCurve _scurve_prev_leg;        // previous scurve trajectory used to blend with current scurve trajectory
    SCurve _scurve_this_leg;        // current scurve trajectory
    SCurve _scurve_next_leg;        // next scurve trajectory used to blend with current scurve trajectory
    bool _fast_waypoint;            // true if vehicle will stop at the next waypoint
    bool _pivot_at_next_wp;         // true if vehicle should pivot at next waypoint
    bool _overspeed_enabled;        // if true scurve's position target will speedup to catch vehicles travelling faster than WP_SPEED
    float _track_scalar_dt;         // time scaler to ensure scurve target doesn't get too far ahead of vehicle
    enum class ExactPivotPhase : uint8_t {
        None,
        Move,
        CapturePath,
        Spin,
        PromotedMove,
        Fault,
    } _exact_phase{ExactPivotPhase::None};
    ExactPivotFault _exact_fault{ExactPivotFault::None};
    Location _exact_next_destination;
    Location _exact_frozen_origin;
    float _exact_pivot_heading_cd{0.0f};
    uint32_t _last_yaw_reset_ms{0U};
    uint32_t _exact_position_loss_start_ms{0U};
    bool _exact_position_loss_active{false};
    uint32_t _exact_drift_start_ms{0U};
    bool _exact_drift_active{false};
    uint32_t _exact_timeout_start_ms{0U};
    bool _exact_timeout_active{false};
    bool _exact_reversed{false};
    int8_t _exact_turn_direction{0};
    bool _exact_capture_attempted{false};
    struct ExactCapturePath {
        Vector2f center_from_destination;
        Vector2f start_radial_unit;
        Vector2f tangent_from_destination;
        Vector2f line_unit;
        float radius_m{0.0f};
        float arc_angle_rad{0.0f};
        float arc_length_m{0.0f};
        float total_length_m{0.0f};
        int8_t turn_direction{0};
        bool valid{false};
    } _exact_capture_path;
    SCurve _exact_capture_scurve;
    SCurve _exact_capture_scurve_aux;
    uint32_t _exact_capture_violation_start_ms{0U};
    bool _completion_event_pending{false};
    uint32_t _handoff_generation{0U};
    Location _completed_destination;
    Location _active_destination;
    Vector2f _rejoin_unit;
    float _rejoin_floor_m{0.0f};
    bool _rejoin_active{false};
    struct HeadingHandoff {
        Location origin;
        Vector2f track_unit;
        float track_length_m{0.0f};
        float start_along_m{0.0f};
        float blend_distance_m{0.0f};
        float heading_cd{0.0f};
        uint32_t generation{0U};
        bool active{false};
    } _heading_handoff;
    bool _pivot_handoff{false};      // one-cycle zero-output edge after an ordinary pivot
    float _pivot_heading_cd{0.0f};   // locked effective heading for an ordinary pivot
    bool _pivot_heading_valid{false};

    // variables held in vehicle code (for now)
    float _turn_radius;             // vehicle turn radius in meters

    // variables for navigation
    uint32_t _last_update_ms;       // system time of last call to update
    Location _origin;               // origin Location (vehicle will travel from the origin to the destination)
    Location _destination;          // destination Location when in Guided_WP
    Location _next_destination;     // next destination Location when in Guided_WP
    bool _orig_and_dest_valid;      // true if the origin and destination have been set
    bool _reversed;                 // execute the mission by backing up
    enum class NavControllerType {
        NAV_SCURVE = 0,             // scurves used for navigation
        NAV_PSC_INPUT_SHAPING       // position controller input shaping used for navigation
    } _nav_control_type;            // navigation controller that should be used to travel from _origin to _destination

    // speed_max handling
    float _base_speed_max;          // speed max (in m/s) derived from parameters or passed into init
    float _nudge_speed_max;         // "nudge" speed max (in m/s) normally from the pilot.  has no effect if less than _base_speed_max.  always positive.
    uint32_t _last_speed_update_ms; // system time that speed_max was last update.  used to ensure speed_max is not update too quickly

    // main outputs from navigation library
    float _desired_speed_limited;   // desired speed (above) but accel/decel limited
    float _desired_turn_rate_rads;  // desired turn-rate in rad/sec (negative is counter clockwise, positive is clockwise)
    float _desired_lat_accel;       // desired lateral acceleration (for reporting only)
    float _desired_heading_cd;      // desired heading (back towards line between origin and destination)
    float _wp_bearing_cd;           // heading to waypoint in centi-degrees
    float _cross_track_error;       // cross track error (in meters).  distance from current position to closest point on line between origin and destination

    // variables for reporting
    float _distance_to_destination; // distance from vehicle to final destination in meters
    bool _reached_destination;      // true once the vehicle has reached the destination

private:
    struct ExactPivotDiagCache {
        uint16_t transition_flags{0U};
        uint32_t transition_sequence{0U};
        uint32_t leg_id{0U};
        uint32_t acknowledged_generation{0U};
        bool endpoint_valid{false};
        bool plan_valid{false};
        bool path_terminal{false};
        bool target_heading_valid{false};
        float endpoint_distance_m{0.0f};
        float planned_distance_m{0.0f};
        float planned_speed_mps{0.0f};
        float target_heading_deg{0.0f};
        uint8_t spin_source{0U};
        uint8_t fault_from_phase{0U};
        uint8_t oa_cancel_from_phase{0U};
        uint8_t oa_cancel_reason{0U};
        ExactPivotCaptureDiag capture;
        ExactPivotHandoffDiag handoff;
    } _exact_diag;

    // return the common speed limit for an exact-pivot capture path
    float get_exact_capture_speed_max(float radius_m) const;

    // Exact-pivot diagnostic writers.  These only copy values already used by
    // the controller and are never read by navigation decisions.
    void mark_exact_pivot_diag_event(uint16_t event);
    void start_exact_pivot_diag_leg(uint16_t event);
    void update_exact_pivot_diag_endpoint(const Location &current_loc);
    static uint8_t exact_pivot_phase_to_diag(ExactPivotPhase phase);
};
