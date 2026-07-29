#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/SCurve.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <APM_Control/AR_PosControl.h>
#include "AR_PivotTurn.h"

class AR_WPNav {
public:

    enum class PlannedPivotResult : uint8_t {
        Running,
        Complete,
        Fault,
    };

    // constructor
    AR_WPNav(AR_AttitudeControl& atc, AR_PosControl &pos_control);

    // initialise waypoint controller. speed_max may be left at zero to use the default
    void init(float speed_max = 0);

    // update navigation
    virtual void update(float dt);

    // get or set maximum speed in m/s
    // if set_speed_max is called in rapid succession changes may be delayed by up to 0.5sec
    float get_speed_max() const { return _base_speed_max; }
    bool set_speed_max(float speed_max);

    // set speed nudge in m/s. this has no effect unless nudge_speed_max > speed_max
    void set_nudge_speed_max(float nudge_speed_max);

    // execute the mission in reverse
    bool get_reversed() const { return _reversed; }
    void set_reversed(bool reversed);

    // navigation outputs
    float get_speed() const { return _desired_speed_limited; }
    float get_turn_rate_rads() const { return _desired_turn_rate_rads; }
    float get_lat_accel() const { return _desired_lat_accel; }

    // set desired location and optional next destination for normal waypoint navigation
    virtual bool set_desired_location(const Location &destination,
                                      Location next_destination = Location()) WARN_IF_UNUSED;

    // Install one explicit theoretical line with a zero-speed endpoint. This
    // contract has no next-leg preview, waypoint smoothing or entry pivot.
    virtual bool set_desired_location_stopping_from_origin(
        const Location &origin,
        const Location &destination,
        float speed_max_mps = 0.0f) WARN_IF_UNUSED;
    virtual void cancel_stopping_line();

    // Read-only stopping-line outputs used by Patrol's Drive state.
    bool path_terminal() const { return _path_terminal; }
    float get_path_target_speed() const { return _path_target_speed_mps; }

    // Minimal planned-spin primitive used by Patrol's own Drive/Spin state machine.
    bool start_planned_pivot(float target_heading_cd,
                             int8_t preferred_direction = 0) WARN_IF_UNUSED;
    PlannedPivotResult update_planned_pivot(float &turn_rate_rads);
    void cancel_planned_pivot();

    // set desired location to a reasonable stopping point
    bool set_desired_location_to_stopping_location() WARN_IF_UNUSED;

    // set desired location as offset from the EKF origin
    bool set_desired_location_NED(const Vector3f &destination) WARN_IF_UNUSED;
    bool set_desired_location_NED(const Vector3f &destination,
                                  const Vector3f &next_destination) WARN_IF_UNUSED;

    // set desired location but expect it to be updated again soon
    // object avoidance is not supported by this interface
    bool set_desired_location_expect_fast_update(const Location &destination) WARN_IF_UNUSED;

    // true if vehicle has reached desired location
    virtual bool reached_destination() const { return _reached_destination; }

    float get_distance_to_destination() const { return _distance_to_destination; }
    bool is_destination_valid() const { return _orig_and_dest_valid; }
    const Location &get_destination() const { return _destination; }

    // reporting outputs
    float wp_bearing_cd() const { return _wp_bearing_cd; }
    float nav_bearing_cd() const { return _desired_heading_cd; }
    float crosstrack_error() const { return _cross_track_error; }

    virtual const Location &get_oa_origin() const { return _origin; }
    virtual const Location &get_oa_destination() const { return get_destination(); }

    // provide vehicle turn capabilities
    void set_turn_params(float turn_radius, bool pivot_possible);

    // parameter accessors
    float get_default_speed() const { return _speed_max; }
    float get_default_accel() const { return _accel_max; }
    float get_default_jerk() const { return _jerk_max; }
    float get_radius() const { return _radius; }
    float get_pivot_radius() const { return constrain_float(_pivot_radius, 0.05f, 2.0f); }
    float get_pivot_drift() const { return constrain_float(_pivot_drift, 0.0f, 3.0f); }

    // calculate stopping location using current velocity and maximum deceleration
    bool get_stopping_location(Location &stopping_loc) WARN_IF_UNUSED;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    bool is_active() const;
    void advance_wp_target_along_track(const Location &current_loc, float dt);
    void update_psc_input_shaping(float dt);
    void update_distance_and_bearing_to_destination();
    void update_steering_and_speed(const Location &current_loc, float dt);
    void check_pivot_activation(bool force_active = false);
    void set_navigation_outputs_zero()
    {
        _desired_speed_limited = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _desired_lat_accel = 0.0f;
    }
    float calc_crosstrack_error(const Location &current_loc) const;
    float get_corner_angle(const Location &loc1,
                           const Location &loc2,
                           const Location &loc3) const;
    void init_pos_control_if_necessary();
    bool set_origin_and_destination_to_stopping_point();
    void update_speed_max();

    float get_pivot_target_heading_cd() const;
    float get_leg_speed_max(float requested_speed_max_mps) const;

    // Build a zero-speed stopping S-curve between two locations.
    bool calculate_stopping_scurve(const Location &origin,
                                   const Location &destination,
                                   SCurve &scurve,
                                   float speed_max_mps = 0.0f) WARN_IF_UNUSED;

    // The stopping line may start at an already-traversed point when Patrol
    // resumes. Only along-line progress is absorbed; the theoretical line is
    // never rotated or replaced from realtime position.
    void apply_stopping_line_progress_floor(const Vector2f &origin_ne_m,
                                            Vector3p &target_pos);
    void clear_stopping_line_progress();
    void clear_stopping_line_state();

    // parameters
    AP_Float _speed_max;
    AP_Float _radius;
    AR_PivotTurn _pivot;
    AP_Float _accel_max;
    AP_Float _jerk_max;
    AP_Float _pivot_radius;         // Patrol endpoint radius before Spin
    AP_Float _pivot_drift;          // Patrol Spin position drift allowance

    // references
    AR_AttitudeControl &_atc;
    AR_PosControl &_pos_control;

    // S-curve navigation
    SCurve _scurve_prev_leg;
    SCurve _scurve_this_leg;
    SCurve _scurve_next_leg;
    bool _fast_waypoint;
    bool _pivot_at_next_wp;
    bool _overspeed_enabled;
    float _track_scalar_dt;

    // Patrol stopping-line and planned-spin state
    bool _stopping_line_active{false};
    float _stopping_line_speed_max_mps{0.0f};
    bool _path_terminal{false};
    float _path_target_speed_mps{0.0f};
    Vector2f _stopping_line_progress_unit;
    float _stopping_line_progress_floor_m{0.0f};
    bool _stopping_line_progress_active{false};
    float _planned_pivot_heading_cd{0.0f};
    bool _planned_pivot_active{false};
    uint32_t _last_yaw_reset_ms{0U};

    // ordinary pivot heading lock
    float _pivot_heading_cd{0.0f};
    bool _pivot_heading_valid{false};

    // variables held in vehicle code
    float _turn_radius;

    // navigation state
    uint32_t _last_update_ms;
    Location _origin;
    Location _destination;
    Location _next_destination;
    bool _orig_and_dest_valid;
    bool _reversed;
    enum class NavControllerType {
        NAV_SCURVE = 0,
        NAV_PSC_INPUT_SHAPING
    } _nav_control_type;

    // speed handling
    float _base_speed_max;
    float _nudge_speed_max;
    uint32_t _last_speed_update_ms;

    // outputs
    float _desired_speed_limited;
    float _desired_turn_rate_rads;
    float _desired_lat_accel;
    float _desired_heading_cd;
    float _wp_bearing_cd;
    float _cross_track_error;

    // reporting
    float _distance_to_destination;
    bool _reached_destination;
};
