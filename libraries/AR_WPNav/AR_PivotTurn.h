#pragma once

#include <AP_Common/AP_Common.h>
#include <APM_Control/AR_AttitudeControl.h>
#include "AR_PivotTurnCompletion.h"

/*
 * Pivot Turn controller for skid-steering rovers and boats
 *
 * How-to-Use:
 *    1. call "enable(true)" once for skid-steering vehicles to enable this controller
 *    2. before vehicle starts towards a waypoint call "check_activation" and provide the earth-frame heading to the waypoint
 *       this may change the controller's internal state to "active"
 *    3. on each main loop iteration call "active()" to see if this controller thinks it is controllering the vehicle
 *    4. call "get_turn_rate_rads()" to retrieve the desired turn rate towards the next waypoint
 *    5. pass above turn rate into the rate controller and apply the caller's speed policy
 *    6. call "update_completion()" with the measured yaw rate
 *    7. this controller's "active" state changes to false after heading accuracy and delay are satisfied
 */

class AR_PivotTurn {
public:

    // constructor
    AR_PivotTurn(AR_AttitudeControl& atc);

    // enable or disable pivot controller
    void enable(bool enable_pivot);

    // true if this controller is controlling vehicle
    bool active() const;

    // true if this vehicle can execute an explicitly planned pivot.  The
    // ordinary automatic-pivot angle is a policy setting, not a hardware gate.
    bool available() const { return _enabled; }

    // checks if pivot turns should be activated or deactivated
    // force_active should be true if the caller wishes to trigger the start of a pivot turn regardless of the heading error
    void check_activation(float desired_heading_deg, bool force_active = false);

    // check if pivot turn would be activated given an expected change in yaw in degrees
    // note this does not actually active the pivot turn.  To activate use the check_activation method
    bool would_activate(float yaw_change_deg) const WARN_IF_UNUSED;

    // forcibly deactivate this controller
    void deactivate();

    // get turn rate (in rad/sec) without changing completion state
    // desired heading should be the heading towards the next waypoint in degrees
    float get_turn_rate_rads(float desired_heading_deg);

    // update completion state using heading error and measured yaw rate
    // returns true only on the active-to-inactive completion edge
    bool update_completion(float desired_heading_deg, float yaw_rate_rads, uint32_t now_ms);

    // clear completion timing without leaving the active turn
    void reset_completion() { _completion.reset(); }

    // reset completion and planned direction state after an estimator yaw reset
    void handle_yaw_reset();

    // force activation for an explicitly planned spin primitive.  A preferred
    // direction of -1 or +1 resolves the otherwise ambiguous 180 degree case.
    bool activate_planned(int8_t preferred_direction = 0);

    // accessors for parameter values
    float get_rate_max() const { return _rate_max; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // return post-turn delay duration in milliseconds
    uint32_t get_delay_duration_ms() const;

    // return absolute heading error in degrees
    float get_heading_error_deg(float desired_heading_deg) const;

    // clear state used only by explicitly planned turns
    void clear_planned_state();

    // parameters
    AP_Int16 _angle;                // minimum angle error (in degrees) that leads to pivot turn
    AP_Int16 _rate_max;             // maximum turn rate (in degrees) during pivot turn
    AP_Float _delay;                // waiting time (in seconds) after pivot turn completes

    // references
    AR_AttitudeControl& _atc;       // rover attitude control library

    // local variables
    bool _enabled;                  // true if vehicle can pivot
    bool _active;                   // true if vehicle is currently pivoting
    AR_PivotTurnCompletion _completion;
    uint32_t _legacy_delay_start_ms{0U};
    bool _planned_turn{false};
    bool _planned_direction_initialised{false};
    int8_t _planned_turn_direction{0};
    int8_t _planned_preferred_direction{0};
};
