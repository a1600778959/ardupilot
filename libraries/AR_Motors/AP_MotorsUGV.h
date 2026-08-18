#pragma once

#include <AP_Arming/AP_Arming.h>
#include <AP_WheelEncoder/AP_WheelRateControl.h>
#include <SRV_Channel/SRV_Channel.h>

class AP_MotorsUGV {
public:
    // Constructor
    AP_MotorsUGV(AP_WheelRateControl& rate_controller);
    // singleton support
    static AP_MotorsUGV    *get_singleton(void) { return _singleton; }

    enum motor_test_order {
        MOTOR_TEST_THROTTLE = 1,
        MOTOR_TEST_STEERING = 2,
        MOTOR_TEST_THROTTLE_LEFT = 3,
        MOTOR_TEST_THROTTLE_RIGHT = 4,
        MOTOR_TEST_LAST
    };

    // initialise motors
    void init();

    // return true if motors are active
    bool active() const;

    // setup output in case of main CPU failure
    void setup_safety_output();

    // setup servo output ranges
    void setup_servo_output();

    // get or set steering as a value from -4500 to +4500
    // apply_scaling is retained for API compatibility and ignored in differential mode
    float get_steering() const { return _steering; }
    void set_steering(float steering);

    // get or set throttle as a value from -100 to 100
    float get_throttle() const { return _throttle; }
    void set_throttle(float throttle);

    // get slew limited throttle
    // used by manual mode to avoid bad steering behaviour during transitions from forward to reverse
    // same as private slew_limit_throttle method (see below) but does not update throttle state
    float get_slew_limited_throttle(float throttle, float dt) const;

    // true if vehicle is capable of skid steering
    bool have_skid_steering() const;

    // configured throttle ceiling in percent
    float get_throttle_max() const { return constrain_float(_throttle_max, 0.0f, 100.0f); }

    // temporary absolute limit applied to each final left/right motor output.
    // This is a runtime safety limit and is not an EEPROM parameter.
    void set_actuator_output_limit_pct(float limit_pct) {
        _actuator_output_limit_pct = constrain_float(limit_pct, 0.0f, 100.0f);
    }
    void clear_actuator_output_limit() {
        _actuator_output_limit_pct = 100.0f;
        _actuator_output_limited = false;
    }
    bool actuator_output_limited() const { return _actuator_output_limited; }

    // output to motors and steering servos
    // ground_speed should be the vehicle's speed over the surface in m/s
    // dt should be expected time between calls to this function
    void output(bool armed, float ground_speed, float dt);

    // test steering or throttle output as a percentage of the total (range -100 to +100)
    // used in response to DO_MOTOR_TEST mavlink command
    bool output_test_pct(motor_test_order motor_seq, float pct);

    // test steering or throttle output using a pwm value
    bool output_test_pwm(motor_test_order motor_seq, float pwm);

    //  returns true if checks pass, false if they fail.  display_failure argument should be true to send text messages to GCS
    bool pre_arm_check(bool report) const;

    // return the motor mask
    uint32_t get_motor_mask() const { return _motor_mask; }

    // returns true if the configured PWM type is digital and should have fixed endpoints
    bool is_digital_pwm_type() const;

    // Return the relay index that would be used for param conversion to relay functions
    bool get_legacy_relay_index(int8_t &index1, int8_t &index2, int8_t &index3, int8_t &index4) const;

    // structure for holding motor limit flags
    struct AP_MotorsUGV_limit {
        uint8_t steer_left      : 1; // we have reached the steering controller's left most limit
        uint8_t steer_right     : 1; // we have reached the steering controller's right most limit
        uint8_t throttle_lower  : 1; // we have reached throttle's lower limit
        uint8_t throttle_upper  : 1; // we have reached throttle's upper limit
    } limit;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

private:

    enum PWMType {
        NORMAL = 0,
        ONESHOT = 1,
        ONESHOT125 = 2,
        BRUSHED_WITH_RELAY = 3,
        BRUSHED_BIPOLAR = 4,
        DSHOT150 = 5,
        DSHOT300 = 6,
        DSHOT600 = 7,
        DSHOT1200 = 8
    };

    // sanity check parameters
    void sanity_check_parameters();

    // setup pwm output type
    void setup_pwm_type();

    // output to skid steering channels
    void output_skid_steering(bool armed, float steering, float throttle, float dt);

    // output throttle (-100 ~ +100) to a throttle channel.  Sets relays if required
    // dt is the main loop time interval and is required when rate control is required
    void output_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt = 0.0f);
    
    // slew limit throttle for one iteration
    void slew_limit_throttle(float dt);

    // slew limit steering for one iteration
    void slew_limit_steering(float dt);

    // set limits based on steering and throttle input
    void set_limits_from_input(bool armed, float steering, float throttle);

    // scale a throttle using the _thrust_curve_expo parameter.  throttle should be in the range -100 to +100
    float get_scaled_throttle(float throttle) const;

    // use rate controller to achieve desired throttle
    float get_rate_controlled_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt);

    // external references
    AP_WheelRateControl &_rate_controller;

    // parameters
    AP_Int8 _pwm_type;  // PWM output type
    AP_Int8 _pwm_freq;  // PWM output freq for brushed motors
    AP_Int8 _disarm_disable_pwm;    // disable PWM output while disarmed
    AP_Int16 _slew_rate; // slew rate expressed as a percentage / second
    AP_Int8 _throttle_min; // throttle minimum percentage
    AP_Int8 _throttle_max; // throttle maximum percentage
    AP_Float _steering_curve_tc; // steering curve time constant in seconds for skid steering
    AP_Float _steering_curve_strength; // steering curve strength for skid steering response
    AP_Float _thrust_curve_expo; // thrust curve exponent from -1 to +1 with 0 being linear
    AP_Float _thrust_asymmetry; // asymmetry factor, how much better your skid-steering motors are at going forward than backwards (forward/backward thrust ratio)
    AP_Float _steering_throttle_mix; // Steering vs Throttle priorisation.  Higher numbers prioritise steering, lower numbers prioritise throttle.  Only valid for Skid Steering vehicles
    AP_Float _stop_distance; // distance in meters to stop UGV when obstacle detected
    AP_Float _reverse_delay; // delay in seconds when reversing motor

    // internal variables
    float   _steering;  // requested steering as a value from -4500 to +4500
    float   _steering_prev; // limited steering request from previous iteration
    float   _steering_rate_state; // steering rate state used by curved skid-steering response
    float   _throttle;  // requested throttle as a value from -100 to 100
    float   _throttle_prev; // limited throttle request from previous iteration
    float   _actuator_output_limit_pct{100.0f}; // temporary final left/right output limit
    bool    _actuator_output_limited{false}; // true if the temporary limit clipped either wheel this cycle
    uint32_t _motor_mask;   // mask of motors configured with pwm_type

    struct ReverseThrottle {
        float last_throttle = 0.0f;
        uint32_t last_output_ms = 0;

        // output with delay for reversal
        void output(SRV_Channel::Aux_servo_function_t function, float throttle, float delay);
    } rev_delay_throttleLeft, rev_delay_throttleRight;

    static AP_MotorsUGV *_singleton;
};

namespace AP {
    AP_MotorsUGV *motors_ugv();
};
