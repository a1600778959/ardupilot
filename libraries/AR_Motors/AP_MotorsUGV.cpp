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

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsUGV.h"
#include <AP_Relay/AP_Relay.h>

#define SERVO_MAX 4500  // This value represents 45 degrees and is just an arbitrary representation of servo max travel.

extern const AP_HAL::HAL& hal;

// singleton instance
AP_MotorsUGV *AP_MotorsUGV::_singleton;

// parameters for the motor class
const AP_Param::GroupInfo AP_MotorsUGV::var_info[] = {
    // @Param: PWM_TYPE
    // @DisplayName: Motor Output PWM type
    // @Description: This selects the output PWM type as regular PWM, OneShot, Brushed motor support using PWM (duty cycle) with separated direction signal, Brushed motor support with separate throttle and direction PWM (duty cyle)
    // @Values: 0:Normal,1:OneShot,2:OneShot125,3:BrushedWithRelay,4:BrushedBiPolar,5:DShot150,6:DShot300,7:DShot600,8:DShot1200
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_TYPE", 1, AP_MotorsUGV, _pwm_type, PWMType::NORMAL),

    // @Param: PWM_FREQ
    // @DisplayName: Motor Output PWM freq for brushed motors
    // @Description: Motor Output PWM freq for brushed motors
    // @Units: kHz
    // @Range: 1 20
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_FREQ", 2, AP_MotorsUGV, _pwm_freq, 16),

    // @Param: SAFE_DISARM
    // @DisplayName: Motor PWM output disabled when disarmed
    // @Description: Disables motor PWM output when disarmed
    // @Values: 0:PWM enabled while disarmed, 1:PWM disabled while disarmed
    // @User: Advanced
    AP_GROUPINFO("SAFE_DISARM", 3, AP_MotorsUGV, _disarm_disable_pwm, 0),

    // @Param: THR_MIN
    // @DisplayName: Throttle minimum
    // @Description: Throttle minimum percentage the autopilot will apply. This is useful for handling a deadzone around low throttle and for preventing internal combustion motors cutting out during missions. Must be less than MOT_THR_MAX.
    // @Units: %
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MIN", 5, AP_MotorsUGV, _throttle_min, 0),

    // @Param: THR_MAX
    // @DisplayName: Throttle maximum
    // @Description: Throttle maximum percentage the autopilot will apply. This can be used to prevent overheating an ESC or motor on an electric rover
    // @Units: %
    // @Range: 5 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MAX", 6, AP_MotorsUGV, _throttle_max, 100),

    // @Param: SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: Throttle slew rate as a percentage of total range per second. A value of 100 allows the motor to change over its full range in one second.  A value of zero disables the limit.  Note some NiMH powered rovers require a lower setting of 40 to reduce current demand to avoid brownouts.
    // @Units: %/s
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SLEWRATE", 8, AP_MotorsUGV, _slew_rate, 100),

    // @Param: STR_TC
    // @DisplayName: Steering Curve Time Constant
    // @Description: Time constant for skid-steering steering response curve. Zero disables this curve and makes steering output follow input directly. Larger values produce smoother, more curved left/right transitions.
    // @Units: s
    // @Range: 0 2
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("STR_TC", 17, AP_MotorsUGV, _steering_curve_tc, 0.0f),

    // @Param: STR_CURVE
    // @DisplayName: Steering Curve Strength
    // @Description: Curvature strength for skid-steering steering response. Zero gives a standard 2nd-order response. Higher values make the slow-fast-slow S-curve more pronounced.
    // @Range: 0 30
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("STR_CURVE", 18, AP_MotorsUGV, _steering_curve_strength, 0.0f),

    // @Param: THST_EXPO
    // @DisplayName: Thrust Curve Expo
    // @Description: Thrust curve exponent (-1 to +1 with 0 being linear)
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("THST_EXPO", 9, AP_MotorsUGV, _thrust_curve_expo, 0.0f),

    // 10 was VEC_THR_BASE
    // 11 was SPD_SCA_BASE

    // @Param: STR_THR_MIX
    // @DisplayName: Motor steering vs throttle prioritisation
    // @Description: Steering vs Throttle priorisation.  Higher numbers prioritise steering, lower numbers prioritise throttle.  Only valid for Skid Steering vehicles
    // @Range: 0.2 1.0
    // @User: Advanced
    AP_GROUPINFO("STR_THR_MIX", 12, AP_MotorsUGV, _steering_throttle_mix, 0.5f),

    // 13 was VEC_ANGLEMAX

    // @Param: THST_ASYM
    // @DisplayName: Motor Thrust Asymmetry
    // @Description: Thrust Asymetry. Used for skid-steering. 2.0 means your motors move twice as fast forward than they do backwards.
    // @Range: 1.0 10.0
    // @User: Advanced
    AP_GROUPINFO("THST_ASYM", 14, AP_MotorsUGV, _thrust_asymmetry, 1.0f),

    AP_GROUPINFO("STOP_DIST", 15, AP_MotorsUGV, _stop_distance, 1000.0f),

    // @Param: REV_DELAY
    // @DisplayName: Motor reversal delay
    // @Description: For reversible motors that need a delay before they can change direction. When greater than zero the throttle will go to zero for this amount of time before outputting the new throttle when the demanded motor direction changes.
    // @Units: s
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("REV_DELAY", 19, AP_MotorsUGV, _reverse_delay, 0),

    AP_GROUPEND
};

AP_MotorsUGV::AP_MotorsUGV(AP_WheelRateControl& rate_controller) :
    _rate_controller(rate_controller)
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void AP_MotorsUGV::init()
{
    // setup servo output
    setup_servo_output();

    // setup pwm type
    setup_pwm_type();

    // set safety output
    setup_safety_output();

}

bool AP_MotorsUGV::get_legacy_relay_index(int8_t &index1, int8_t &index2, int8_t &index3, int8_t &index4) const
{
    index1 = -1;
    index2 = -1;
    index3 = -1;
    index4 = -1;

    if (_pwm_type != PWMType::BRUSHED_WITH_RELAY) {
        // Relays only used if PWM type is set to brushed with relay
        return false;
    }

    // Differential drive uses two brushed reverse relays (left/right).
    index1 = 0;
    if (have_skid_steering()) {
        index2 = 1;
    }

    return true;
}

// setup output in case of main CPU failure
void AP_MotorsUGV::setup_safety_output()
{
    if (_pwm_type == PWMType::BRUSHED_WITH_RELAY) {
        // set trim to min to set duty cycle range (0 - 100%) to servo range
        // ignore servo revese flag, it is used by the relay
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleLeft, true);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleRight, true);
    }

    // stop sending pwm if main CPU fails
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::ZERO_PWM);
}

// setup servo output ranges
void AP_MotorsUGV::setup_servo_output()
{
    // skid steering left/right throttle as -1000 to 1000 values
    SRV_Channels::set_angle(SRV_Channel::k_throttleLeft,  1000);
    SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 1000);
}

// set steering as a value from -4500 to +4500
void AP_MotorsUGV::set_steering(float steering)
{
    _steering = steering;
}

// set throttle as a value from -100 to 100
void AP_MotorsUGV::set_throttle(float throttle)
{
    // only allow setting throttle if armed
    if (!hal.util->get_soft_armed()) {
        return;
    }    
    _throttle = constrain_float(throttle, -_throttle_max, _throttle_max);
}

// get slew limited throttle
// used by manual mode to avoid bad steering behaviour during transitions from forward to reverse
// same as private slew_limit_throttle method (see below) but does not update throttle state
float AP_MotorsUGV::get_slew_limited_throttle(float throttle, float dt) const
{
    if (_slew_rate <= 0) {
        return throttle;
    }

    const float throttle_change_max = static_cast<float>(_slew_rate) * dt;
    return constrain_float(throttle, _throttle_prev - throttle_change_max, _throttle_prev + throttle_change_max);
}

/*
  work out if skid steering is available
 */
bool AP_MotorsUGV::have_skid_steering() const
{
    return SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) &&
           SRV_Channels::function_assigned(SRV_Channel::k_throttleRight);
}

void AP_MotorsUGV::output(bool armed, float ground_speed, float dt)
{
    (void)ground_speed;
    // soft-armed overrides passed in armed status
    if (!hal.util->get_soft_armed()) {
        armed = false;
        _throttle = 0.0f;
    }

    // clear limit flags
    // output_ methods are responsible for setting them to true if required on each iteration
    limit.steer_left = limit.steer_right = limit.throttle_lower = limit.throttle_upper = false;
    _actuator_output_limited = false;

    // sanity check parameters
    sanity_check_parameters();

    // slew limit steering before throttle so skid-steering can use the previous
    // throttle state to work out how much wheel-output delta remains this cycle
    slew_limit_steering(dt);
    slew_limit_throttle(dt);

    // differential-drive output path
    output_skid_steering(armed, _steering, _throttle, dt);

    // send values to the PWM timers for output
    auto &srv = AP::srv();
    SRV_Channels::calc_pwm();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();
}

// test steering or throttle output as a percentage of the total (range -100 to +100)
// used in response to DO_MOTOR_TEST mavlink command
bool AP_MotorsUGV::output_test_pct(motor_test_order motor_seq, float pct)
{
    // check if the motor_seq is valid
    if (motor_seq >= MOTOR_TEST_LAST) {
        return false;
    }
    pct = constrain_float(pct, -100.0f, 100.0f);

    switch (motor_seq) {
        case MOTOR_TEST_THROTTLE: {
            output_throttle(SRV_Channel::k_throttleLeft, pct);
            output_throttle(SRV_Channel::k_throttleRight, pct);
            break;
        }
        case MOTOR_TEST_STEERING: {
            output_throttle(SRV_Channel::k_throttleLeft, pct);
            output_throttle(SRV_Channel::k_throttleRight, -pct);
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            output_throttle(SRV_Channel::k_throttleLeft, pct);
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            output_throttle(SRV_Channel::k_throttleRight, pct);
            break;
        }
        case MOTOR_TEST_LAST:
            return false;
    }
    auto &srv = AP::srv();
    SRV_Channels::calc_pwm();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();
    return true;
}

// test steering or throttle output using a pwm value
bool AP_MotorsUGV::output_test_pwm(motor_test_order motor_seq, float pwm)
{
    // check if the motor_seq is valid
    if (motor_seq > MOTOR_TEST_THROTTLE_RIGHT) {
        return false;
    }
    switch (motor_seq) {
        case MOTOR_TEST_THROTTLE: {
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        }
        case MOTOR_TEST_STEERING: {
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            if (const SRV_Channel *right = SRV_Channels::get_channel_for(SRV_Channel::k_throttleRight)) {
                const uint16_t mirrored = uint16_t(MAX(0, (2 * right->get_trim()) - int(pwm)));
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, mirrored);
            } else {
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        }
        default:
            return false;
    }
    auto &srv = AP::srv();
    SRV_Channels::calc_pwm();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();
    return true;
}

//  returns true if checks pass, false if they fail.  report should be true to send text messages to GCS
bool AP_MotorsUGV::pre_arm_check(bool report) const
{
    const bool have_throttle_left = SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft);
    const bool have_throttle_right = SRV_Channels::function_assigned(SRV_Channel::k_throttleRight);

    // Differential-drive mode requires both left and right motor outputs.
    if (!have_throttle_left && !have_throttle_right) {
        if (report) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: no skid-steering outputs defined");
        }
        return false;
    }

    // check if only one of skid-steering output has been configured
    if (have_throttle_left != have_throttle_right) {
        if (report) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: check skid steering config");
        }
        return false;
    }

    // Check relays are configured for brushed with relay outputs
#if AP_RELAY_ENABLED
    AP_Relay*relay = AP::relay();
    if ((_pwm_type == PWMType::BRUSHED_WITH_RELAY) && (relay != nullptr)) {
        // If a output is configured its relay must be enabled
        struct RelayTable {
            bool output_assigned;
            AP_Relay_Params::FUNCTION fun;
        };

        const RelayTable relay_table[] = {
            { have_throttle_left,  AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_1 },
            { have_throttle_right, AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_2 },
        };

        for (uint8_t i=0; i<ARRAY_SIZE(relay_table); i++) {
            if (relay_table[i].output_assigned && !relay->enabled(relay_table[i].fun)) {
                if (report) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: relay function %u unassigned", uint8_t(relay_table[i].fun));
                }
                return false;
            }
        }
    }
#endif

    return true;
}

// sanity check parameters
void AP_MotorsUGV::sanity_check_parameters()
{
    _throttle_max.set(constrain_int16(_throttle_max, 5, 100));
    _throttle_min.set(constrain_int16(_throttle_min, 0, MIN(20, _throttle_max)));
}

// setup pwm output type
void AP_MotorsUGV::setup_pwm_type()
{
    _motor_mask = 0;

    hal.rcout->set_dshot_esc_type(SRV_Channels::get_dshot_esc_type());

    // work out mask of channels assigned to motors
    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_throttleLeft);
    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_throttleRight);

    switch (_pwm_type) {
    case PWMType::ONESHOT:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        break;
    case PWMType::ONESHOT125:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT125);
        break;
    case PWMType::BRUSHED_WITH_RELAY:
    case PWMType::BRUSHED_BIPOLAR:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        hal.rcout->set_freq(_motor_mask, uint16_t(_pwm_freq * 1000));
        break;
    case PWMType::DSHOT150:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT150);
        break;
    case PWMType::DSHOT300:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT300);
        break;
    case PWMType::DSHOT600:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT600);
        break;
    case PWMType::DSHOT1200:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT1200);
        break;
    default:
        // do nothing
        break;
    }
}

// output to skid steering channels
void AP_MotorsUGV::output_skid_steering(bool armed, float steering, float throttle, float dt)
{
    if (!have_skid_steering()) {
        return;
    }

    // clear and set limits based on input
    set_limits_from_input(armed, steering, throttle);

    // constrain steering
    steering = constrain_float(steering, -4500.0f, 4500.0f);

    // handle simpler disarmed case
    if (!armed) {
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
        }
        return;
    }

    // skid steering mixer
    float steering_scaled = steering / 4500.0f; // steering scaled -1 to +1
    float throttle_scaled = throttle * 0.01f;  // throttle scaled -1 to +1

    // sanitize values for asymmetry of thrust, mixer assumes forward thrust is always larger than reverse
    const float thrust_asymmetry = MAX(_thrust_asymmetry, 1.0);
    const float lower_throttle_limit = -1.0 / thrust_asymmetry;

    // Maximum steering is half way between upper and lower limits
    const float best_steering_throttle = (1.0 + lower_throttle_limit) * 0.5;
    float steering_range;
    if (throttle_scaled < best_steering_throttle) {
        // steering range is reduced as throttle will never be increased by mixer
        steering_range = MAX(throttle_scaled,0.0) - lower_throttle_limit;
    } else {
        // full range available, throttle can always be lowered down to best_steering_throttle
        steering_range = 1 - best_steering_throttle;
    }

    // apply steering constraints
    if (steering_scaled > steering_range) {
        limit.steer_right = true;
        steering_scaled = steering_range;
    } else if (steering_scaled < -steering_range) {
        limit.steer_left = true;
        steering_scaled = -steering_range;
    }

    if (throttle_scaled > 1.0) {
        limit.throttle_upper = true;
        throttle_scaled = 1.0;
    } else if (throttle_scaled < lower_throttle_limit) {
        limit.throttle_lower = true;
        throttle_scaled = lower_throttle_limit;
    }

    // All throttle or all steering will now fit, check if they will both fit together
    const float max_output = throttle_scaled + fabsf(steering_scaled);
    const float min_output = throttle_scaled - fabsf(steering_scaled);

    // check for saturation and scale back throttle and steering proportionally
    const float saturation_value = MAX(max_output, min_output / lower_throttle_limit);
    if (saturation_value > 1.0f) {
        // store pre-scaled values so we can set limit flags afterwards
        const float steering_scaled_orig = steering_scaled;
        const float throttle_scaled_orig = throttle_scaled;

        const float str_thr_mix = constrain_float(_steering_throttle_mix, 0.0f, 1.0f);
        const float fair_scaler = 1.0f / saturation_value;
        if (str_thr_mix >= 0.5f) {
            // prioritise steering over throttle
            steering_scaled *= linear_interpolate(fair_scaler, 1.0f, str_thr_mix, 0.5f, 1.0f);
            if (throttle_scaled >= best_steering_throttle) {
                // constrained by upper limit
                throttle_scaled = 1.0 - fabsf(steering_scaled);
            } else {
                // constrained by lower limit
                throttle_scaled = fabsf(steering_scaled) + lower_throttle_limit;
            }

        } else {
            // prioritise throttle over steering
            throttle_scaled *= linear_interpolate(fair_scaler, 1.0f, 0.5f - str_thr_mix, 0.0f, 0.5f);
            const float steering_sign = is_positive(steering_scaled) ? 1.0 : -1.0;
            if (throttle_scaled >= best_steering_throttle) {
                // constrained by upper limit
                steering_scaled = (1.0 - throttle_scaled) * steering_sign;
            } else {
                // constrained by lower limit
                steering_scaled = (throttle_scaled - lower_throttle_limit) * steering_sign;
            }
        }

        // update limits if either steering or throttle has been reduced
        if (fabsf(steering_scaled) < fabsf(steering_scaled_orig)) {
            limit.steer_left |= is_negative(steering_scaled_orig);
            limit.steer_right |= is_positive(steering_scaled_orig);
        }
        if (fabsf(throttle_scaled) < fabsf(throttle_scaled_orig)) {
            limit.throttle_lower |= is_negative(throttle_scaled_orig);
            limit.throttle_upper |= is_positive(throttle_scaled_orig);
        }
    }

    // add in throttle and steering
    float motor_left = throttle_scaled + steering_scaled;
    float motor_right = throttle_scaled - steering_scaled;

    // Apply asymmetry correction
    if (is_negative(motor_right)) {
        motor_right *= thrust_asymmetry;
    }
    if (is_negative(motor_left)) {
        motor_left *= thrust_asymmetry;
    }

    // send pwm value to each motor
    output_throttle(SRV_Channel::k_throttleLeft, 100.0f * motor_left, dt);
    output_throttle(SRV_Channel::k_throttleRight, 100.0f * motor_right, dt);
}

// output throttle value to main throttle channel, left throttle or right throttle.  throttle should be scaled from -100 to 100
void AP_MotorsUGV::output_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt)
{
    // sanity check servo function
    if ((function != SRV_Channel::k_throttleLeft) && (function != SRV_Channel::k_throttleRight)) {
        return;
    }

    // constrain and scale output
    throttle = get_scaled_throttle(throttle);

    // apply rate control
    throttle = get_rate_controlled_throttle(function, throttle, dt);

    // Apply any temporary mode-level actuator ceiling after thrust-curve and
    // wheel-rate control so neither path can exceed the requested physical
    // left/right output envelope.
    const float throttle_before_limit = throttle;
    throttle = constrain_float(throttle,
                               -_actuator_output_limit_pct,
                               _actuator_output_limit_pct);
    _actuator_output_limited |= !is_equal(throttle, throttle_before_limit);

    // set relay if necessary
#if AP_RELAY_ENABLED
    AP_Relay*relay = AP::relay();
    if ((_pwm_type == PWMType::BRUSHED_WITH_RELAY) && (relay != nullptr)) {

        // find the output channel, if not found return
        const SRV_Channel *out_chan = SRV_Channels::get_channel_for(function);
        if (out_chan == nullptr) {
            return;
        }
        const int8_t reverse_multiplier = out_chan->get_reversed() ? -1 : 1;
        bool relay_high = is_negative(reverse_multiplier * throttle);

        AP_Relay_Params::FUNCTION relay_function;
        switch (function) {
            case SRV_Channel::k_throttleLeft:
            default:
                relay_function = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_1;
                break;
            case SRV_Channel::k_throttleRight:
                relay_function = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_2;
                break;
        }
        relay->set(relay_function, relay_high);

        // invert the output to always have positive value calculated by calc_pwm
        throttle = reverse_multiplier * fabsf(throttle);
    }
#endif  // AP_RELAY_ENABLED

    if (_reverse_delay > 0) {
        switch (function) {
        case SRV_Channel::k_throttleLeft:
            rev_delay_throttleLeft.output(function, throttle * 10.0f, _reverse_delay);
            return;
        case SRV_Channel::k_throttleRight:
            rev_delay_throttleRight.output(function, throttle * 10.0f, _reverse_delay);
            return;
        default:
            break;
        }
    }

    // output to servo channel
    SRV_Channels::set_output_scaled(function, throttle * 10.0f);
}

// slew limit throttle for one iteration
void AP_MotorsUGV::slew_limit_throttle(float dt)
{
    const float throttle_orig = _throttle;
    _throttle = get_slew_limited_throttle(_throttle, dt);
    if (throttle_orig > _throttle) {
        limit.throttle_upper = true;
    } else if (throttle_orig < _throttle) {
        limit.throttle_lower = true;
    }
    _throttle_prev = _throttle;
}

// slew limit steering for one iteration
void AP_MotorsUGV::slew_limit_steering(float dt)
{
    const float steering_max = static_cast<float>(SERVO_MAX);
    const float steering_orig = _steering;
    const float steering_target = constrain_float(_steering, -steering_max, steering_max);

    if (dt <= 0.0f) {
        _steering = steering_target;
        _steering_prev = _steering;
        _steering_rate_state = 0.0f;
        return;
    }

    const bool is_skid_steer = have_skid_steering();
    const bool use_skid_curve_tc = is_skid_steer && is_positive(_steering_curve_tc);
    if (!use_skid_curve_tc) {
        _steering = steering_target;
        _steering_prev = _steering;
        _steering_rate_state = 0.0f;
        return;
    }

    const float steering_prev = constrain_float(_steering_prev, -steering_max, steering_max);
    float steering_limited = steering_target;

    // Differential drive uses left = v + w and right = v - w, so steering is
    // the wheel-speed differential term.
    const float steering_prev_scaled = steering_prev / steering_max;
    const float steering_target_scaled = steering_target / steering_max;

    // 2nd-order critically damped response with optional non-linear error shaping
    // to increase S-curve visibility (slow-fast-slow).
    const float tc = MAX(static_cast<float>(_steering_curve_tc), 0.02f);
    const float omega = 4.6f / tc;
    const float curve_strength = constrain_float(_steering_curve_strength, 0.0f, 30.0f);
    const float steering_error = steering_target_scaled - steering_prev_scaled;
    const float steering_error_abs = fabsf(steering_error);
    const float center_gain = 1.0f + (0.2f * curve_strength * (1.0f - sq(constrain_float(steering_error_abs, 0.0f, 1.0f))));
    const float steering_error_shaped = constrain_float(steering_error * center_gain, -2.0f, 2.0f);
    const float accel = (sq(omega) * steering_error_shaped) - (2.0f * omega * _steering_rate_state);
    _steering_rate_state += accel * dt;
    float steering_next_scaled = steering_prev_scaled + (_steering_rate_state * dt);

    // preserve differential-drive coupling when throttle slew limiting is active
    if (_slew_rate > 0) {
        const float throttle_prev_scaled = constrain_float(_throttle_prev * 0.01f, -1.0f, 1.0f);
        const float throttle_limited_scaled = constrain_float(get_slew_limited_throttle(_throttle, dt) * 0.01f, -1.0f, 1.0f);
        const float throttle_delta = fabsf(throttle_limited_scaled - throttle_prev_scaled);
        const float steering_step_max = MAX(0.0f, 1.0f - throttle_delta);
        steering_next_scaled = constrain_float(steering_next_scaled,
                                               steering_prev_scaled - steering_step_max,
                                               steering_prev_scaled + steering_step_max);
    }

    // prevent overshoot across target when dt is large or command changes abruptly
    if (((steering_target_scaled - steering_prev_scaled) * (steering_target_scaled - steering_next_scaled)) <= 0.0f) {
        steering_next_scaled = steering_target_scaled;
        _steering_rate_state = 0.0f;
    }

    steering_limited = constrain_float(steering_next_scaled, -1.0f, 1.0f) * steering_max;

    _steering = steering_limited;

    if (steering_orig > _steering) {
        limit.steer_right = true;
    } else if (steering_orig < _steering) {
        limit.steer_left = true;
    }

    _steering_prev = _steering;
}

// set limits based on steering and throttle input
void AP_MotorsUGV::set_limits_from_input(bool armed, float steering, float throttle)
{
    // set limits based on inputs
    limit.steer_left |= !armed || (steering <= -4500.0f);
    limit.steer_right |= !armed || (steering >= 4500.0f);
    limit.throttle_lower |= !armed || (throttle <= -_throttle_max);
    limit.throttle_upper |= !armed || (throttle >= _throttle_max);
}

// scale a throttle using the _throttle_min and _thrust_curve_expo parameters.  throttle should be in the range -100 to +100
float AP_MotorsUGV::get_scaled_throttle(float throttle) const
{
    // exit immediately if throttle is zero
    if (is_zero(throttle)) {
        return throttle;
    }

    // scale using throttle_min
    if (_throttle_min > 0) {
        if (is_negative(throttle)) {
            throttle = -_throttle_min + (throttle * ((100.0f - _throttle_min) * 0.01f));
        } else {
            throttle = _throttle_min + (throttle * ((100.0f - _throttle_min) * 0.01f));
        }
    }

    // skip further scaling if thrust curve disabled or invalid
    if (is_zero(_thrust_curve_expo) || (_thrust_curve_expo > 1.0f) || (_thrust_curve_expo < -1.0f)) {
        return throttle;
    }

    // calculate scaler
    const float sign = (throttle < 0.0f) ? -1.0f : 1.0f;
    const float throttle_pct = constrain_float(throttle, -100.0f, 100.0f) * 0.01f;
    return 100.0f * sign * ((_thrust_curve_expo - 1.0f) + safe_sqrt((1.0f - _thrust_curve_expo) * (1.0f - _thrust_curve_expo) + 4.0f * _thrust_curve_expo * fabsf(throttle_pct))) / (2.0f * _thrust_curve_expo);
}

// use rate controller to achieve desired throttle
float AP_MotorsUGV::get_rate_controlled_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt)
{
    // require non-zero dt
    if (!is_positive(dt)) {
        return throttle;
    }

    // attempt to rate control left throttle
    if ((function == SRV_Channel::k_throttleLeft) && _rate_controller.enabled(0)) {
        return _rate_controller.get_rate_controlled_throttle(0, throttle, dt);
    }

    // rate control right throttle
    if ((function == SRV_Channel::k_throttleRight) && _rate_controller.enabled(1)) {
        return _rate_controller.get_rate_controlled_throttle(1, throttle, dt);
    }

    // return throttle unchanged
    return throttle;
}

// return true if motors are moving
bool AP_MotorsUGV::active() const
{
    // if soft disarmed, motors not active
    if (!hal.util->get_soft_armed()) {
        return false;
    }

    // check throttle is active
    if (!is_zero(get_throttle())) {
        return true;
    }

    // skid-steering vehicles active when steering
    if (have_skid_steering() && !is_zero(get_steering())) {
        return true;
    }

    return false;
}

// returns true if the configured PWM type is digital and should have fixed endpoints
bool AP_MotorsUGV::is_digital_pwm_type() const
{
    switch (_pwm_type) {
    case PWMType::DSHOT150:
    case PWMType::DSHOT300:
    case PWMType::DSHOT600:
    case PWMType::DSHOT1200:
            return true;
    case PWMType::NORMAL:
    case PWMType::ONESHOT:
    case PWMType::ONESHOT125:
    case PWMType::BRUSHED_WITH_RELAY:
    case PWMType::BRUSHED_BIPOLAR:
            break;
    }
    return false;
}

/*
  handle delay on reversal for a throttle
 */
void AP_MotorsUGV::ReverseThrottle::output(SRV_Channel::Aux_servo_function_t function, float throttle, float delay)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (is_zero(throttle)) {
        // pass through, no change, don't update the last throttle
    } else if (throttle * last_throttle < 0 &&
               now_ms - last_output_ms < delay * 1000) {
        // sign change, add pause
        throttle = 0;
    } else {
        last_output_ms = now_ms;
        last_throttle = throttle;
    }
    SRV_Channels::set_output_scaled(function, throttle);
}

namespace AP {
    AP_MotorsUGV *motors_ugv()
    {
        return AP_MotorsUGV::get_singleton();
    }
}
