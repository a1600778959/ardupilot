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

/*
 *       RC_Channel.cpp - class for one RC channel input
 */

#include "RC_Channel_config.h"

#if AP_RC_CHANNEL_ENABLED

#include <stdlib.h>
#include <cmath>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>

#include "RC_Channel.h"
#include <GCS_MAVLink/GCS.h>

#include <AC_Avoidance/AC_Avoid.h>
#include <AP_Camera/AP_Camera.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GyroFFT/AP_GyroFFT.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_GPS/AP_GPS.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#define SWITCH_DEBOUNCE_TIME_MS  200

const AP_Param::GroupInfo RC_Channel::var_info[] = {
    // @Param: MIN
    // @DisplayName: RC min PWM
    // @Description: RC minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN",  1, RC_Channel, radio_min, 1100),

    // @Param: TRIM
    // @DisplayName: RC trim PWM
    // @Description: RC trim (neutral) PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRIM", 2, RC_Channel, radio_trim, 1500),

    // @Param: MAX
    // @DisplayName: RC max PWM
    // @Description: RC maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX",  3, RC_Channel, radio_max, 1900),

    // @Param: REVERSED
    // @DisplayName: RC reversed
    // @Description: Reverse channel input. Set to 0 for normal operation. Set to 1 to reverse this input channel.
    // @Values: 0:Normal,1:Reversed
    // @User: Advanced
    AP_GROUPINFO("REVERSED",  4, RC_Channel, reversed, 0),

    // @Param: DZ
    // @DisplayName: RC dead-zone
    // @Description: PWM dead zone in microseconds around trim or bottom
    // @Units: PWM
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("DZ",   5, RC_Channel, dead_zone, 0),

    // @Param: OPTION
    // @DisplayName: RC input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @Values{Rover}: 0:Do Nothing,4:RTL,7:Save WP,9:Camera Trigger,11:Fence Enable,16:AUTO Mode
    // @Values{Rover}: 24:Auto Mission Reset,28:Relay1 On/Off,30:Lost Rover Sound,31:Motor Emergency Stop
    // @Values{Rover}: 34:Relay2 On/Off,35:Relay3 On/Off,36:Relay4 On/Off,40:Proximity Avoidance Enable
    // @Values{Rover}: 42:SmartRTL Mode,46:RC Override Enable,50:LearnCruise Speed,51:Manual Mode
    // @Values{Rover}: 54:Hold Mode,55:Guided Mode,56:Loiter Mode,58:Clear Waypoints,62:Compass Learn
    // @Values{Rover}: 65:GPS Disable,66:Relay5 On/Off,67:Relay6 On/Off,72:Circle Mode,81:Disarm
    // @Values{Rover}: 90:EKF Source Set,100:KillIMU1,101:KillIMU2,102:Camera Mode Toggle,103:EKF Lane Switch
    // @Values{Rover}: 104:EKF Yaw Reset,105:GPS Disable Yaw,110:KillIMU3,112:SwitchExternalAHRS
    // @Values{Rover}: 153:ArmDisarm,155:Set steering trim to current servo and RC,162:FFT Tune
    // @Values{Rover}: 164:Pause Stream Logging,165:Arm/Emergency Motor Stop
    // @Values{Rover}: 166:Camera Record Video,167:Camera Zoom,168:Camera Manual Focus,169:Camera Auto Focus
    // @Values{Rover}: 171:Calibrate Compasses,172:Battery MPPT Enable,174:Camera Image Tracking
    // @Values{Rover}: 300:Scripting1,301:Scripting2,302:Scripting3,303:Scripting4,304:Scripting5,305:Scripting6,306:Scripting7,307:Scripting8
    // @User: Standard
    AP_GROUPINFO_FRAME("OPTION",  6, RC_Channel, option, 0, AP_PARAM_FRAME_ROVER),

    AP_GROUPINFO("BRAKE",  7, RC_Channel, brake_ch, 0),

    AP_GROUPEND
};


// constructor
RC_Channel::RC_Channel(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void RC_Channel::set_range(uint16_t high)
{
    type_in = ControlType::RANGE;
    high_in = high;
}

void RC_Channel::set_angle(uint16_t angle)
{
    type_in = ControlType::ANGLE;
    high_in = angle;
}

void RC_Channel::set_default_dead_zone(int16_t dzone)
{
    dead_zone.set_default(abs(dzone));
}

bool RC_Channel::get_reverse(void) const
{
    return bool(reversed.get());
}

// read input from hal.rcin or overrides
bool RC_Channel::update(void)
{
    if (has_override() && !rc().option_is_enabled(RC_Channels::Option::IGNORE_OVERRIDES)) {
        radio_in = override_value;
    } else if (rc().has_had_rc_receiver() && !rc().option_is_enabled(RC_Channels::Option::IGNORE_RECEIVER)) {
        radio_in = hal.rcin->read(ch_in);
    } else {
        return false;
    }

    if (type_in == ControlType::RANGE) {
        control_in = pwm_to_range();
    } else {
        // ControlType::ANGLE
        control_in = pwm_to_angle();
    }

    return true;
}

/*
  return the center stick position expressed as a control_in value
  used for thr_mid in copter
 */
int16_t RC_Channel::get_control_mid() const
{
    if (type_in == ControlType::RANGE) {
        int16_t r_in = (radio_min.get() + radio_max.get())/2;

        int16_t radio_trim_low  = radio_min + dead_zone;

        return (((int32_t)(high_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    } else {
        return 0;
    }
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
int16_t RC_Channel::pwm_to_angle_dz_trim(uint16_t _dead_zone, uint16_t _trim) const
{
    int16_t radio_trim_high = _trim + _dead_zone;
    int16_t radio_trim_low  = _trim - _dead_zone;

    int16_t reverse_mul = (reversed?-1:1);

    // don't allow out of range values
    int16_t r_in = constrain_int16(radio_in, radio_min.get(), radio_max.get());

    if (r_in > radio_trim_high && radio_max != radio_trim_high) {
        return reverse_mul * ((int32_t)high_in * (int32_t)(r_in - radio_trim_high)) / (int32_t)(radio_max  - radio_trim_high);
    } else if (r_in < radio_trim_low && radio_trim_low != radio_min) {
        return reverse_mul * ((int32_t)high_in * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_trim_low - radio_min);
    } else {
        return 0;
    }
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
int16_t RC_Channel::pwm_to_angle_dz(uint16_t _dead_zone) const
{
    return pwm_to_angle_dz_trim(_dead_zone, radio_trim);
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value
 */
int16_t RC_Channel::pwm_to_angle() const
{
    return pwm_to_angle_dz(dead_zone);
}


/*
  convert a pulse width modulation value to a value in the configured
  range, using the specified deadzone
 */
int16_t RC_Channel::pwm_to_range_dz(uint16_t _dead_zone) const
{
    int16_t r_in = constrain_int16(radio_in, radio_min.get(), radio_max.get());

    if (reversed) {
        r_in = radio_max.get() - (r_in - radio_min.get());
    }

    int16_t radio_trim_low  = radio_min + _dead_zone;

    if (r_in > radio_trim_low) {
        return (((int32_t)(high_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    }
    return 0;
}

/*
  convert a pulse width modulation value to a value in the configured
  range
 */
int16_t RC_Channel::pwm_to_range() const
{
    return pwm_to_range_dz(dead_zone);
}


int16_t RC_Channel::get_control_in_zero_dz(void) const
{
    if (type_in == ControlType::RANGE) {
        return pwm_to_range_dz(0);
    }
    return pwm_to_angle_dz(0);
}

// ------------------------------------------

float RC_Channel::norm_input() const
{
    float ret;
    int16_t reverse_mul = (reversed?-1:1);
    if (radio_in < radio_trim) {
        if (radio_min >= radio_trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
    } else {
        if (radio_max <= radio_trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
    }
    return constrain_float(ret, -1.0f, 1.0f);
}

float RC_Channel::norm_input_dz() const
{
    int16_t dz_min = radio_trim - dead_zone;
    int16_t dz_max = radio_trim + dead_zone;
    float ret;
    int16_t reverse_mul = (reversed?-1:1);
    if (radio_in < dz_min && dz_min > radio_min) {
        ret = reverse_mul * (float)(radio_in - dz_min) / (float)(dz_min - radio_min);
    } else if (radio_in > dz_max && radio_max > dz_max) {
        ret = reverse_mul * (float)(radio_in - dz_max) / (float)(radio_max  - dz_max);
    } else {
        ret = 0;
    }
    return constrain_float(ret, -1.0f, 1.0f);
}

// return a normalised input for a channel, in range -1 to 1,
// ignores trim and deadzone
float RC_Channel::norm_input_ignore_trim() const
{
    // sanity check min and max to avoid divide by zero
    if (radio_max <= radio_min) {
        return 0.0f;
    }
    const float ret = (reversed ? -2.0f : 2.0f) * (((float)(radio_in - radio_min) / (float)(radio_max - radio_min)) - 0.5f);
    return constrain_float(ret, -1.0f, 1.0f);
}

/*
  get percentage input from 0 to 100. This ignores the trim value.
 */
uint8_t RC_Channel::percent_input() const
{
    if (radio_in <= radio_min) {
        return reversed?100:0;
    }
    if (radio_in >= radio_max) {
        return reversed?0:100;
    }
    uint8_t ret = 100.0f * (radio_in - radio_min) / (float)(radio_max - radio_min);
    if (reversed) {
        ret = 100 - ret;
    }
    return ret;
}

/*
  return true if input is within deadzone of trim
*/
bool RC_Channel::in_trim_dz() const
{
    return is_bounded_int32(radio_in, radio_trim - dead_zone, radio_trim + dead_zone);
}


/*
   return trues if input is within deadzone of min
*/
bool RC_Channel::in_min_dz() const
{
    return radio_in < radio_min + dead_zone;
}

void RC_Channel::set_override(const uint16_t v, const uint32_t timestamp_ms)
{
    if (!rc().gcs_overrides_enabled()) {
        return;
    }

    last_override_time = timestamp_ms != 0 ? timestamp_ms : AP_HAL::millis();
    override_value = v;
    rc().new_override_received();
}

void RC_Channel::clear_override()
{
    last_override_time = 0;
    override_value = 0;
}

bool RC_Channel::has_override() const
{
    if (override_value == 0) {
        return false;
    }

    uint32_t override_timeout_ms;
    if (!rc().get_override_timeout_ms(override_timeout_ms)) {
        // timeouts are disabled
        return true;
    }

    if (override_timeout_ms == 0) {
        // overrides are explicitly disabled by a zero value
        return false;
    }

    return (AP_HAL::millis() - last_override_time < override_timeout_ms);
}

/*
  perform stick mixing on one channel
  This type of stick mixing reduces the influence of the auto
  controller as it increases the influence of the users stick input,
  allowing the user full deflection if needed
 */
float RC_Channel::stick_mixing(const float servo_in)
{
    float ch_inf = (float)(radio_in - radio_trim);
    ch_inf = fabsf(ch_inf);
    ch_inf = MIN(ch_inf, 400.0f);
    ch_inf = ((400.0f - ch_inf) / 400.0f);

    float servo_out = servo_in;
    servo_out *= ch_inf;
    servo_out += control_in;

    return servo_out;
}

//
// support for auxiliary switches:
//

void RC_Channel::reset_mode_switch()
{
    switch_state.current_position = -1;
    switch_state.debounce_position = -1;
    read_mode_switch();
}

// read a 6 position switch
bool RC_Channel::read_6pos_switch(int8_t& position)
{
    // calculate position of 6 pos switch
    const uint16_t pulsewidth = get_radio_in();
    if (pulsewidth <= RC_MIN_LIMIT_PWM || pulsewidth >= RC_MAX_LIMIT_PWM) {
        return false;  // This is an error condition
    }

    if (pulsewidth < 1231) {
        position = 0;
    } else if (pulsewidth < 1361) {
        position = 1;
    } else if (pulsewidth < 1491) {
        position = 2;
    } else if (pulsewidth < 1621) {
        position = 3;
    } else if (pulsewidth < 1750) {
        position = 4;
    } else {
        position = 5;
    }

    if (!debounce_completed(position)) {
        return false;
    }

    return true;
}

void RC_Channel::read_mode_switch()
{
    int8_t position;
    if (read_6pos_switch(position)) {
        // set flight mode and simple mode setting
        mode_switch_changed(modeswitch_pos_t(position));
    }
}

bool RC_Channel::debounce_completed(int8_t position)
{
    // switch change not detected
    if (switch_state.current_position == position) {
        // reset debouncing
        switch_state.debounce_position = position;
    } else {
        // switch change detected
        const uint32_t tnow_ms = AP_HAL::millis();

        // position not established yet
        if (switch_state.debounce_position != position) {
            switch_state.debounce_position = position;
            switch_state.last_edge_time_ms = tnow_ms;
        } else if (tnow_ms - switch_state.last_edge_time_ms >= SWITCH_DEBOUNCE_TIME_MS) {
            // position estabilished; debounce completed
            switch_state.current_position = position;
            return true;
        }
    }

    return false;
}

// init_aux_switch_function - initialize aux functions
void RC_Channel::init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    // init channel options
    switch (ch_option) {
    // the following functions do not need to be initialised:
#if AP_ARMING_ENABLED
    case AUX_FUNC::ARMDISARM:
    case AUX_FUNC::ARMDISARM_AIRMODE:
#endif
#if AP_BATTERY_ENABLED
    case AUX_FUNC::BATTERY_MPPT_ENABLE:
#endif
#if AP_CAMERA_ENABLED
    case AUX_FUNC::CAMERA_TRIGGER:
#endif
#if AP_MISSION_ENABLED
    case AUX_FUNC::CLEAR_WP:
#endif
    case AUX_FUNC::COMPASS_LEARN:
#if AP_ARMING_ENABLED
    case AUX_FUNC::DISARM:
#endif
    case AUX_FUNC::DO_NOTHING:
    case AUX_FUNC::LOST_VEHICLE_SOUND:
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
    case AUX_FUNC::RELAY:
    case AUX_FUNC::RELAY2:
    case AUX_FUNC::RELAY3:
    case AUX_FUNC::RELAY4:
    case AUX_FUNC::RELAY5:
    case AUX_FUNC::RELAY6:
#endif
#if AP_AHRS_ENABLED
    case AUX_FUNC::EKF_LANE_SWITCH:
    case AUX_FUNC::EKF_YAW_RESET:
#endif
#if AP_AHRS_ENABLED
    case AUX_FUNC::EKF_SOURCE_SET:
#endif
#if AP_SCRIPTING_ENABLED
    case AUX_FUNC::SCRIPTING_1:
    case AUX_FUNC::SCRIPTING_2:
    case AUX_FUNC::SCRIPTING_3:
    case AUX_FUNC::SCRIPTING_4:
    case AUX_FUNC::SCRIPTING_5:
    case AUX_FUNC::SCRIPTING_6:
    case AUX_FUNC::SCRIPTING_7:
    case AUX_FUNC::SCRIPTING_8:
#endif
    case AUX_FUNC::TURBINE_START:
#if COMPASS_CAL_ENABLED
    case AUX_FUNC::MAG_CAL:
#endif
#if AP_CAMERA_ENABLED
    case AUX_FUNC::CAMERA_IMAGE_TRACKING:
#endif
        break;
    case AUX_FUNC::AVOID_PROXIMITY:
#if AP_FENCE_ENABLED
    case AUX_FUNC::FENCE:
#endif
#if AP_GPS_ENABLED
    case AUX_FUNC::GPS_DISABLE:
    case AUX_FUNC::GPS_DISABLE_YAW:
#endif
#if AP_INERTIALSENSOR_KILL_IMU_ENABLED
    case AUX_FUNC::KILL_IMU1:
    case AUX_FUNC::KILL_IMU2:
    case AUX_FUNC::KILL_IMU3:
#endif
#if AP_MISSION_ENABLED
    case AUX_FUNC::MISSION_RESET:
#endif
    case AUX_FUNC::MOTOR_ESTOP:
    case AUX_FUNC::RC_OVERRIDE_ENABLE:
    case AUX_FUNC::FFT_NOTCH_TUNE:
#if HAL_LOGGING_ENABLED
    case AUX_FUNC::LOG_PAUSE:
#endif
    case AUX_FUNC::ARM_EMERGENCY_STOP:
#if AP_CAMERA_ENABLED
    case AUX_FUNC::CAMERA_REC_VIDEO:
    case AUX_FUNC::CAMERA_ZOOM:
    case AUX_FUNC::CAMERA_MANUAL_FOCUS:
    case AUX_FUNC::CAMERA_AUTO_FOCUS:
#endif
#if AP_AHRS_ENABLED
    case AUX_FUNC::AHRS_TYPE:
        run_aux_function(ch_option, ch_flag, AuxFuncTriggerSource::INIT);
        break;
#endif
    default:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failed to init: RC%u_OPTION: %u\n",
                        (unsigned)(this->ch_in+1), (unsigned)ch_option);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_BoardConfig::config_error("Failed to init: RC%u_OPTION: %u",
                                     (unsigned)(this->ch_in+1), (unsigned)ch_option);
#endif
        break;
    }
}

#if AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED

const RC_Channel::LookupTable RC_Channel::lookuptable[] = {
#if AP_MISSION_ENABLED
    { AUX_FUNC::SAVE_WP,"SaveWaypoint"},
#endif
#if AP_CAMERA_ENABLED
    { AUX_FUNC::CAMERA_TRIGGER,"CameraTrigger"},
#endif
#if AP_RANGEFINDER_ENABLED
    { AUX_FUNC::RANGEFINDER,"Rangefinder"},
#endif
#if AP_FENCE_ENABLED
    { AUX_FUNC::FENCE,"Fence"},
#endif
#if AP_MISSION_ENABLED
    { AUX_FUNC::MISSION_RESET,"MissionReset"},
#endif
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
    { AUX_FUNC::RELAY,"Relay1"},
#endif
    { AUX_FUNC::MOTOR_ESTOP,"MotorEStop"},
    { AUX_FUNC::MOTOR_INTERLOCK,"MotorInterlock"},
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
    { AUX_FUNC::RELAY2,"Relay2"},
    { AUX_FUNC::RELAY3,"Relay3"},
    { AUX_FUNC::RELAY4,"Relay4"},
#endif
    { AUX_FUNC::PRECISION_LOITER,"PrecisionLoiter"},
    { AUX_FUNC::AVOID_PROXIMITY,"AvoidProximity"},
#if AP_MISSION_ENABLED
    { AUX_FUNC::CLEAR_WP,"ClearWaypoint"},
#endif
    { AUX_FUNC::COMPASS_LEARN,"CompassLearn"},
#if AP_GPS_ENABLED
    { AUX_FUNC::GPS_DISABLE,"GPSDisable"},
    { AUX_FUNC::GPS_DISABLE_YAW,"GPSDisableYaw"},
#endif
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
    { AUX_FUNC::RELAY5,"Relay5"},
    { AUX_FUNC::RELAY6,"Relay6"},
#endif
    { AUX_FUNC::SURFACE_TRACKING,"SurfaceTracking"},
    { AUX_FUNC::AIRMODE, "AirMode"},
#if AP_CAMERA_ENABLED
    { AUX_FUNC::CAM_MODE_TOGGLE,"CamModeToggle"},
#endif
#if AP_BATTERY_ENABLED
    { AUX_FUNC::BATTERY_MPPT_ENABLE,"Battery MPPT Enable"},
#endif
    { AUX_FUNC::EMERGENCY_LANDING_EN, "Emergency Landing"},
    { AUX_FUNC::WEATHER_VANE_ENABLE, "Weathervane"},
    { AUX_FUNC::TURBINE_START, "Turbine Start"},
    { AUX_FUNC::FFT_NOTCH_TUNE, "FFT Notch Tuning"},
    { AUX_FUNC::PATROL, "Patrol"},
    { AUX_FUNC::PATROL_DIST_INC, "PatrolDistInc"},
    { AUX_FUNC::PATROL_DIST_DEC, "PatrolDistDec"},
#if HAL_LOGGING_ENABLED
    { AUX_FUNC::LOG_PAUSE, "Pause Stream Logging"},
#endif
#if AP_CAMERA_ENABLED
    { AUX_FUNC::CAMERA_REC_VIDEO, "Camera Record Video"},
    { AUX_FUNC::CAMERA_ZOOM, "Camera Zoom"},
    { AUX_FUNC::CAMERA_MANUAL_FOCUS, "Camera Manual Focus"},
    { AUX_FUNC::CAMERA_AUTO_FOCUS, "Camera Auto Focus"},
    { AUX_FUNC::CAMERA_IMAGE_TRACKING, "Camera Image Tracking"},
#endif
};

/* lookup the announcement for switch change */
const char *RC_Channel::string_for_aux_function(AUX_FUNC function) const
{
    for (const struct LookupTable &entry : lookuptable) {
        if (entry.option == function) {
            return entry.announcement;
        }
    }
    return nullptr;
}

/* find string for postion */
const char *RC_Channel::string_for_aux_pos(AuxSwitchPos pos) const
{
    switch (pos) {
    case AuxSwitchPos::HIGH:
        return "HIGH";
    case AuxSwitchPos::MIDDLE:
        return "MIDDLE";
    case AuxSwitchPos::LOW:
        return "LOW";
    }
    return "";
}

#endif // AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED

/*
  read an aux channel. Return true if a switch has changed
 */
bool RC_Channel::read_aux()
{
    const AUX_FUNC _option = (AUX_FUNC)option.get();
    if (_option == AUX_FUNC::DO_NOTHING) {
        // may wish to add special cases for other "AUXSW" things
        // here e.g. RCMAP_ROLL etc once they become options
        return false;
    }

    AuxSwitchPos new_position;
    if (!read_3pos_switch(new_position)) {
        return false;
    }

    if (!switch_state.initialised) {
        switch_state.initialised = true;
        if (init_position_on_first_radio_read((AUX_FUNC)option.get())) {
            switch_state.current_position = (int8_t)new_position;
            switch_state.debounce_position = (int8_t)new_position;
        }
    }

    if (!debounce_completed((int8_t)new_position)) {
        return false;
    }

#if AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED
    // announce the change to the GCS:
    const char *aux_string = string_for_aux_function(_option);
    if (aux_string != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RC%i: %s %s", ch_in+1, aux_string, string_for_aux_pos(new_position));
    }
#endif

    // debounced; undertake the action:
    run_aux_function(_option, new_position, AuxFuncTriggerSource::RC);
    return true;
}

// returns true if the first time we successfully read the
// channel's three-position-switch position we should record that
// position as the current position *without* executing the
// associated auxiliary function.  e.g. do not attempt to arm a
// vehicle when the user turns on their transmitter with the arm
// switch high!
bool RC_Channel::init_position_on_first_radio_read(AUX_FUNC func) const
{
    switch (func) {
#if AP_ARMING_ENABLED
    case AUX_FUNC::ARMDISARM_AIRMODE:
    case AUX_FUNC::ARMDISARM:
    case AUX_FUNC::ARM_EMERGENCY_STOP:
#endif
        // we do not want to process 
        return true;
    default:
        return false;
    }
}

void RC_Channel::do_aux_function_armdisarm(const AuxSwitchPos ch_flag)
{
    // arm or disarm the vehicle
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        AP::arming().arm(AP_Arming::Method::AUXSWITCH, true);
        do_aux_function_relay(brake_ch, true);
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW:
        AP::arming().disarm(AP_Arming::Method::AUXSWITCH);
        do_aux_function_relay(brake_ch, false);
        break;
    }
}

void RC_Channel::do_aux_function_avoid_proximity(const AuxSwitchPos ch_flag)
{
#if AP_AVOIDANCE_ENABLED
    AC_Avoid *avoid = AP::ac_avoid();
    if (avoid == nullptr) {
        return;
    }

    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        avoid->proximity_avoidance_enable(true);
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW:
        avoid->proximity_avoidance_enable(false);
        break;
    }
#endif // !APM_BUILD_ArduPlane
}

#if AP_CAMERA_ENABLED
void RC_Channel::do_aux_function_camera_trigger(const AuxSwitchPos ch_flag)
{
    if (ch_flag == AuxSwitchPos::HIGH) {
        AP_Camera *camera = AP::camera();
        if (camera == nullptr) {
            return;
        }
        camera->take_picture();
    }
}

bool RC_Channel::do_aux_function_record_video(const AuxSwitchPos ch_flag)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return false;
    }
    return camera->record_video(ch_flag == AuxSwitchPos::HIGH);
}

bool RC_Channel::do_aux_function_camera_zoom(const AuxSwitchPos ch_flag)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return false;
    }
    int8_t zoom_step = 0;   // zoom out = -1, hold = 0, zoom in = 1
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        zoom_step = 1;  // zoom in
        break;
    case AuxSwitchPos::MIDDLE:
        zoom_step = 0;  // zoom hold
        break;
    case AuxSwitchPos::LOW:
        zoom_step = -1; // zoom out
        break;
    }
    return camera->set_zoom(ZoomType::RATE, zoom_step);
}

bool RC_Channel::do_aux_function_camera_manual_focus(const AuxSwitchPos ch_flag)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return false;
    }
    int8_t focus_step = 0;  // focus in = -1, focus hold = 0, focus out = 1
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        // wide shot, focus out
        focus_step = 1;
        break;
    case AuxSwitchPos::MIDDLE:
        focus_step = 0;
        break;
    case AuxSwitchPos::LOW:
        // close shot, focus in
        focus_step = -1;
        break;
    }
    return camera->set_focus(FocusType::RATE, focus_step) == SetFocusResult::ACCEPTED;
}

bool RC_Channel::do_aux_function_camera_auto_focus(const AuxSwitchPos ch_flag)
{
    if (ch_flag == AuxSwitchPos::HIGH) {
        AP_Camera *camera = AP::camera();
        if (camera == nullptr) {
            return false;
        }
        return camera->set_focus(FocusType::AUTO, 0) == SetFocusResult::ACCEPTED;
    }
    return false;
}

bool RC_Channel::do_aux_function_camera_image_tracking(const AuxSwitchPos ch_flag)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return false;
    }
    // High position enables tracking a POINT in middle of image
    // Low or Mediums disables tracking.  (0.5,0.5) is still passed in but ignored
    return camera->set_tracking(ch_flag == AuxSwitchPos::HIGH ? TrackingType::TRK_POINT : TrackingType::TRK_NONE, Vector2f{0.5, 0.5}, Vector2f{});
}

bool RC_Channel::do_aux_function_camera_lens(const AuxSwitchPos ch_flag)
{
    return false;
}
#endif // AP_CAMERA_ENABLED

#if AP_FENCE_ENABLED
// enable or disable the fence
void RC_Channel::do_aux_function_fence(const AuxSwitchPos ch_flag)
{
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    fence->enable_configured(ch_flag == AuxSwitchPos::HIGH);
}
#endif

#if AP_MISSION_ENABLED
void RC_Channel::do_aux_function_clear_wp(const AuxSwitchPos ch_flag)
{
    if (ch_flag == AuxSwitchPos::HIGH) {
        AP_Mission *mission = AP::mission();
        if (mission == nullptr) {
            return;
        }
        mission->clear();
    }
}
#endif  // AP_MISSION_ENABLED

#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
void RC_Channel::do_aux_function_relay(const uint8_t relay, bool val)
{
    AP_ServoRelayEvents *servorelayevents = AP::servorelayevents();
    if (servorelayevents == nullptr) {
        return;
    }
    servorelayevents->do_set_relay(relay, val);
}
#endif

void RC_Channel::do_aux_function_lost_vehicle_sound(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        AP_Notify::flags.vehicle_lost = true;
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW:
        AP_Notify::flags.vehicle_lost = false;
        break;
    }
}

void RC_Channel::do_aux_function_rc_override_enable(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH: {
        rc().set_gcs_overrides_enabled(true);
        break;
    }
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW: {
        rc().set_gcs_overrides_enabled(false);
        break;
    }
    }
}

#if AP_MISSION_ENABLED
void RC_Channel::do_aux_function_mission_reset(const AuxSwitchPos ch_flag)
{
    if (ch_flag != AuxSwitchPos::HIGH) {
        return;
    }
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return;
    }
    mission->reset();
}
#endif

void RC_Channel::do_aux_function_fft_notch_tune(const AuxSwitchPos ch_flag)
{
#if HAL_GYROFFT_ENABLED
    AP_GyroFFT *fft = AP::fft();
    if (fft == nullptr) {
        return;
    }

    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        fft->start_notch_tune();
        break;
    case AuxSwitchPos::MIDDLE:
    case AuxSwitchPos::LOW:
        fft->stop_notch_tune();
        break;
    }
#endif
}

bool RC_Channel::run_aux_function(AUX_FUNC ch_option, AuxSwitchPos pos, AuxFuncTriggerSource source)
{
#if AP_SCRIPTING_ENABLED
    rc().set_aux_cached(ch_option, pos);
#endif
    const bool ret = do_aux_function(ch_option, pos);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: AUXF
    // @Description: Auxiliary function invocation information
    // @Field: TimeUS: Time since system startup
    // @Field: function: ID of triggered function
    // @FieldValueEnum: function: RC_Channel::AUX_FUNC
    // @Field: pos: switch position when function triggered
    // @FieldValueEnum: pos: RC_Channel::AuxSwitchPos
    // @Field: source: source of auxiliary function invocation
    // @FieldValueEnum: source: RC_Channel::AuxFuncTriggerSource
    // @Field: result: true if function was successful
    AP::logger().Write(
        "AUXF",
        "TimeUS,function,pos,source,result",
        "s#---",
        "F----",
        "QHBBB",
        AP_HAL::micros64(),
        uint16_t(ch_option),
        uint8_t(pos),
        uint8_t(source),
        uint8_t(ret)
    );
#endif

    return ret;
}

bool RC_Channel::do_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    switch (ch_option) {
#if AP_FENCE_ENABLED
    case AUX_FUNC::FENCE:
        do_aux_function_fence(ch_flag);
        break;
#endif

    case AUX_FUNC::RC_OVERRIDE_ENABLE:
        // Allow or disallow RC_Override
        do_aux_function_rc_override_enable(ch_flag);
        break;

    case AUX_FUNC::AVOID_PROXIMITY:
        do_aux_function_avoid_proximity(ch_flag);
        break;

#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
    case AUX_FUNC::RELAY:
        do_aux_function_relay(0, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY2:
        do_aux_function_relay(1, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY3:
        do_aux_function_relay(2, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY4:
        do_aux_function_relay(3, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY5:
        do_aux_function_relay(4, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY6:
        do_aux_function_relay(5, ch_flag == AuxSwitchPos::HIGH);
        break;
#endif  // AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED

#if AP_MISSION_ENABLED
    case AUX_FUNC::CLEAR_WP:
        do_aux_function_clear_wp(ch_flag);
        break;
    case AUX_FUNC::MISSION_RESET:
        do_aux_function_mission_reset(ch_flag);
        break;
#endif

    case AUX_FUNC::FFT_NOTCH_TUNE:
        do_aux_function_fft_notch_tune(ch_flag);
        break;

#if AP_BATTERY_ENABLED
    case AUX_FUNC::BATTERY_MPPT_ENABLE:
        if (ch_flag != AuxSwitchPos::MIDDLE) {
            AP::battery().MPPT_set_powered_state_to_all(ch_flag == AuxSwitchPos::HIGH);
        }
        break;
#endif

    case AUX_FUNC::LOST_VEHICLE_SOUND:
        do_aux_function_lost_vehicle_sound(ch_flag);
        break;

#if AP_ARMING_ENABLED
    case AUX_FUNC::ARMDISARM:
        do_aux_function_armdisarm(ch_flag);
        break;

    case AUX_FUNC::DISARM:
        if (ch_flag == AuxSwitchPos::HIGH) {
            AP::arming().disarm(AP_Arming::Method::AUXSWITCH);
        }
        break;
#endif

    case AUX_FUNC::COMPASS_LEARN:
        if (ch_flag == AuxSwitchPos::HIGH) {
            Compass &compass = AP::compass();
            compass.set_learn_type(Compass::LEARN_INFLIGHT, false);
        }
        break;
#if AP_GPS_ENABLED
    case AUX_FUNC::GPS_DISABLE:
        AP::gps().force_disable(ch_flag == AuxSwitchPos::HIGH);
#if HAL_EXTERNAL_AHRS_ENABLED
        AP::externalAHRS().set_gnss_disable(ch_flag == AuxSwitchPos::HIGH);
#endif
        break;

    case AUX_FUNC::GPS_DISABLE_YAW:
        AP::gps().set_force_disable_yaw(ch_flag == AuxSwitchPos::HIGH);
        break;
#endif  // AP_GPS_ENABLED

    case AUX_FUNC::MOTOR_ESTOP:
        switch (ch_flag) {
        case AuxSwitchPos::HIGH: {
            SRV_Channels::set_emergency_stop(true);
            break;
        }
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::LOW: {
            SRV_Channels::set_emergency_stop(false);
            break;
        }
        }
        break;

    case AUX_FUNC::EKF_SOURCE_SET: {
        AP_NavEKF_Source::SourceSetSelection source_set = AP_NavEKF_Source::SourceSetSelection::PRIMARY;
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            // low switches to primary source
            source_set = AP_NavEKF_Source::SourceSetSelection::PRIMARY;
            break;
        case AuxSwitchPos::MIDDLE:
            // middle switches to secondary source
            source_set = AP_NavEKF_Source::SourceSetSelection::SECONDARY;
            break;
        case AuxSwitchPos::HIGH:
            // high switches to tertiary source
            source_set = AP_NavEKF_Source::SourceSetSelection::TERTIARY;
            break;
        }
        AP::ahrs().set_posvelyaw_source_set(source_set);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Using EKF Source Set %u", uint8_t(source_set)+1);
        break;
    }

#if AP_INERTIALSENSOR_KILL_IMU_ENABLED
    case AUX_FUNC::KILL_IMU1:
        AP::ins().kill_imu(0, ch_flag == AuxSwitchPos::HIGH);
        break;

    case AUX_FUNC::KILL_IMU2:
        AP::ins().kill_imu(1, ch_flag == AuxSwitchPos::HIGH);
        break;

    case AUX_FUNC::KILL_IMU3:
        AP::ins().kill_imu(2, ch_flag == AuxSwitchPos::HIGH);
        break;
#endif  // AP_INERTIALSENSOR_KILL_IMU_ENABLED

#if AP_CAMERA_ENABLED
    case AUX_FUNC::CAMERA_TRIGGER:
        do_aux_function_camera_trigger(ch_flag);
        break;

    case AUX_FUNC::CAM_MODE_TOGGLE: {
        // Momentary switch to for cycling camera modes
        AP_Camera *camera = AP_Camera::get_singleton();
        if (camera == nullptr) {
            break;
        }
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            // nothing
            break;
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::HIGH:
            camera->cam_mode_toggle();
            break;
        }
        break;
    }
    case AUX_FUNC::CAMERA_REC_VIDEO:
        return do_aux_function_record_video(ch_flag);

    case AUX_FUNC::CAMERA_ZOOM:
        return do_aux_function_camera_zoom(ch_flag);

    case AUX_FUNC::CAMERA_MANUAL_FOCUS:
        return do_aux_function_camera_manual_focus(ch_flag);

    case AUX_FUNC::CAMERA_AUTO_FOCUS:
        return do_aux_function_camera_auto_focus(ch_flag);

    case AUX_FUNC::CAMERA_IMAGE_TRACKING:
        return do_aux_function_camera_image_tracking(ch_flag);

#endif // AP_CAMERA_ENABLED

#if HAL_LOGGING_ENABLED
    case AUX_FUNC::LOG_PAUSE: {
        AP_Logger *logger = AP_Logger::get_singleton();
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            logger->log_pause(false);
            break;
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::HIGH:
            logger->log_pause(true);
            break;
        }
        break;
    }
#endif

#if COMPASS_CAL_ENABLED
    case AUX_FUNC::MAG_CAL: {
        Compass &compass = AP::compass();
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            compass.cancel_calibration_all();
            break;
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::HIGH:
            if (!hal.util->get_soft_armed()) {
                const bool retry = true;
                const bool autosave = true;
                const float delay = 5.0;
                const bool autoreboot = false;
                compass.start_calibration_all(retry, autosave, delay, autoreboot);
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Disarm to allow compass calibration");
            }
            break;
        }
        break;
    }
#endif

    case AUX_FUNC::ARM_EMERGENCY_STOP: {
        switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            // request arm, disable emergency motor stop
            SRV_Channels::set_emergency_stop(false);
            AP::arming().arm(AP_Arming::Method::AUXSWITCH, true);
            break;
        case AuxSwitchPos::MIDDLE:
            // disable emergency motor stop
            SRV_Channels::set_emergency_stop(false);
            break;
        case AuxSwitchPos::LOW:
            // enable emergency motor stop
            SRV_Channels::set_emergency_stop(true);
            break;
        }
        break;
    }

#if AP_AHRS_ENABLED
    case AUX_FUNC::EKF_LANE_SWITCH:
        // used to test emergency lane switch
        AP::ahrs().check_lane_switch();
        break;

    case AUX_FUNC::EKF_YAW_RESET:
        // used to test emergency yaw reset
        AP::ahrs().request_yaw_reset();
        break;

    case AUX_FUNC::AHRS_TYPE: {
#if HAL_NAVEKF3_AVAILABLE && HAL_EXTERNAL_AHRS_ENABLED
        AP::ahrs().set_ekf_type(ch_flag==AuxSwitchPos::HIGH? AP_AHRS::EKFType::EXTERNAL : AP_AHRS::EKFType::THREE);
#endif
        break;
    }
#endif  // AP_AHRS_ENABLED

#if AP_SCRIPTING_ENABLED
    case AUX_FUNC::SCRIPTING_1:
    case AUX_FUNC::SCRIPTING_2:
    case AUX_FUNC::SCRIPTING_3:
    case AUX_FUNC::SCRIPTING_4:
    case AUX_FUNC::SCRIPTING_5:
    case AUX_FUNC::SCRIPTING_6:
    case AUX_FUNC::SCRIPTING_7:
    case AUX_FUNC::SCRIPTING_8:
#endif
        break;

    default:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Invalid channel option (%u)", (unsigned int)ch_option);
        return false;
    }

    return true;
}

void RC_Channel::init_aux()
{
    AuxSwitchPos position;
    if (!read_3pos_switch(position)) {
        position = AuxSwitchPos::LOW;
    }
    init_aux_function((AUX_FUNC)option.get(), position);
}

// read_3pos_switch
bool RC_Channel::read_3pos_switch(RC_Channel::AuxSwitchPos &ret) const
{
    const uint16_t in = get_radio_in();
    if (in <= RC_MIN_LIMIT_PWM || in >= RC_MAX_LIMIT_PWM) {
        return false;
    }

    // switch is reversed if 'reversed' option set on channel and switches reverse is allowed by RC_OPTIONS
    bool switch_reversed = reversed && rc().option_is_enabled(RC_Channels::Option::ALLOW_SWITCH_REV);

    if (in < AUX_SWITCH_PWM_TRIGGER_LOW) {
        ret = switch_reversed ? AuxSwitchPos::HIGH : AuxSwitchPos::LOW;
    } else if (in > AUX_SWITCH_PWM_TRIGGER_HIGH) {
        ret = switch_reversed ? AuxSwitchPos::LOW : AuxSwitchPos::HIGH;
    } else {
        ret = AuxSwitchPos::MIDDLE;
    }
    return true;
}

// return switch position value as LOW, MIDDLE, HIGH
// if reading the switch fails then it returns LOW
RC_Channel::AuxSwitchPos RC_Channel::get_aux_switch_pos() const
{
    AuxSwitchPos position = AuxSwitchPos::LOW;
    UNUSED_RESULT(read_3pos_switch(position));

    return position;
}

// return stick gesture pos as LOW, MIDDLE, HIGH
// this function uses different threshold values to RC_Chanel::get_aux_switch_pos()
// to avoid glitching on the stick travel and also always honours channel reversal
RC_Channel::AuxSwitchPos RC_Channel::get_stick_gesture_pos() const
{
    const uint16_t in = get_radio_in();
    if (in <= 900 || in >= 2200) {
        return RC_Channel::AuxSwitchPos::LOW;
    }

    // switch is reversed if 'reversed' option set on channel and switches reverse is allowed by RC_OPTIONS
    bool switch_reversed = get_reverse();

    if (in < RC_Channel::AUX_PWM_TRIGGER_LOW) {
        return switch_reversed ? RC_Channel::AuxSwitchPos::HIGH : RC_Channel::AuxSwitchPos::LOW;
    }
    if (in > RC_Channel::AUX_PWM_TRIGGER_HIGH) {
        return switch_reversed ? RC_Channel::AuxSwitchPos::LOW : RC_Channel::AuxSwitchPos::HIGH;
    }
    return RC_Channel::AuxSwitchPos::MIDDLE;
}

RC_Channel *RC_Channels::find_channel_for_option(const RC_Channel::AUX_FUNC option)
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            // odd?
            continue;
        }
        if ((RC_Channel::AUX_FUNC)c->option.get() == option) {
            return c;
        }
    }
    return nullptr;
}

// duplicate_options_exist - returns true if any options are duplicated
bool RC_Channels::duplicate_options_exist()
{
    Bitmask<(uint16_t)RC_Channel::AUX_FUNC::AUX_FUNCTION_MAX> used_auxsw_options;
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        const RC_Channel *c = channel(i);
        if (c == nullptr) {
            // odd?
            continue;
        }
        const uint16_t option = c->option.get();
        if (option == (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING) {
            continue;
        }
        if (option >= used_auxsw_options.size()) {
            continue;
        }
        if (used_auxsw_options.get(option)) {
            return true;
        }
        used_auxsw_options.set(option);
    }
    return false;
}

// convert option parameter from old to new
void RC_Channels::convert_options(const RC_Channel::AUX_FUNC old_option, const RC_Channel::AUX_FUNC new_option)
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            // odd?
            continue;
        }
        if ((RC_Channel::AUX_FUNC)c->option.get() == old_option) {
            c->option.set_and_save((int16_t)new_option);
        }
    }
}

#endif  // AP_RC_CHANNEL_ENABLED
