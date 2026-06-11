#include "Rover.h"

#include "RC_Channel.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Rover
#define RC_CHANNEL_SUBCLASS RC_Channel_Rover

#include <RC_Channel/RC_Channels_VarInfo.h>

int8_t RC_Channels_Rover::flight_mode_channel_number() const
{
    return rover.g.mode_channel;
}

void RC_Channel_Rover::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || (uint8_t)new_pos > rover.num_modes) {
        // should not have been called
        return;
    }

    rover.set_mode((Mode::Number)rover.modes[new_pos].get(), ModeReason::RC_COMMAND);
}

// init_aux_switch_function - initialize aux functions
void RC_Channel_Rover::init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    // init channel options
    switch (ch_option) {
    // the following functions do not need initialising:
    case AUX_FUNC::AUTO:
    case AUX_FUNC::CIRCLE:
    case AUX_FUNC::GUIDED:
    case AUX_FUNC::HOLD:
    case AUX_FUNC::LEARN_CRUISE:
    case AUX_FUNC::LOITER:
    case AUX_FUNC::MANUAL:
    case AUX_FUNC::RTL:
    case AUX_FUNC::TRIM_TO_CURRENT_SERVO_RC:
    case AUX_FUNC::SAVE_WP:
    case AUX_FUNC::SMART_RTL:
        break;
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}


bool RC_Channels_Rover::in_rc_failsafe() const
{
    return rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE;
}

bool RC_Channels_Rover::has_valid_input() const
{
    if (in_rc_failsafe()) {
        return false;
    }
    return true;
}

RC_Channel * RC_Channels_Rover::get_arming_channel(void) const
{
    return rover.channel_steer;
}

void RC_Channel_Rover::do_aux_function_change_mode(Mode &mode,
        const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        rover.set_mode(mode, ModeReason::AUX_FUNCTION);
        break;
    case AuxSwitchPos::MIDDLE:
        // do nothing
        break;
    case AuxSwitchPos::LOW:
        if (rover.control_mode == &mode) {
            rc().reset_mode_switch();
        }
    }
}

void RC_Channel_Rover::add_waypoint_for_current_loc()
{
    // create new mission command
    AP_Mission::Mission_Command cmd = {};

    // set new waypoint to current location
    cmd.content.location = rover.current_loc;

    // make the new command to a waypoint
    cmd.id = MAV_CMD_NAV_WAYPOINT;

    // save command
    if (rover.mode_auto.mission.add_cmd(cmd)) {
        hal.console->printf("Added waypoint %u", (unsigned)rover.mode_auto.mission.num_commands());
    }
}

bool RC_Channel_Rover::do_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    switch (ch_option) {
    case AUX_FUNC::DO_NOTHING:
        break;
    case AUX_FUNC::SAVE_WP:
        if (ch_flag == AuxSwitchPos::HIGH) {
            // do nothing if in AUTO mode
            if (rover.control_mode == &rover.mode_auto) {
                break;
            }

            // if disarmed clear mission and set home to current location
            if (!rover.arming.is_armed()) {
                rover.mode_auto.mission.clear();
                gcs().send_text(MAV_SEVERITY_NOTICE, "SaveWP: Mission cleared!");
                if (!rover.set_home_to_current_location(false)) {
                    // ignored
                }
                break;
            }

            // record the waypoint if not in auto mode
            if (rover.control_mode != &rover.mode_auto) {
                if (rover.mode_auto.mission.num_commands() == 0) {
                    // add a home location....
                    add_waypoint_for_current_loc();
                }
                add_waypoint_for_current_loc();
            }
        }
        break;

    // learn cruise speed and throttle
    case AUX_FUNC::LEARN_CRUISE:
        if (ch_flag == AuxSwitchPos::HIGH) {
            rover.cruise_learn_start();
        }
        break;

    // set mode to Manual
    case AUX_FUNC::MANUAL:
        do_aux_function_change_mode(rover.mode_manual, ch_flag);
        break;
        
    // set mode to Hold
    case AUX_FUNC::HOLD:
        do_aux_function_change_mode(rover.mode_hold, ch_flag);
        break;

    // set mode to Auto
    case AUX_FUNC::AUTO:
        do_aux_function_change_mode(rover.mode_auto, ch_flag);
        break;

    // set mode to RTL
    case AUX_FUNC::RTL:
        do_aux_function_change_mode(rover.mode_rtl, ch_flag);
        break;

    // set mode to SmartRTL
    case AUX_FUNC::SMART_RTL:
        do_aux_function_change_mode(rover.mode_smartrtl, ch_flag);
        break;

    // set mode to Guided
    case AUX_FUNC::GUIDED:
        do_aux_function_change_mode(rover.mode_guided, ch_flag);
        break;

    // Set mode to LOITER
    case AUX_FUNC::LOITER:
        do_aux_function_change_mode(rover.mode_loiter, ch_flag);
        break;

    case AUX_FUNC::CIRCLE:
        do_aux_function_change_mode(rover.g2.mode_circle, ch_flag);
        break;

    // save steering trim
    case AUX_FUNC::TRIM_TO_CURRENT_SERVO_RC:
        if (!rover.g2.motors.have_skid_steering() && rover.arming.is_armed() &&
            (rover.control_mode != &rover.mode_loiter)
            && (rover.control_mode != &rover.mode_hold) && ch_flag == AuxSwitchPos::HIGH) {
            SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_steering);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Steering trim saved!");
        }
        break;

    default:
        return RC_Channel::do_aux_function(ch_option, ch_flag);

    }

    return true;
}
