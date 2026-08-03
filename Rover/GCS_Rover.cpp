#include "GCS_Rover.h"

#include "Rover.h"

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

#define MAV_STREAM_CONVERSION(old_key, channel, element, suffix) \
    { Parameters::old_key, element, AP_PARAM_INT16, "MAV" #channel "_" #suffix }

#define MAV_STREAM_CONVERSIONS(name, old_key, channel) \
    static const AP_Param::ConversionInfo name[] = { \
        MAV_STREAM_CONVERSION(old_key, channel, 0, RAW_SENS), \
        MAV_STREAM_CONVERSION(old_key, channel, 1, EXT_STAT), \
        MAV_STREAM_CONVERSION(old_key, channel, 2, RC_CHAN), \
        MAV_STREAM_CONVERSION(old_key, channel, 3, RAW_CTRL), \
        MAV_STREAM_CONVERSION(old_key, channel, 4, POSITION), \
        MAV_STREAM_CONVERSION(old_key, channel, 5, EXTRA1), \
        MAV_STREAM_CONVERSION(old_key, channel, 6, EXTRA2), \
        MAV_STREAM_CONVERSION(old_key, channel, 7, EXTRA3), \
        MAV_STREAM_CONVERSION(old_key, channel, 8, PARAMS), \
    }

MAV_STREAM_CONVERSIONS(mav1_stream_conversions, k_param_gcs0_old, 1);
MAV_STREAM_CONVERSIONS(mav2_stream_conversions, k_param_gcs1_old, 2);
MAV_STREAM_CONVERSIONS(mav3_stream_conversions, k_param_gcs2_old, 3);
MAV_STREAM_CONVERSIONS(mav4_stream_conversions, k_param_gcs3_old, 4);
MAV_STREAM_CONVERSIONS(mav5_stream_conversions, k_param_gcs4_old, 5);
MAV_STREAM_CONVERSIONS(mav6_stream_conversions, k_param_gcs5_old, 6);
MAV_STREAM_CONVERSIONS(mav7_stream_conversions, k_param_gcs6_old, 7);

#undef MAV_STREAM_CONVERSIONS
#undef MAV_STREAM_CONVERSION

void GCS_Rover::convert_gcs_mavlink_backend_parameters(uint8_t instance)
{
    // MAVn is the nth runtime MAVLink backend, not SERIALn. SerialManager
    // supplies backends in ascending configured SERIALx order (then any
    // registered virtual ports), and protocol 1/2/43 share this ordering.
    switch (instance) {
    case 0:
        AP_Param::convert_old_parameters(mav1_stream_conversions, ARRAY_SIZE(mav1_stream_conversions));
        break;
    case 1:
        AP_Param::convert_old_parameters(mav2_stream_conversions, ARRAY_SIZE(mav2_stream_conversions));
        break;
    case 2:
        AP_Param::convert_old_parameters(mav3_stream_conversions, ARRAY_SIZE(mav3_stream_conversions));
        break;
    case 3:
        AP_Param::convert_old_parameters(mav4_stream_conversions, ARRAY_SIZE(mav4_stream_conversions));
        break;
    case 4:
        AP_Param::convert_old_parameters(mav5_stream_conversions, ARRAY_SIZE(mav5_stream_conversions));
        break;
    case 5:
        AP_Param::convert_old_parameters(mav6_stream_conversions, ARRAY_SIZE(mav6_stream_conversions));
        break;
    case 6:
        AP_Param::convert_old_parameters(mav7_stream_conversions, ARRAY_SIZE(mav7_stream_conversions));
        break;
    default:
        // MAV8 and later have no legacy SRx source in this branch.
        break;
    }
}

uint8_t GCS_Rover::sysid_this_mav() const
{
    return GCS::sysid_this_mav();
}

bool GCS_Rover::simple_input_active() const
{
    return false;
}

bool GCS_Rover::supersimple_input_active() const
{
    return false;
}

void GCS_Rover::update_vehicle_sensor_status_flags(void)
{
    // mode-specific:
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION |
        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

    if (rover.control_mode->attitude_stabilized()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // 3D angular rate control
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // 3D angular rate control
    }
    if (rover.control_mode->is_autopilot_mode()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
    }

#if HAL_PROXIMITY_ENABLED
    const AP_Proximity *proximity = AP_Proximity::get_singleton();
    if (proximity && proximity->get_status() > AP_Proximity::Status::NotConnected) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
    if (proximity && proximity->get_status() != AP_Proximity::Status::NoData) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
#endif

#if AP_RANGEFINDER_ENABLED
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->num_sensors() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        AP_RangeFinder_Backend *s = rangefinder->get_backend(0);
        if (s != nullptr && s->has_data()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif
}
