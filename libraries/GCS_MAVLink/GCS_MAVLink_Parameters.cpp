#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include "GCS.h"

#include <AP_BattMonitor/AP_BattMonitor_config.h>
#include <AP_Compass/AP_Compass_config.h>
#include <AP_ESC_Telem/AP_ESC_Telem_config.h>
#include <AP_GPS/AP_GPS_config.h>
#include <AP_Param/AP_Param.h>
#include <AP_RPM/AP_RPM_config.h>
#include <AP_RangeFinder/AP_RangeFinder_config.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

const AP_Param::GroupInfo *GCS::_chan_var_info[MAVLINK_COMM_NUM_BUFFERS];

const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    // @Param: _RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2 and SCALED_PRESSURE3
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_RAW_SENS", 1, GCS_MAVLINK, streamRates[STREAM_RAW_SENSORS], 1),

    // @Param: _EXT_STAT
    // @DisplayName: Extended status stream rate
    // @Description: MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_EXT_STAT", 2, GCS_MAVLINK, streamRates[STREAM_EXTENDED_STATUS], 1),

    // @Param: _RC_CHAN
    // @DisplayName: RC Channel stream rate
    // @Description: MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_RC_CHAN", 3, GCS_MAVLINK, streamRates[STREAM_RC_CHANNELS], 1),

    // @Param: _RAW_CTRL
    // @DisplayName: Raw Control stream rate
    // @Description: MAVLink Raw Control stream rate of SERVO_OUT
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_RAW_CTRL", 4, GCS_MAVLINK, streamRates[STREAM_RAW_CONTROLLER], 1),

    // @Param: _POSITION
    // @DisplayName: Position stream rate
    // @Description: MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_POSITION", 5, GCS_MAVLINK, streamRates[STREAM_POSITION], 1),

    // @Param: _EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2 and PID_TUNING
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_EXTRA1", 6, GCS_MAVLINK, streamRates[STREAM_EXTRA1], 1),

    // @Param: _EXTRA2
    // @DisplayName: Extra data type 2 stream rate
    // @Description: MAVLink Stream rate of VFR_HUD
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_EXTRA2", 7, GCS_MAVLINK, streamRates[STREAM_EXTRA2], 1),

    // @Param: _EXTRA3
    // @DisplayName: Extra data type 3 stream rate
    // @Description: MAVLink Stream rate of AHRS, SYSTEM_TIME, RANGEFINDER, DISTANCE_SENSOR, BATTERY_STATUS, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, RPM, ESC TELEMETRY, and WHEEL_DISTANCE
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_EXTRA3", 8, GCS_MAVLINK, streamRates[STREAM_EXTRA3], 1),

    // @Param: _PARAMS
    // @DisplayName: Parameter stream rate
    // @Description: MAVLink Stream rate of PARAM_VALUE
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_PARAMS", 9, GCS_MAVLINK, streamRates[STREAM_PARAMS], 10),

    // @Param: _OPTIONS
    // @DisplayName: MAVLink channel options
    // @Description: Configures this MAVLink channel. Set the corresponding bit on each MAVn_OPTIONS parameter where the behaviour is required.
    // @Bitmask: 0:Accept unsigned MAVLink2 messages,1:Don't forward MAVLink to/from,2:Ignore stream-rate requests,3:Forward packets that fail CRC
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 20, GCS_MAVLINK, options, 0),

    // PARAMETER_CONVERSION - Added: Jul-2026 for this ArduPilot-4.7 backport
    AP_GROUPINFO_FLAGS("_OPTIONSCNV", 21, GCS_MAVLINK,
                       options_were_converted, 0, AP_PARAM_FLAG_HIDDEN),

    AP_GROUPEND
};

#if APM_BUILD_TYPE(APM_BUILD_Rover)

// Keep the Rover stream membership exactly as it was before the parameter
// objects became dynamically owned by each backend.
static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU,
    MSG_SCALED_IMU2,
    MSG_SCALED_IMU3,
    MSG_SCALED_PRESSURE,
    MSG_SCALED_PRESSURE2,
    MSG_SCALED_PRESSURE3,
};

static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
#if HAL_WITH_MCU_MONITORING
    MSG_MCU_STATUS,
#endif
    MSG_MEMINFO,
    MSG_CURRENT_WAYPOINT,
    MSG_GPS_RAW,
    MSG_GPS_RTK,
#if GPS_MAX_RECEIVERS > 1
    MSG_GPS2_RAW,
    MSG_GPS2_RTK,
#endif
    MSG_NAV_CONTROLLER_OUTPUT,
#if AP_FENCE_ENABLED
    MSG_FENCE_STATUS,
#endif
    MSG_POSITION_TARGET_GLOBAL_INT,
};

static const ap_message STREAM_POSITION_msgs[] = {
    MSG_LOCATION,
    MSG_LOCAL_POSITION,
};

static const ap_message STREAM_RAW_CONTROLLER_msgs[] = {
    MSG_SERVO_OUT,
};

static const ap_message STREAM_RC_CHANNELS_msgs[] = {
    MSG_SERVO_OUTPUT_RAW,
    MSG_RC_CHANNELS,
#if AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED
    MSG_RC_CHANNELS_RAW,
#endif
};

static const ap_message STREAM_EXTRA1_msgs[] = {
    MSG_ATTITUDE,
#if AP_SIM_ENABLED
    MSG_SIMSTATE,
#endif
    MSG_AHRS2,
    MSG_PID_TUNING,
};

static const ap_message STREAM_EXTRA2_msgs[] = {
    MSG_VFR_HUD,
};

static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
#if AP_RANGEFINDER_ENABLED
    MSG_RANGEFINDER,
#endif
    MSG_DISTANCE_SENSOR,
    MSG_SYSTEM_TIME,
#if AP_BATTERY_ENABLED
    MSG_BATTERY_STATUS,
#endif
#if COMPASS_CAL_ENABLED
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
#endif
    MSG_EKF_STATUS_REPORT,
    MSG_VIBRATION,
#if AP_RPM_ENABLED
    MSG_RPM,
#endif
    MSG_WHEEL_DISTANCE,
#if HAL_WITH_ESC_TELEM
    MSG_ESC_TELEMETRY,
#endif
};

static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM,
};

const GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_RAW_CONTROLLER),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
    MAV_STREAM_ENTRY(STREAM_EXTRA2),
    MAV_STREAM_ENTRY(STREAM_EXTRA3),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR
};

#endif // APM_BUILD_TYPE(APM_BUILD_Rover)

#endif // HAL_GCS_ENABLED
