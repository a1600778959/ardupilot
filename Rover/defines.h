#pragma once

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define SERVO_MAX 4500.0  // This value represents 45 degrees and is just an arbitrary representation of servo max travel.

// types of failsafe events
#define FAILSAFE_EVENT_THROTTLE (1<<0)
#define FAILSAFE_EVENT_GCS      (1<<1)

//  Logging parameters - only 32 messages are available to the vehicle here.
enum LoggingParameters {
    LOG_THR_MSG,
    LOG_NTUN_MSG,
    LOG_STEERING_MSG,
    LOG_GUIDEDTARGET_MSG,
    // IDs 4, 5 and 6 were XPNV, XCAP and XHOF. Keep them reserved so
    // existing logs cannot be decoded as a different Rover message.
    LOG_PTRL_MSG = 7,
    LOG_PTRG_MSG = 8,
};

static_assert(LOG_PTRG_MSG < 32, "Rover vehicle log IDs exhausted");

#define MASK_LOG_ATTITUDE_FAST  (1<<0)
#define MASK_LOG_ATTITUDE_MED   (1<<1)
#define MASK_LOG_GPS            (1<<2)
#define MASK_LOG_PM             (1<<3)
#define MASK_LOG_THR            (1<<4)
#define MASK_LOG_NTUN           (1<<5)
//#define MASK_LOG_MODE         (1<<6) // no longer used
#define MASK_LOG_IMU            (1<<7)
#define MASK_LOG_CMD            (1<<8)
#define MASK_LOG_CURRENT        (1<<9)
#define MASK_LOG_RANGEFINDER    (1<<10)
#define MASK_LOG_COMPASS        (1<<11)
#define MASK_LOG_CAMERA         (1<<12)
#define MASK_LOG_STEERING       (1<<13)
#define MASK_LOG_RC             (1<<14)
// #define MASK_LOG_ARM_DISARM     (1<<15)
#define MASK_LOG_IMU_RAW        (1UL<<19)
#define MASK_LOG_VIDEO_STABILISATION (1UL<<20)
#define MASK_LOG_OPTFLOW                (1UL<<21)

// for mavlink SET_POSITION_TARGET messages
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1))
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4))
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7))
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<9)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<10)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<11)

// for mavlink SET_ATTITUDE_TARGET messages
#define MAVLINK_SET_ATT_TYPE_MASK_ROLL_RATE_IGNORE     (1<<0)
#define MAVLINK_SET_ATT_TYPE_MASK_PITCH_RATE_IGNORE    (1<<1)
#define MAVLINK_SET_ATT_TYPE_MASK_YAW_RATE_IGNORE      (1<<2)
#define MAVLINK_SET_ATT_TYPE_MASK_THROTTLE_IGNORE      (1<<6)
#define MAVLINK_SET_ATT_TYPE_MASK_ATTITUDE_IGNORE      (1<<7)

// radio failsafe enum (FS_THR_ENABLE parameter)
enum fs_thr_enable {
    FS_THR_DISABLED = 0,
    FS_THR_ENABLED,
    FS_THR_ENABLED_CONTINUE_MISSION,
};

// gcs failsafe enum (FS_GCS_ENABLE parameter)
enum fs_gcs_enable {
    FS_GCS_DISABLED = 0,
    FS_GCS_ENABLED,
    FS_GCS_ENABLED_CONTINUE_MISSION,
};

enum fs_crash_action {
  FS_CRASH_DISABLE = 0,
  FS_CRASH_HOLD = 1,
  FS_CRASH_HOLD_AND_DISARM = 2
};

enum fs_ekf_action {
    FS_EKF_DISABLE = 0,
    FS_EKF_HOLD = 1,
    FS_EKF_REPORT_ONLY = 2,
};

#define DISTANCE_HOME_MINCHANGE 0.5f  // minimum distance to adjust home location

enum class PilotSteerType : uint8_t {
    DEFAULT = 0,
    TWO_PADDLES = 1,
    DIR_REVERSED_WHEN_REVERSING = 2,
    DIR_UNCHANGED_WHEN_REVERSING = 3,
};
