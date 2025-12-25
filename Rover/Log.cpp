#include "Rover.h"

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

#if HAL_LOGGING_ENABLED

// Write an attitude packet
void Rover::Log_Write_Attitude()
{
    float desired_pitch = degrees(g2.attitude_control.get_desired_pitch());
    const Vector3f targets(0.0f, desired_pitch, 0.0f);

    ahrs.Write_Attitude(targets);

    AP::ahrs().Log_Write();

    // log steering rate controller
    logger.Write_PID(LOG_PIDS_MSG, g2.attitude_control.get_steering_rate_pid().get_pid_info());
    logger.Write_PID(LOG_PIDA_MSG, g2.attitude_control.get_throttle_speed_pid_info());

}

// guided mode logging
struct PACKED log_GuidedTarget {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
};

// Write a Guided mode target
void Rover::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target)
{
    struct log_GuidedTarget pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GUIDEDTARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : target_type,
        pos_target_x    : pos_target.x,
        pos_target_y    : pos_target.y,
        pos_target_z    : pos_target.z,
        vel_target_x    : vel_target.x,
        vel_target_y    : vel_target.y,
        vel_target_z    : vel_target.z
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float wp_distance;
    float wp_bearing;
    float nav_bearing;
    uint16_t yaw;
    float xtrack_error;
};

// Write a navigation tuning packet
void Rover::Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_us             : AP_HAL::micros64(),
        wp_distance         : control_mode->get_distance_to_destination(),
        wp_bearing          : control_mode->wp_bearing(),
        nav_bearing         : control_mode->nav_bearing(),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        xtrack_error        : control_mode->crosstrack_error()
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Steering {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t steering_in;
    float steering_out;
    float desired_lat_accel;
    float lat_accel;
    float desired_turn_rate;
    float turn_rate;
};

// Write a steering packet
void Rover::Log_Write_Steering()
{
    float lat_accel = logger.quiet_nanf();
    g2.attitude_control.get_lat_accel(lat_accel);
    struct log_Steering pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STEERING_MSG),
        time_us        : AP_HAL::micros64(),
        steering_in        : channel_steer->get_control_in(),
        steering_out       : g2.motors.get_steering(),
        desired_lat_accel  : control_mode->get_desired_lat_accel(),
        lat_accel          : lat_accel,
        desired_turn_rate  : degrees(g2.attitude_control.get_desired_turn_rate()),
        turn_rate          : degrees(ahrs.get_yaw_rate_earth())
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Throttle {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t throttle_in;
    float throttle_out;
    float desired_speed;
    float speed;
    float accel_x;
};

// Write a throttle control packet
void Rover::Log_Write_Throttle()
{
    const Vector3f accel = ins.get_accel();
    float speed = logger.quiet_nanf();
    g2.attitude_control.get_forward_speed(speed);
    struct log_Throttle pkt = {
        LOG_PACKET_HEADER_INIT(LOG_THR_MSG),
        time_us         : AP_HAL::micros64(),
        throttle_in     : channel_throttle->get_control_in(),
        throttle_out    : g2.motors.get_throttle(),
        desired_speed   : g2.attitude_control.get_desired_speed(),
        speed           : speed,
        accel_x         : accel.x
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Rover::Log_Write_RC(void)
{
    logger.Write_RCIN();
    logger.Write_RCOUT();
#if AP_RSSI_ENABLED
    if (rssi.enabled()) {
        logger.Write_RSSI();
    }
#endif
}

void Rover::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_Mode((uint8_t)control_mode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information

const LogStructure Rover::log_structure[] = {
    LOG_COMMON_STRUCTURES,

// @LoggerMessage: THR
// @Description: Throttle related messages
// @Field: TimeUS: Time since system startup
// @Field: ThrIn: Throttle Input
// @Field: ThrOut: Throttle Output 
// @Field: DesSpeed: Desired speed 
// @Field: Speed: Actual speed
// @Field: AccX: Acceleration

    { LOG_THR_MSG, sizeof(log_Throttle),
      "THR", "Qhffff", "TimeUS,ThrIn,ThrOut,DesSpeed,Speed,AccX", "s--nno", "F--000" },

// @LoggerMessage: NTUN
// @Description: Navigation Tuning information - e.g. vehicle destination
// @URL: http://ardupilot.org/rover/docs/navigation.html
// @Field: TimeUS: Time since system startup
// @Field: WpDist: distance to the current navigation waypoint
// @Field: WpBrg: bearing to the current navigation waypoint
// @Field: DesYaw: the vehicle's desired heading
// @Field: Yaw: the vehicle's current heading
// @Field: XTrack: the vehicle's current distance from the current travel segment

    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),
      "NTUN", "QfffHf", "TimeUS,WpDist,WpBrg,DesYaw,Yaw,XTrack", "smhhhm", "F000B0" },
    
// @LoggerMessage: STER
// @Description: Steering related messages
// @Field: TimeUS: Time since system startup
// @Field: SteerIn: Steering input
// @Field: SteerOut: Normalized steering output 
// @Field: DesLatAcc: Desired lateral acceleration
// @Field: LatAcc: Actual lateral acceleration
// @Field: DesTurnRate: Desired turn rate
// @Field: TurnRate: Actual turn rate
    
    { LOG_STEERING_MSG, sizeof(log_Steering),
      "STER", "Qhfffff",   "TimeUS,SteerIn,SteerOut,DesLatAcc,LatAcc,DesTurnRate,TurnRate", "s--ookk", "F--0000" },

// @LoggerMessage: GUIP
// @Description: Guided mode target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: pX: Target position, X-Axis
// @Field: pY: Target position, Y-Axis
// @Field: pZ: Target position, Z-Axis
// @Field: vX: Target velocity, X-Axis
// @Field: vY: Target velocity, Y-Axis
// @Field: vZ: Target velocity, Z-Axis
    
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUIP",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ", "s-mmmnnn", "F-000000" },
};

uint8_t Rover::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

#endif  // LOGGING_ENABLED
