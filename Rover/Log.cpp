#include "Rover.h"

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

#if HAL_LOGGING_ENABLED

// Write an attitude packet
void Rover::Log_Write_Attitude()
{
    const Vector3f targets(0.0f, 0.0f, 0.0f);

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

// Exact-pivot state, CapturePath and heading-handoff diagnostics.  These are
// vehicle-specific packets so their message IDs remain in Rover's 0..31 range.
struct PACKED log_XPNV {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t phase;
    uint8_t primitive;
    uint8_t fault;
    uint16_t events;
    uint16_t flags;
    uint32_t generation;
    float endpoint_distance;
    float planned_distance;
    float planned_speed;
    float desired_speed;
    float desired_turn_rate;
    float yaw_error;
    float xtrack_error;
};

struct PACKED log_XCAP {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t segment;
    int8_t direction;
    float progress;
    float length;
    float radius;
    float target_speed;
    float tracking_error;
    float heading_error;
    float endpoint_distance;
};

struct PACKED log_XHOF {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float along;
    float rejoin;
    float blend;
    float distance_ratio;
    float heading_ratio;
    float path_weight;
    float heading_rate;
    float path_rate;
    float output_rate;
};

static_assert(sizeof(log_XPNV) == 50, "XPNV format/struct mismatch");
static_assert(sizeof(log_XCAP) == 41, "XCAP format/struct mismatch");
static_assert(sizeof(log_XHOF) == 47, "XHOF format/struct mismatch");

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

void Rover::Log_Write_Exact_Pivot_Nav(
    const AR_WPNav::ExactPivotDiagSnapshot &snapshot,
    uint64_t time_us,
    bool critical)
{
    const float quiet_nan = logger.quiet_nanf();
    const bool endpoint_valid =
        (snapshot.state_flags & AR_WPNav::DiagStateEndpointValid) != 0U;
    const bool plan_valid =
        (snapshot.state_flags & AR_WPNav::DiagStatePlanValid) != 0U;
    const bool yaw_valid =
        (snapshot.state_flags & AR_WPNav::DiagStateYawErrorValid) != 0U;
    const log_XPNV pkt = {
        LOG_PACKET_HEADER_INIT(LOG_XPNV_MSG),
        time_us             : time_us,
        phase               : snapshot.phase,
        primitive           : snapshot.primitive,
        fault               : snapshot.fault,
        events              : snapshot.transition_flags,
        flags               : snapshot.state_flags,
        generation          : snapshot.handoff_generation,
        endpoint_distance   : endpoint_valid && isfinite(snapshot.endpoint_distance_m) ?
                              snapshot.endpoint_distance_m : quiet_nan,
        planned_distance    : plan_valid && isfinite(snapshot.planned_distance_m) ?
                              snapshot.planned_distance_m : quiet_nan,
        planned_speed       : plan_valid && isfinite(snapshot.planned_speed_mps) ?
                              snapshot.planned_speed_mps : quiet_nan,
        desired_speed       : isfinite(snapshot.desired_speed_mps) ?
                              snapshot.desired_speed_mps : quiet_nan,
        desired_turn_rate   : isfinite(snapshot.desired_turn_rate_rads) ?
                              snapshot.desired_turn_rate_rads : quiet_nan,
        yaw_error           : yaw_valid && isfinite(snapshot.yaw_error_deg) ?
                              snapshot.yaw_error_deg : quiet_nan,
        xtrack_error        : endpoint_valid && isfinite(snapshot.xtrack_error_m) ?
                              snapshot.xtrack_error_m : quiet_nan,
    };
    if (critical) {
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    } else {
        logger.WriteBlock(&pkt, sizeof(pkt));
    }
}

void Rover::Log_Write_Exact_Pivot_Capture(
    const AR_WPNav::ExactPivotDiagSnapshot &snapshot,
    uint64_t time_us,
    bool critical)
{
    const float quiet_nan = logger.quiet_nanf();
    const auto finite_or_nan = [quiet_nan](float value) {
        return isfinite(value) ? value : quiet_nan;
    };
    const log_XCAP pkt = {
        LOG_PACKET_HEADER_INIT(LOG_XCAP_MSG),
        time_us             : time_us,
        segment             : snapshot.capture.segment,
        direction           : snapshot.capture.direction,
        progress            : finite_or_nan(snapshot.capture.progress_m),
        length              : finite_or_nan(snapshot.capture.length_m),
        radius              : finite_or_nan(snapshot.capture.radius_m),
        target_speed        : finite_or_nan(snapshot.capture.target_speed_mps),
        tracking_error      : finite_or_nan(snapshot.capture.tracking_error_m),
        heading_error       : finite_or_nan(snapshot.capture.heading_error_deg),
        endpoint_distance   : finite_or_nan(snapshot.capture.endpoint_distance_m),
    };
    if (critical) {
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    } else {
        logger.WriteBlock(&pkt, sizeof(pkt));
    }
}

void Rover::Log_Write_Exact_Pivot_Handoff(
    const AR_WPNav::ExactPivotDiagSnapshot &snapshot,
    uint64_t time_us,
    bool critical)
{
    const float quiet_nan = logger.quiet_nanf();
    const auto finite_or_nan = [quiet_nan](float value) {
        return isfinite(value) ? value : quiet_nan;
    };
    const log_XHOF pkt = {
        LOG_PACKET_HEADER_INIT(LOG_XHOF_MSG),
        time_us        : time_us,
        along          : finite_or_nan(snapshot.handoff.along_m),
        rejoin         : finite_or_nan(snapshot.handoff.rejoin_m),
        blend          : finite_or_nan(snapshot.handoff.blend_m),
        distance_ratio : finite_or_nan(snapshot.handoff.distance_ratio),
        heading_ratio  : finite_or_nan(snapshot.handoff.heading_ratio),
        path_weight    : finite_or_nan(snapshot.handoff.path_weight),
        heading_rate   : finite_or_nan(snapshot.handoff.heading_rate_rads),
        path_rate      : finite_or_nan(snapshot.handoff.path_rate_rads),
        output_rate    : finite_or_nan(snapshot.handoff.output_rate_rads),
    };
    if (critical) {
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    } else {
        logger.WriteBlock(&pkt, sizeof(pkt));
    }
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

// @LoggerMessage: XPNV
// @Description: Exact-pivot navigation state and transition diagnostics
// @Field: Ph: Exact-pivot phase
// @Field: Pr: Active motion primitive
// @Field: Flt: Exact-pivot fault reason
// @Field: Ev: Sticky transition flags
// @Field: Flg: Live state and validity flags
// @Field: Gen: Atomic handoff generation

    { LOG_XPNV_MSG, sizeof(log_XPNV),
      "XPNV", "QBBBHHIfffffff",
      "TimeUS,Ph,Pr,Flt,Ev,Flg,Gen,EndD,PlanD,PlanV,DesV,DesR,YErr,XTrk",
      "s------mmnnEdm", "F------0000000", true },

// @LoggerMessage: XCAP
// @Description: Exact-pivot one-shot CapturePath diagnostics

    { LOG_XCAP_MSG, sizeof(log_XCAP),
      "XCAP", "QBbfffffff",
      "TimeUS,Seg,Dir,Prog,Length,Radius,TgtSpd,TrkErr,HdgErr,D0Dist",
      "s--mmmnmdm", "F--0000000", true },

// @LoggerMessage: XHOF
// @Description: Exact-pivot Spin-to-Path heading handoff diagnostics

    { LOG_XHOF_MSG, sizeof(log_XHOF),
      "XHOF", "Qfffffffff",
      "TimeUS,Along,Rejoin,Blend,DRat,HRat,Lambda,HRate,PRate,OutRate",
      "smmm---EEE", "F000000000", true },
};

uint8_t Rover::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

#endif  // LOGGING_ENABLED
