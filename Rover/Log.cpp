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

static constexpr char patrol_log_format[] = "QBBBBHHffff";
static constexpr char patrol_log_labels[] =
    "TimeUS,Ev,St,Leg,Flt,Flg,Line,Sp,Off,XTrk,Dist";
static constexpr char patrol_geometry_log_format[] = "QHLLLLLLLLLL";
static constexpr char patrol_geometry_log_labels[] =
    "TimeUS,Line,ALat,ALng,BLat,BLng,OLat,OLng,DLat,DLng,NLat,NLng";
static constexpr char autotune_sample_format[] = "QBBHHffffffffff";
static constexpr char autotune_sample_labels[] =
    "TimeUS,St,Src,Flg,Run,Tgt,Act,Out,X,Y,Spd,YRate,XMar,YMar,Unc";
static constexpr char autotune_model_format[] = "QBBBHHfffffffff";
static constexpr char autotune_model_labels[] =
    "TimeUS,St,Ev,Why,Run,N,A1,A2,B1,B2,Fit,Sat,Ov,Gain,Tau";
static constexpr char autotune_param_format[] = "QBHNBBBfff";
static constexpr char autotune_param_labels[] =
    "TimeUS,Idx,Run,Name,Act,Why,Res,Base,Cand,Read";

static_assert((sizeof(patrol_log_format) - 1U) <=
              sizeof(((log_Format *)nullptr)->format),
              "PTRL format exceeds DataFlash FMT limit");
static_assert((sizeof(patrol_geometry_log_format) - 1U) <=
              sizeof(((log_Format *)nullptr)->format),
              "PTRG format exceeds DataFlash FMT limit");
static_assert((sizeof(patrol_log_labels) - 1U) <=
              sizeof(((log_Format *)nullptr)->labels),
              "PTRL labels exceed DataFlash FMT limit");
static_assert((sizeof(patrol_geometry_log_labels) - 1U) <=
              sizeof(((log_Format *)nullptr)->labels),
              "PTRG labels exceed DataFlash FMT limit");
static_assert((sizeof(autotune_sample_format) - 1U) <=
              sizeof(((log_Format *)nullptr)->format),
              "RATS format exceeds DataFlash FMT limit");
static_assert((sizeof(autotune_model_format) - 1U) <=
              sizeof(((log_Format *)nullptr)->format),
              "RATM format exceeds DataFlash FMT limit");
static_assert((sizeof(autotune_param_format) - 1U) <=
              sizeof(((log_Format *)nullptr)->format),
              "RATP format exceeds DataFlash FMT limit");
static_assert((sizeof(autotune_sample_labels) - 1U) <=
              sizeof(((log_Format *)nullptr)->labels),
              "RATS labels exceed DataFlash FMT limit");
static_assert((sizeof(autotune_model_labels) - 1U) <=
              sizeof(((log_Format *)nullptr)->labels),
              "RATM labels exceed DataFlash FMT limit");
static_assert((sizeof(autotune_param_labels) - 1U) <=
              sizeof(((log_Format *)nullptr)->labels),
              "RATP labels exceed DataFlash FMT limit");

struct PACKED log_Patrol {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t event;
    uint8_t state;
    uint8_t leg;
    uint8_t fault;
    uint16_t flags;
    uint16_t line;
    float spacing;
    float offset;
    float xtrack;
    float distance;
};

struct PACKED log_PatrolGeometry {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t line;
    int32_t point_a_lat;
    int32_t point_a_lng;
    int32_t point_b_lat;
    int32_t point_b_lng;
    int32_t origin_lat;
    int32_t origin_lng;
    int32_t destination_lat;
    int32_t destination_lng;
    int32_t next_destination_lat;
    int32_t next_destination_lng;
};

struct PACKED log_AutoTuneSample {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t stage;
    uint8_t source;
    uint16_t flags;
    uint16_t run_id;
    float target;
    float actual;
    float output;
    float position_x;
    float position_y;
    float speed;
    float yaw_rate;
    float margin_x;
    float margin_y;
    float position_uncertainty;
};

struct PACKED log_AutoTuneModel {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t stage;
    uint8_t event;
    uint8_t reason;
    uint16_t run_id;
    uint16_t samples;
    float theta[4];
    float fit;
    float saturation;
    float overshoot;
    float gain;
    float time_constant;
};

struct PACKED log_AutoTuneParam {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t index;
    uint16_t run_id;
    char name[16];
    uint8_t action;
    uint8_t reason;
    uint8_t result;
    float baseline;
    float candidate;
    float readback;
};

static_assert(sizeof(log_Patrol) == 35, "PTRL format/struct mismatch");
static_assert(sizeof(log_PatrolGeometry) == 53,
              "PTRG format/struct mismatch");
static_assert(sizeof(log_AutoTuneSample) == 57,
              "RATS format/struct mismatch");
static_assert(sizeof(log_AutoTuneModel) == 54,
              "RATM format/struct mismatch");
static_assert(sizeof(log_AutoTuneParam) == 45,
              "RATP format/struct mismatch");

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

void Rover::Log_Write_Patrol(const ModePatrol::LogSnapshot &snapshot,
                             bool critical,
                             bool write_geometry)
{
    const uint64_t time_us = AP_HAL::micros64();
    const log_Patrol pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PTRL_MSG),
        time_us              : time_us,
        event                : snapshot.event,
        state                : snapshot.state,
        leg                  : snapshot.leg,
        fault                : snapshot.fault,
        flags                : snapshot.flags,
        line                 : snapshot.line,
        spacing              : snapshot.spacing_m,
        offset               : snapshot.offset_m,
        xtrack               : snapshot.xtrack_m,
        distance             : snapshot.distance_m,
    };
    if (critical) {
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    } else {
        logger.WriteBlock(&pkt, sizeof(pkt));
    }

    if (!write_geometry) {
        return;
    }

    const log_PatrolGeometry geometry_pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PTRG_MSG),
        time_us              : time_us,
        line                 : snapshot.geometry_line,
        point_a_lat          : snapshot.point_a.lat,
        point_a_lng          : snapshot.point_a.lng,
        point_b_lat          : snapshot.point_b.lat,
        point_b_lng          : snapshot.point_b.lng,
        origin_lat           : snapshot.origin.lat,
        origin_lng           : snapshot.origin.lng,
        destination_lat      : snapshot.destination.lat,
        destination_lng      : snapshot.destination.lng,
        next_destination_lat : snapshot.next_destination.lat,
        next_destination_lng : snapshot.next_destination.lng,
    };
    if (critical) {
        logger.WriteCriticalBlock(&geometry_pkt, sizeof(geometry_pkt));
    } else {
        logger.WriteBlock(&geometry_pkt, sizeof(geometry_pkt));
    }
}

void Rover::Log_Write_AutoTune_Sample(const ModeAutoTune::LogSample &snapshot)
{
    const log_AutoTuneSample pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RATS_MSG),
        time_us     : AP_HAL::micros64(),
        stage       : snapshot.stage,
        source      : snapshot.source,
        flags       : snapshot.flags,
        run_id      : snapshot.run_id,
        target      : snapshot.target,
        actual      : snapshot.actual,
        output      : snapshot.output,
        position_x  : snapshot.position_x,
        position_y  : snapshot.position_y,
        speed       : snapshot.speed,
        yaw_rate    : snapshot.yaw_rate,
        margin_x    : snapshot.margin_x,
        margin_y    : snapshot.margin_y,
        position_uncertainty : snapshot.position_uncertainty,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Rover::Log_Write_AutoTune_Model(const ModeAutoTune::LogModel &snapshot,
                                     bool critical)
{
    log_AutoTuneModel pkt{};
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_RATM_MSG;
    pkt.time_us = AP_HAL::micros64();
    pkt.stage = snapshot.stage;
    pkt.event = snapshot.event;
    pkt.reason = snapshot.reason;
    pkt.run_id = snapshot.run_id;
    pkt.samples = snapshot.samples;
    memcpy(pkt.theta, snapshot.theta, sizeof(pkt.theta));
    pkt.fit = snapshot.fit;
    pkt.saturation = snapshot.saturation;
    pkt.overshoot = snapshot.overshoot;
    pkt.gain = snapshot.gain;
    pkt.time_constant = snapshot.time_constant;
    if (critical) {
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    } else {
        logger.WriteBlock(&pkt, sizeof(pkt));
    }
}

void Rover::Log_Write_AutoTune_Param(const ModeAutoTune::LogParam &snapshot,
                                     bool critical)
{
    log_AutoTuneParam pkt{};
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_RATP_MSG;
    pkt.time_us = AP_HAL::micros64();
    pkt.index = snapshot.index;
    pkt.run_id = snapshot.run_id;
    memcpy(pkt.name, snapshot.name, sizeof(pkt.name));
    pkt.action = snapshot.action;
    pkt.reason = snapshot.reason;
    pkt.result = snapshot.result;
    pkt.baseline = snapshot.baseline;
    pkt.candidate = snapshot.candidate;
    pkt.readback = snapshot.readback;
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

// @LoggerMessage: PTRL
// @Description: Patrol route state, events and live theoretical-line observations
// @Field: Ev: Patrol event
// @Field: St: Patrol route state
// @Field: Leg: Active or event-related leg type
// @Field: Flt: Patrol fault reason
// @Field: Flg: Patrol state, observation-validity and input-blocking flags
// @Field: Line: Patrol line number
// @Field: Sp: Effective or queued adjacent-line spacing
// @Field: Off: Cumulative offset from the saved A-B baseline
// @Field: XTrk: Live cross-track error to the active theoretical leg
// @Field: Dist: Live distance to the active destination

    { LOG_PTRL_MSG, sizeof(log_Patrol),
      "PTRL", patrol_log_format, patrol_log_labels,
      "s------mmmm", "F------0000", true },

// @LoggerMessage: PTRG
// @Description: Patrol fixed A-B geometry and active theoretical endpoints
// @Field: Line: Active theoretical geometry line; event PTRL may name a queued line
// @Field: ALat: Saved A latitude
// @Field: ALng: Saved A longitude
// @Field: BLat: Saved B latitude
// @Field: BLng: Saved B longitude
// @Field: OLat: Active theoretical origin latitude
// @Field: OLng: Active theoretical origin longitude
// @Field: DLat: Active theoretical destination latitude
// @Field: DLng: Active theoretical destination longitude
// @Field: NLat: Next theoretical destination latitude
// @Field: NLng: Next theoretical destination longitude

    { LOG_PTRG_MSG, sizeof(log_PatrolGeometry),
      "PTRG", patrol_geometry_log_format, patrol_geometry_log_labels,
      "s-DUDUDUDUDU", "F-GGGGGGGGGG", true },

// @LoggerMessage: RATS
// @Description: Rover built-in AutoTune sample data
// @Field: St: AutoTune stage
// @Field: Src: Feedback source
// @Field: Flg: Runtime state and limit flags
// @Field: Run: AutoTune run identifier
// @Field: Tgt: Active test target
// @Field: Act: Active measured response
// @Field: Out: Maximum mixed actuator request
// @Field: X: Vehicle field-frame longitudinal position
// @Field: Y: Vehicle field-frame lateral position
// @Field: Spd: Vehicle speed
// @Field: YRate: Vehicle yaw rate
// @Field: XMar: Remaining longitudinal envelope margin
// @Field: YMar: Remaining lateral envelope margin
// @Field: Unc: Conservative horizontal position uncertainty
    { LOG_RATS_MSG, sizeof(log_AutoTuneSample),
      "RATS", autotune_sample_format, autotune_sample_labels,
      "s-------mmnEmmm", "F-------0000000", true },

// @LoggerMessage: RATM
// @Description: Rover built-in AutoTune stage and identified model data
// @Field: St: AutoTune stage
// @Field: Ev: Model or stage event
// @Field: Why: Failure or decision reason
// @Field: Run: AutoTune run identifier
// @Field: N: Valid model samples
// @Field: A1: First autoregressive coefficient
// @Field: A2: Second autoregressive coefficient
// @Field: B1: First input coefficient
// @Field: B2: Second input coefficient
// @Field: Fit: Normalised root mean square model error
// @Field: Sat: Saturated sample fraction
// @Field: Ov: Closed-loop overshoot fraction
// @Field: Gain: Identified steady-state gain
// @Field: Tau: Identified dominant time constant
    { LOG_RATM_MSG, sizeof(log_AutoTuneModel),
      "RATM", autotune_model_format, autotune_model_labels,
      "s--------------", "F--------------" },

// @LoggerMessage: RATP
// @Description: Rover built-in AutoTune parameter transaction
// @Field: Idx: Managed parameter index
// @Field: Run: AutoTune run identifier
// @Field: Name: Parameter name
// @Field: Act: Candidate, save, verify or rollback action
// @Field: Why: Parameter decision reason
// @Field: Res: Action result
// @Field: Base: RAM baseline value
// @Field: Cand: Candidate value
// @Field: Read: EEPROM readback value
    { LOG_RATP_MSG, sizeof(log_AutoTuneParam),
      "RATP", autotune_param_format, autotune_param_labels,
      "s---------", "F---------" },
};

uint8_t Rover::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

#endif  // LOGGING_ENABLED
