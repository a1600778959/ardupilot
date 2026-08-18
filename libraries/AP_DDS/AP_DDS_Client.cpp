#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_DDS_config.h"
#if AP_DDS_ENABLED
#include <uxr/client/util/ping.h>

#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_AHRS/AP_AHRS.h>
#if AP_DDS_ARM_SERVER_ENABLED || AP_DDS_ARM_CHECK_SERVER_ENABLED
#include <AP_Arming/AP_Arming.h>
#endif // AP_DDS_ARM_SERVER_ENABLED || AP_DDS_ARM_CHECK_SERVER_ENABLED
#if AP_DDS_PARAMETER_SERVER_ENABLED
#include <AP_BoardConfig/AP_BoardConfig.h>
#endif
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h>

#if AP_DDS_ARM_SERVER_ENABLED
#include "ardupilot_msgs/srv/ArmMotors.h"
#endif // AP_DDS_ARM_SERVER_ENABLED
#if AP_DDS_MODE_SWITCH_SERVER_ENABLED
#include "ardupilot_msgs/srv/ModeSwitch.h"
#endif // AP_DDS_MODE_SWITCH_SERVER_ENABLED
#if AP_DDS_ARM_CHECK_SERVER_ENABLED
#include "std_srvs/srv/Trigger.h"
#endif // AP_DDS_ARM_CHECK_SERVER_ENABLED
#if AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
#include "ardupilot_msgs/srv/Takeoff.h"
#endif // AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED

#if AP_EXTERNAL_CONTROL_ENABLED
#include "AP_DDS_ExternalControl.h"
#endif // AP_EXTERNAL_CONTROL_ENABLED
#if AP_DDS_EXTNAV_VEL_SUB_ENABLED || AP_DDS_DYNAMIC_TF_SUB_ENABLED
#include "AP_DDS_ExternalNav.h"
#endif // AP_DDS_EXTNAV_VEL_SUB_ENABLED || AP_DDS_DYNAMIC_TF_SUB_ENABLED
#include "AP_DDS_Frames.h"

#include "AP_DDS_Client.h"
#include "AP_DDS_Topic_Table.h"
#include "AP_DDS_Service_Table.h"

#define STRCPY(D,S) strncpy(D, S, ARRAY_SIZE(D))

// Enable DDS at runtime by default
static constexpr uint8_t ENABLED_BY_DEFAULT = 1;
#if AP_DDS_TIME_PUB_ENABLED
static constexpr uint16_t DELAY_TIME_TOPIC_MS = AP_DDS_DELAY_TIME_TOPIC_MS;
#endif // AP_DDS_TIME_PUB_ENABLED
#if AP_DDS_BATTERY_STATE_PUB_ENABLED
static constexpr uint16_t DELAY_BATTERY_STATE_TOPIC_MS = AP_DDS_DELAY_BATTERY_STATE_TOPIC_MS;
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED
#if AP_DDS_IMU_PUB_ENABLED
static constexpr uint16_t DELAY_IMU_TOPIC_MS = AP_DDS_DELAY_IMU_TOPIC_MS;
#endif // AP_DDS_IMU_PUB_ENABLED
#if AP_DDS_UWB_PUB_ENABLED
static constexpr uint16_t DELAY_UWB_TOPIC_MS = AP_DDS_DELAY_UWB_TOPIC_MS;
#endif // AP_DDS_UWB_PUB_ENABLED
#if AP_DDS_LOCAL_POSE_PUB_ENABLED
static constexpr uint16_t DELAY_LOCAL_POSE_TOPIC_MS = AP_DDS_DELAY_LOCAL_POSE_TOPIC_MS;
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED
#if AP_DDS_LOCAL_VEL_PUB_ENABLED
static constexpr uint16_t DELAY_LOCAL_VELOCITY_TOPIC_MS = AP_DDS_DELAY_LOCAL_VELOCITY_TOPIC_MS;
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_GEOPOSE_PUB_ENABLED
static constexpr uint16_t DELAY_GEO_POSE_TOPIC_MS = AP_DDS_DELAY_GEO_POSE_TOPIC_MS;
#endif // AP_DDS_GEOPOSE_PUB_ENABLED
#if AP_DDS_CLOCK_PUB_ENABLED
static constexpr uint16_t DELAY_CLOCK_TOPIC_MS =AP_DDS_DELAY_CLOCK_TOPIC_MS;
#endif // AP_DDS_CLOCK_PUB_ENABLED
#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
static constexpr uint16_t DELAY_GPS_GLOBAL_ORIGIN_TOPIC_MS = AP_DDS_DELAY_GPS_GLOBAL_ORIGIN_TOPIC_MS;
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
static constexpr uint16_t DELAY_PING_MS = 500;
static constexpr uint16_t SERIAL_RETRY_DELAY_MS = 1000;
static constexpr uint8_t MAX_MISSED_PING_COUNT = 3;
static constexpr uint16_t RX_ACTIVITY_GRACE_MS = 2000;
static constexpr uint16_t STATUS_FAIL_TIMEOUT_MS = 2000;
static constexpr uint16_t RUNTIME_SUPERVISOR_INTERVAL_MS = 1000;
static constexpr uint16_t RUNTIME_DEFAULT_ALLOWED_MS = 3000;
static constexpr uint16_t RUNTIME_OPERATION_SLACK_MS = 1000;

// Define the subscriber data members, which are static class scope.
// If these are created on the stack in the subscriber,
// the AP_DDS_Client::on_topic frame size is exceeded.
#if AP_DDS_JOY_SUB_ENABLED
sensor_msgs_msg_Joy AP_DDS_Client::rx_joy_topic {};
#endif // AP_DDS_JOY_SUB_ENABLED
tf2_msgs_msg_TFMessage AP_DDS_Client::rx_dynamic_transforms_topic {};
#if AP_DDS_VEL_CTRL_ENABLED
geometry_msgs_msg_TwistStamped AP_DDS_Client::rx_velocity_control_topic {};
#endif // AP_DDS_VEL_CTRL_ENABLED
#if AP_DDS_EXTNAV_VEL_SUB_ENABLED
geometry_msgs_msg_TwistStamped AP_DDS_Client::rx_extnav_velocity_topic {};
#endif // AP_DDS_EXTNAV_VEL_SUB_ENABLED
#if AP_DDS_GLOBAL_POS_CTRL_ENABLED
ardupilot_msgs_msg_GlobalPosition AP_DDS_Client::rx_global_position_control_topic {};
#endif // AP_DDS_GLOBAL_POS_CTRL_ENABLED

// Define the parameter server data members, which are static class scope.
// If these are created on the stack, then the AP_DDS_Client::on_request
// frame size is exceeded.
#if AP_DDS_PARAMETER_SERVER_ENABLED
rcl_interfaces_srv_SetParameters_Request AP_DDS_Client::set_parameter_request {};
rcl_interfaces_srv_SetParameters_Response AP_DDS_Client::set_parameter_response {};
rcl_interfaces_srv_GetParameters_Request AP_DDS_Client::get_parameters_request {};
rcl_interfaces_srv_GetParameters_Response AP_DDS_Client::get_parameters_response {};
rcl_interfaces_msg_Parameter AP_DDS_Client::param {};
#endif

const AP_Param::GroupInfo AP_DDS_Client::var_info[] {

    // @Param: _ENABLE
    // @DisplayName: DDS enable
    // @Description: Enable DDS subsystem
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_DDS_Client, enabled, ENABLED_BY_DEFAULT, AP_PARAM_FLAG_ENABLE),

#if AP_DDS_UDP_ENABLED
    // @Param: _UDP_PORT
    // @DisplayName: DDS UDP port
    // @Description: UDP port number for DDS
    // @Range: 1 65535
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_UDP_PORT", 2, AP_DDS_Client, udp.port, 2019),

    // @Group: _IP
    // @Path: ../AP_Networking/AP_Networking_address.cpp
    AP_SUBGROUPINFO(udp.ip, "_IP", 3,  AP_DDS_Client, AP_Networking_IPV4),

#endif

    // @Param: _DOMAIN_ID
    // @DisplayName: DDS DOMAIN ID
    // @Description: Set the ROS_DOMAIN_ID
    // @Range: 0 232
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_DOMAIN_ID", 4, AP_DDS_Client, domain_id, 0),

    // @Param: _TIMEOUT_MS
    // @DisplayName: DDS ping timeout
    // @Description: The time in milliseconds the DDS client will wait for a response from the XRCE agent before reattempting.
    // @Units: ms
    // @Range: 1 10000
    // @RebootRequired: True
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_TIMEOUT_MS", 5, AP_DDS_Client, ping_timeout_ms, 1000),

    // @Param: _MAX_RETRY
    // @DisplayName: DDS ping max attempts
    // @Description: The maximum number of times the DDS client will attempt to ping the XRCE agent before exiting. Set to 0 to allow unlimited retries.
    // @Range: 0 100
    // @RebootRequired: True
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MAX_RETRY", 6, AP_DDS_Client, ping_max_retry, 10),

    // @Param: _EXTVEL_ERR
    // @DisplayName: DDS external navigation velocity error
    // @Description: One-sigma uncertainty for DDS external navigation velocity observations
    // @Units: m/s
    // @Range: 0.01 100
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_EXTVEL_ERR", 7, AP_DDS_Client, extnav_velocity_error, 0.5),

    // @Param: _EXTVEL_DLY
    // @DisplayName: DDS external navigation velocity delay
    // @Description: Average delay of DDS external navigation velocity observations relative to inertial measurements
    // @Units: ms
    // @Range: 0 1000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_EXTVEL_DLY", 8, AP_DDS_Client, extnav_velocity_delay_ms, 0),

    AP_GROUPEND
};

static void initialize(geometry_msgs_msg_Quaternion& q)
{
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
}

static void set_unknown_covariance(double (&covariance)[9])
{
    memset(covariance, 0, sizeof(covariance));
}

static void set_unavailable_covariance(double (&covariance)[9])
{
    set_unknown_covariance(covariance);
    covariance[0] = -1.0;
}

static void copy_covariance(const Matrix3f &src, double (&dst)[9])
{
    for (uint8_t row = 0; row < 3; row++) {
        for (uint8_t col = 0; col < 3; col++) {
            dst[row * 3 + col] = src[row][col];
        }
    }
}

static Matrix3f diagonal_covariance(const Vector3f &variances)
{
    Matrix3f covariance;
    covariance[0][0] = variances.x;
    covariance[1][1] = variances.y;
    covariance[2][2] = variances.z;
    return covariance;
}

namespace {

// Releases a semaphore that has already been acquired by the caller.
class ScopedSemaphoreRelease
{
public:
    explicit ScopedSemaphoreRelease(AP_HAL::Semaphore &sem) : semaphore(sem) {}
    ~ScopedSemaphoreRelease() { semaphore.give(); }

    ScopedSemaphoreRelease(const ScopedSemaphoreRelease&) = delete;
    ScopedSemaphoreRelease& operator=(const ScopedSemaphoreRelease&) = delete;

private:
    AP_HAL::Semaphore &semaphore;
};

// XRCE reliable sequence numbers wrap at 16 bits.  A target is considered
// acknowledged when it is no more than half the sequence space behind the
// Agent's latest ACK.
bool reliable_sequence_acked(uxrSeqNum ack_sequence, uxrSeqNum target_sequence)
{
    return static_cast<int16_t>(ack_sequence - target_sequence) >= 0;
}

} // namespace

AP_DDS_Client::~AP_DDS_Client()
{
    cleanup_session(false);

    // close transport
    if (is_using_serial) {
        uxr_close_custom_transport(&serial.transport);
    } else {
#if AP_DDS_UDP_ENABLED
        uxr_close_custom_transport(&udp.transport);
#endif
    }
}

void AP_DDS_Client::cleanup_session(bool notify_agent)
{
    set_runtime_phase(RuntimePhase::SESSION_CLEANUP);
    session_epoch.fetch_add(1U);

    WITH_SEMAPHORE(csem);

    connected = false;
    status_ok = false;
#if AP_DDS_TIME_PUB_ENABLED
    time_ack_pending = false;
#endif

    if (session_created) {
        if (notify_agent) {
            (void)uxr_delete_session(&session);
        }
        session_created = false;
    }

    delete[] input_reliable_stream;
    input_reliable_stream = nullptr;

    delete[] output_reliable_stream;
    output_reliable_stream = nullptr;

    delete[] output_best_effort_stream;
    output_best_effort_stream = nullptr;

    discard_stale_service_request();
}

void AP_DDS_Client::note_transport_rx(size_t wire_bytes)
{
    (void)wire_bytes;
    last_rx_activity_ms = AP_HAL::millis64();
}

bool AP_DDS_Client::take_topic_semaphore(AP_HAL::Semaphore &semaphore)
{
    if (!is_using_serial) {
        semaphore.take_blocking();
        return true;
    }
    return semaphore.take_nonblocking();
}

void AP_DDS_Client::set_runtime_phase(RuntimePhase phase, uint32_t allowed_ms)
{
    runtime_allowed_ms.store(MAX<uint32_t>(allowed_ms, 1U));
    runtime_phase.store(static_cast<uint8_t>(phase));
    runtime_progress_ms.store(AP_HAL::millis());
}

const char *AP_DDS_Client::runtime_phase_name(RuntimePhase phase)
{
    switch (phase) {
    case RuntimePhase::IDLE:
        return "idle";
    case RuntimePhase::TRANSPORT_OPEN:
        return "transport";
    case RuntimePhase::STARTUP_PING:
        return "startup_ping";
    case RuntimePhase::SESSION_CREATE:
        return "session";
    case RuntimePhase::ENTITY_CREATE:
        return "entity";
    case RuntimePhase::TOPIC_UPDATE:
        return "topic";
    case RuntimePhase::GPS_SNAPSHOT:
        return "gps";
    case RuntimePhase::AHRS_SNAPSHOT:
        return "ahrs";
    case RuntimePhase::XRCE_RUN:
        return "xrce";
    case RuntimePhase::TOPIC_CALLBACK:
        return "topic_cb";
    case RuntimePhase::SERVICE_CALLBACK:
        return "service_cb";
    case RuntimePhase::HEALTH_PING:
        return "health_ping";
    case RuntimePhase::SESSION_CLEANUP:
        return "cleanup";
    case RuntimePhase::TRANSPORT_CLOSE:
        return "transport_close";
    case RuntimePhase::BACKOFF:
        return "backoff";
    }
    return "unknown";
}

void AP_DDS_Client::request_reconnect(ReconnectReason reason)
{
    uint8_t expected = static_cast<uint8_t>(ReconnectReason::NONE);
    (void)reconnect_reason.compare_exchange_strong(expected, static_cast<uint8_t>(reason));
}

AP_DDS_Client::ReconnectReason AP_DDS_Client::consume_reconnect_request()
{
    const ReconnectReason reason = static_cast<ReconnectReason>(
        reconnect_reason.exchange(static_cast<uint8_t>(ReconnectReason::NONE)));
    if (reason == ReconnectReason::WATCHDOG_STALL && is_using_serial) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "%s Watchdog reconnect serial_cancel=%u",
                      msg_prefix,
                      (unsigned)serial.read_cancelled);
        serial.read_cancelled = false;
    }
    return reason;
}

void AP_DDS_Client::runtime_supervisor()
{
    while (true) {
        hal.scheduler->delay(RUNTIME_SUPERVISOR_INTERVAL_MS);
        if (!runtime_owner_active.load()) {
            continue;
        }

        const uint32_t now_ms = AP_HAL::millis();
        const uint32_t stalled_ms = now_ms - runtime_progress_ms.load();
        if (stalled_ms <= runtime_allowed_ms.load()) {
            runtime_stall_reported.store(false);
            continue;
        }

        // The supervisor never touches XRCE, stream storage, the transport or
        // UART ownership.  It only asks the DDS owner thread to reconnect.
        request_reconnect(ReconnectReason::WATCHDOG_STALL);
        if (!runtime_stall_reported.exchange(true)) {
            const RuntimePhase phase = static_cast<RuntimePhase>(runtime_phase.load());
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                          "%s stalled %s %lu ms",
                          msg_prefix,
                          runtime_phase_name(phase),
                          (unsigned long)stalled_ms);
        }
    }
}

bool AP_DDS_Client::begin_service_request(ServiceCommand command,
                                          uint8_t replier_id,
                                          const SampleIdentity &sample_id)
{
    uint8_t expected = static_cast<uint8_t>(ServiceState::IDLE);
    if (!pending_service_state.compare_exchange_strong(
            expected, static_cast<uint8_t>(ServiceState::FILLING))) {
        return false;
    }

    pending_service.command = command;
    pending_service.sample_id = sample_id;
    pending_service.replier_id = replier_id;
    pending_service.epoch = session_epoch.load();
    pending_service.arm = false;
    pending_service.mode = 0;
    pending_service.takeoff_alt = 0.0f;
    pending_service.result = false;
    pending_service.current_mode = 0;
    return true;
}

void AP_DDS_Client::commit_service_request()
{
    pending_service_state.store(static_cast<uint8_t>(ServiceState::QUEUED));
}

void AP_DDS_Client::cancel_service_request()
{
    pending_service_state.store(static_cast<uint8_t>(ServiceState::IDLE));
}

void AP_DDS_Client::discard_stale_service_request()
{
    const uint32_t current_epoch = session_epoch.load();
    uint8_t state = pending_service_state.load();
    if ((state != static_cast<uint8_t>(ServiceState::QUEUED) &&
         state != static_cast<uint8_t>(ServiceState::COMPLETED)) ||
        pending_service.epoch == current_epoch) {
        return;
    }

    (void)pending_service_state.compare_exchange_strong(
        state, static_cast<uint8_t>(ServiceState::IDLE));
}

void AP_DDS_Client::send_service_failure(uxrSession *uxr_session,
                                         ServiceCommand command,
                                         uint8_t replier_id_value,
                                         const SampleIdentity &sample_id)
{
    const uxrObjectId replier_id {
        .id = replier_id_value,
        .type = UXR_REPLIER_ID
    };
    SampleIdentity reply_sample_id = sample_id;

    switch (command) {
#if AP_DDS_ARM_SERVER_ENABLED
    case ServiceCommand::ARM: {
        ardupilot_msgs_srv_ArmMotors_Response response {};
        uint8_t reply_buffer[8] {};
        ucdrBuffer reply_ub;
        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        if (ardupilot_msgs_srv_ArmMotors_Response_serialize_topic(&reply_ub, &response)) {
            uxr_buffer_reply(uxr_session, reliable_out, replier_id, &reply_sample_id,
                             reply_buffer, ucdr_buffer_length(&reply_ub));
        }
        break;
    }
#endif
#if AP_DDS_MODE_SWITCH_SERVER_ENABLED
    case ServiceCommand::MODE_SWITCH: {
        ardupilot_msgs_srv_ModeSwitch_Response response {};
        uint8_t reply_buffer[8] {};
        ucdrBuffer reply_ub;
        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        if (ardupilot_msgs_srv_ModeSwitch_Response_serialize_topic(&reply_ub, &response)) {
            uxr_buffer_reply(uxr_session, reliable_out, replier_id, &reply_sample_id,
                             reply_buffer, ucdr_buffer_length(&reply_ub));
        }
        break;
    }
#endif
#if AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
    case ServiceCommand::TAKEOFF: {
        ardupilot_msgs_srv_Takeoff_Response response {};
        uint8_t reply_buffer[8] {};
        ucdrBuffer reply_ub;
        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        if (ardupilot_msgs_srv_Takeoff_Response_serialize_topic(&reply_ub, &response)) {
            uxr_buffer_reply(uxr_session, reliable_out, replier_id, &reply_sample_id,
                             reply_buffer, ucdr_buffer_length(&reply_ub));
        }
        break;
    }
#endif
#if AP_DDS_ARM_CHECK_SERVER_ENABLED
    case ServiceCommand::PREARM_CHECK: {
        std_srvs_srv_Trigger_Response response {};
        STRCPY(response.message, "DDS busy");
        uint8_t reply_buffer[sizeof(response.message) + 1] {};
        ucdrBuffer reply_ub;
        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        if (std_srvs_srv_Trigger_Response_serialize_topic(&reply_ub, &response)) {
            uxr_buffer_reply(uxr_session, reliable_out, replier_id, &reply_sample_id,
                             reply_buffer, ucdr_buffer_length(&reply_ub));
        }
        break;
    }
#endif
#if AP_DDS_PARAMETER_SERVER_ENABLED
    case ServiceCommand::SET_PARAMETERS:
    case ServiceCommand::GET_PARAMETERS: {
        // Both responses start with a bounded sequence.  An empty sequence is
        // a valid failure response and avoids a second very large parameter
        // response object on the DDS stack.
        uint8_t reply_buffer[4] {};
        ucdrBuffer reply_ub;
        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        if (ucdr_serialize_uint32_t(&reply_ub, 0U)) {
            uxr_buffer_reply(uxr_session, reliable_out, replier_id, &reply_sample_id,
                             reply_buffer, ucdr_buffer_length(&reply_ub));
        }
        break;
    }
#endif
    case ServiceCommand::NONE:
    default:
        break;
    }
}

void AP_DDS_Client::process_service_request_main_thread()
{
    uint8_t expected = static_cast<uint8_t>(ServiceState::QUEUED);
    if (!pending_service_state.compare_exchange_strong(
            expected, static_cast<uint8_t>(ServiceState::PROCESSING))) {
        return;
    }

    if (pending_service.epoch != session_epoch.load()) {
        pending_service_state.store(static_cast<uint8_t>(ServiceState::COMPLETED));
        return;
    }

    switch (pending_service.command) {
#if AP_DDS_ARM_SERVER_ENABLED
    case ServiceCommand::ARM:
        pending_service.result = pending_service.arm
                                     ? AP::arming().arm(AP_Arming::Method::DDS)
                                     : AP::arming().disarm(AP_Arming::Method::DDS);
        break;
#endif
#if AP_DDS_MODE_SWITCH_SERVER_ENABLED
    case ServiceCommand::MODE_SWITCH: {
        AP_Vehicle *vehicle = AP::vehicle();
        if (vehicle != nullptr) {
            pending_service.result = vehicle->set_mode(pending_service.mode, ModeReason::DDS_COMMAND);
            pending_service.current_mode = vehicle->get_mode();
        }
        break;
    }
#endif
#if AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
    case ServiceCommand::TAKEOFF: {
        AP_Vehicle *vehicle = AP::vehicle();
        pending_service.result = vehicle != nullptr && vehicle->start_takeoff(pending_service.takeoff_alt);
        break;
    }
#endif
#if AP_DDS_ARM_CHECK_SERVER_ENABLED
    case ServiceCommand::PREARM_CHECK:
        pending_service.result = AP::arming().pre_arm_checks(false);
        break;
#endif
#if AP_DDS_PARAMETER_SERVER_ENABLED
    case ServiceCommand::SET_PARAMETERS: {
        set_parameter_response.results_size = set_parameter_request.parameters_size;
        for (size_t i = 0; i < set_parameter_request.parameters_size; i++) {
            param = set_parameter_request.parameters[i];

            enum ap_var_type var_type;
            AP_Param *vp;
            char param_key[AP_MAX_NAME_SIZE + 1];
            strncpy(param_key, (char *)param.name, AP_MAX_NAME_SIZE);
            param_key[AP_MAX_NAME_SIZE] = 0;

            bool param_isnan = true;
            bool param_isinf = true;
            float param_value = 0.0f;
            switch (param.value.type) {
            case PARAMETER_INTEGER:
                param_isnan = isnan(param.value.integer_value);
                param_isinf = isinf(param.value.integer_value);
                param_value = float(param.value.integer_value);
                break;
            case PARAMETER_DOUBLE:
                param_isnan = isnan(param.value.double_value);
                param_isinf = isinf(param.value.double_value);
                param_value = float(param.value.double_value);
                break;
            default:
                break;
            }

            uint16_t parameter_flags = 0;
            vp = AP_Param::find(param_key, &var_type, &parameter_flags);
            if (vp == nullptr || param_isnan || param_isinf) {
                set_parameter_response.results[i].successful = false;
                strncpy(set_parameter_response.results[i].reason, "Parameter not found",
                        sizeof(set_parameter_response.results[i].reason));
                continue;
            }

            if ((parameter_flags & AP_PARAM_FLAG_INTERNAL_USE_ONLY) &&
                AP_BoardConfig::allow_set_internal_parameters()) {
                parameter_flags &= ~AP_PARAM_FLAG_INTERNAL_USE_ONLY;
            }

            if ((parameter_flags & AP_PARAM_FLAG_INTERNAL_USE_ONLY) || vp->is_read_only()) {
                set_parameter_response.results[i].successful = false;
                strncpy(set_parameter_response.results[i].reason, "Parameter is read only",
                        sizeof(set_parameter_response.results[i].reason));
                continue;
            }

            const bool force_save = vp->set_and_save_by_name_ifchanged(param_key, param_value);
            if (force_save && (parameter_flags & AP_PARAM_FLAG_ENABLE)) {
                AP_Param::invalidate_count();
            }

            set_parameter_response.results[i].successful = true;
            strncpy(set_parameter_response.results[i].reason, "Parameter accepted",
                    sizeof(set_parameter_response.results[i].reason));
        }
        break;
    }
    case ServiceCommand::GET_PARAMETERS:
        get_parameters_response.values_size = get_parameters_request.names_size;
        for (size_t i = 0; i < get_parameters_request.names_size; i++) {
            enum ap_var_type var_type;
            char param_key[AP_MAX_NAME_SIZE + 1];
            strncpy(param_key, (char *)get_parameters_request.names[i], AP_MAX_NAME_SIZE);
            param_key[AP_MAX_NAME_SIZE] = 0;

            AP_Param *vp = AP_Param::find(param_key, &var_type);
            if (vp == nullptr) {
                get_parameters_response.values[i].type = PARAMETER_NOT_SET;
                continue;
            }

            switch (var_type) {
            case AP_PARAM_INT8:
                get_parameters_response.values[i].type = PARAMETER_INTEGER;
                get_parameters_response.values[i].integer_value = ((AP_Int8 *)vp)->get();
                break;
            case AP_PARAM_INT16:
                get_parameters_response.values[i].type = PARAMETER_INTEGER;
                get_parameters_response.values[i].integer_value = ((AP_Int16 *)vp)->get();
                break;
            case AP_PARAM_INT32:
                get_parameters_response.values[i].type = PARAMETER_INTEGER;
                get_parameters_response.values[i].integer_value = ((AP_Int32 *)vp)->get();
                break;
            case AP_PARAM_FLOAT:
                get_parameters_response.values[i].type = PARAMETER_DOUBLE;
                get_parameters_response.values[i].double_value = vp->cast_to_float(var_type);
                break;
            default:
                get_parameters_response.values[i].type = PARAMETER_NOT_SET;
                break;
            }
        }
        break;
#endif
    case ServiceCommand::NONE:
    default:
        break;
    }

    pending_service_state.store(static_cast<uint8_t>(ServiceState::COMPLETED));
}

void AP_DDS_Client::drain_service_reply()
{
    if (pending_service_state.load() != static_cast<uint8_t>(ServiceState::COMPLETED)) {
        return;
    }

    if (!connected || pending_service.epoch != session_epoch.load()) {
        pending_service_state.store(static_cast<uint8_t>(ServiceState::IDLE));
        return;
    }

    WITH_SEMAPHORE(csem);
    const uxrObjectId replier_id {
        .id = pending_service.replier_id,
        .type = UXR_REPLIER_ID
    };
    SampleIdentity sample_id = pending_service.sample_id;

    switch (pending_service.command) {
#if AP_DDS_ARM_SERVER_ENABLED
    case ServiceCommand::ARM: {
        ardupilot_msgs_srv_ArmMotors_Response response {
            .result = pending_service.result
        };
        uint8_t reply_buffer[8] {};
        ucdrBuffer reply_ub;
        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        if (ardupilot_msgs_srv_ArmMotors_Response_serialize_topic(&reply_ub, &response)) {
            uxr_buffer_reply(&session, reliable_out, replier_id, &sample_id,
                             reply_buffer, ucdr_buffer_length(&reply_ub));
        }
        break;
    }
#endif
#if AP_DDS_MODE_SWITCH_SERVER_ENABLED
    case ServiceCommand::MODE_SWITCH: {
        ardupilot_msgs_srv_ModeSwitch_Response response {
            .status = pending_service.result,
            .curr_mode = pending_service.current_mode
        };
        uint8_t reply_buffer[8] {};
        ucdrBuffer reply_ub;
        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        if (ardupilot_msgs_srv_ModeSwitch_Response_serialize_topic(&reply_ub, &response)) {
            uxr_buffer_reply(&session, reliable_out, replier_id, &sample_id,
                             reply_buffer, ucdr_buffer_length(&reply_ub));
        }
        break;
    }
#endif
#if AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
    case ServiceCommand::TAKEOFF: {
        ardupilot_msgs_srv_Takeoff_Response response {
            .status = pending_service.result
        };
        uint8_t reply_buffer[8] {};
        ucdrBuffer reply_ub;
        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        if (ardupilot_msgs_srv_Takeoff_Response_serialize_topic(&reply_ub, &response)) {
            uxr_buffer_reply(&session, reliable_out, replier_id, &sample_id,
                             reply_buffer, ucdr_buffer_length(&reply_ub));
        }
        break;
    }
#endif
#if AP_DDS_ARM_CHECK_SERVER_ENABLED
    case ServiceCommand::PREARM_CHECK: {
        std_srvs_srv_Trigger_Response response {
            .success = pending_service.result
        };
        STRCPY(response.message, response.success ? "Vehicle is Armable" : "Vehicle is Not Armable");
        uint8_t reply_buffer[sizeof(response.message) + 1] {};
        ucdrBuffer reply_ub;
        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        if (std_srvs_srv_Trigger_Response_serialize_topic(&reply_ub, &response)) {
            uxr_buffer_reply(&session, reliable_out, replier_id, &sample_id,
                             reply_buffer, ucdr_buffer_length(&reply_ub));
        }
        break;
    }
#endif
#if AP_DDS_PARAMETER_SERVER_ENABLED
    case ServiceCommand::SET_PARAMETERS: {
        const uint32_t reply_size = rcl_interfaces_srv_SetParameters_Response_size_of_topic(&set_parameter_response, 0U);
        uint8_t reply_buffer[reply_size] {};
        ucdrBuffer reply_ub;
        ucdr_init_buffer(&reply_ub, reply_buffer, reply_size);
        if (rcl_interfaces_srv_SetParameters_Response_serialize_topic(&reply_ub, &set_parameter_response)) {
            uxr_buffer_reply(&session, reliable_out, replier_id, &sample_id,
                             reply_buffer, ucdr_buffer_length(&reply_ub));
        }
        break;
    }
    case ServiceCommand::GET_PARAMETERS: {
        const uint32_t reply_size = rcl_interfaces_srv_GetParameters_Response_size_of_topic(&get_parameters_response, 0U);
        uint8_t reply_buffer[reply_size] {};
        ucdrBuffer reply_ub;
        ucdr_init_buffer(&reply_ub, reply_buffer, reply_size);
        if (rcl_interfaces_srv_GetParameters_Response_serialize_topic(&reply_ub, &get_parameters_response)) {
            uxr_buffer_reply(&session, reliable_out, replier_id, &sample_id,
                             reply_buffer, ucdr_buffer_length(&reply_ub));
        }
        break;
    }
#endif
    case ServiceCommand::NONE:
    default:
        break;
    }

    pending_service_state.store(static_cast<uint8_t>(ServiceState::IDLE));
}

uxrStreamId AP_DDS_Client::output_stream_for_qos(const uxrQoS_t& qos) const
{
    return (qos.reliability == UXR_RELIABILITY_RELIABLE) ? reliable_out : best_effort_out;
}

uxrStreamId AP_DDS_Client::input_stream_for_qos(const uxrQoS_t& qos) const
{
    return (qos.reliability == UXR_RELIABILITY_RELIABLE) ? reliable_in : best_effort_in;
}

void AP_DDS_Client::finalize_topic_write(const uxrQoS_t& qos, uint32_t payload_bytes)
{
    (void)payload_bytes;
    if (qos.reliability == UXR_RELIABILITY_BEST_EFFORT) {
        uxr_flash_output_streams(&session);
    }
}

bool AP_DDS_Client::prepare_topic_stream(ucdrBuffer& ub, uxrObjectId datawriter_id, uint32_t topic_size, const char* topic_name, const uxrQoS_t& qos, uint16_t* request_id)
{
    const uxrStreamId stream_id = output_stream_for_qos(qos);
    uint16_t req_id = uxr_prepare_output_stream(&session, stream_id, datawriter_id, &ub, topic_size);

    // Best-effort streams use a single packet buffer. If earlier best-effort topics
    // in the same update cycle already filled it, flush and retry once so later
    // topics like IMU are not starved behind NavSat/Pose.
    if ((req_id == UXR_INVALID_REQUEST_ID || ub.error || ub.init == nullptr || ub.iterator == nullptr || ub.final == nullptr) &&
        stream_id.type == UXR_BEST_EFFORT_STREAM) {
        uxr_flash_output_streams(&session);
        ub = {};
        req_id = uxr_prepare_output_stream(&session, stream_id, datawriter_id, &ub, topic_size);
    }

    if (request_id != nullptr) {
        *request_id = req_id;
    }

    if (req_id != UXR_INVALID_REQUEST_ID &&
        !ub.error &&
        ub.init != nullptr &&
        ub.iterator != nullptr &&
        ub.final != nullptr) {
        return true;
    }

    (void)topic_name;
    return false;
}

#if AP_DDS_TIME_PUB_ENABLED
void AP_DDS_Client::update_topic(builtin_interfaces_msg_Time& msg)
{
    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        utc_usec = AP_HAL::micros64();
    }
    msg.sec = utc_usec / 1000000ULL;
    msg.nanosec = (utc_usec % 1000000ULL) * 1000UL;

}
#endif // AP_DDS_TIME_PUB_ENABLED

#if AP_DDS_NAVSATFIX_PUB_ENABLED
bool AP_DDS_Client::update_topic(sensor_msgs_msg_NavSatFix& msg, const uint8_t instance)
{
    set_runtime_phase(RuntimePhase::GPS_SNAPSHOT);
    auto &gps = AP::gps();
    auto &gps_semaphore = gps.get_semaphore();
    if (!take_topic_semaphore(gps_semaphore)) {
        return false;
    }
    ScopedSemaphoreRelease gps_lock(gps_semaphore);

    update_topic(msg.header.stamp);
    static_assert(GPS_MAX_RECEIVERS <= 9, "GPS_MAX_RECEIVERS is greater than 9");
    hal.util->snprintf(msg.header.frame_id, 2, "%u", instance);

    msg.status.service = 1; // SERVICE_GPS
    msg.status.status = -1; // STATUS_NO_FIX
    msg.latitude = NAN;
    msg.longitude = NAN;
    msg.altitude = NAN;
    msg.position_covariance_type = 0; // COVARIANCE_TYPE_UNKNOWN
    memset(msg.position_covariance, 0, sizeof(msg.position_covariance));

    if (!gps.is_healthy(instance)) {
        return true;
    }

    //! @todo What about glonass, compass, galileo?
    //! This will be properly designed and implemented to spec in #23277
    const auto status = gps.status(instance);
    int8_t navsat_status = -1; // STATUS_NO_FIX
    switch (status) {
    case AP_GPS::NO_GPS:
    case AP_GPS::NO_FIX:
        return true;
    case AP_GPS::GPS_OK_FIX_2D:
    case AP_GPS::GPS_OK_FIX_3D:
        navsat_status = 0; // STATUS_FIX
        break;
    case AP_GPS::GPS_OK_FIX_3D_DGPS:
        navsat_status = 1; // STATUS_SBAS_FIX
        break;
    case AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT:
    case AP_GPS::GPS_OK_FIX_3D_RTK_FIXED:
        navsat_status = 2; // STATUS_GBAS_FIX
        break;
    default:
        return true;
    }

    const auto loc = gps.location(instance);
    int32_t alt_cm;
    if (!loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
        return true;
    }

    msg.status.status = navsat_status;
    msg.latitude = loc.lat * 1E-7;
    msg.longitude = loc.lng * 1E-7;
    msg.altitude = alt_cm * 0.01;

    // Use the time of the physical GPS sample. AP_GPS::time_epoch_usec()
    // advances with the local clock and would make repeated publications of
    // one sample appear to be new measurements.
    uint64_t sample_time_usec = gps.last_message_epoch_usec(instance);
    if (sample_time_usec == 0) {
        uint64_t now_usec;
        if (!AP::rtc().get_utc_usec(now_usec)) {
            now_usec = AP_HAL::micros64();
        }
        const uint64_t sample_age_usec =
            uint64_t(AP_HAL::millis() - gps.last_message_time_ms(instance)) * 1000ULL;
        sample_time_usec = now_usec > sample_age_usec ? now_usec - sample_age_usec : 0;
    }
    msg.header.stamp.sec = sample_time_usec / 1000000ULL;
    msg.header.stamp.nanosec = (sample_time_usec % 1000000ULL) * 1000UL;

    // ROS allows double precision, ArduPilot exposes float precision today
    Matrix3f cov {};
    msg.position_covariance_type = (uint8_t)gps.position_covariance(instance, cov);
    msg.position_covariance[0] = cov[0][0];
    msg.position_covariance[1] = cov[0][1];
    msg.position_covariance[2] = cov[0][2];
    msg.position_covariance[3] = cov[1][0];
    msg.position_covariance[4] = cov[1][1];
    msg.position_covariance[5] = cov[1][2];
    msg.position_covariance[6] = cov[2][0];
    msg.position_covariance[7] = cov[2][1];
    msg.position_covariance[8] = cov[2][2];

    return true;
}
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED

#if AP_DDS_STATIC_TF_PUB_ENABLED
void AP_DDS_Client::populate_static_transforms(tf2_msgs_msg_TFMessage& msg)
{
    msg.transforms_size = 0;

    auto &gps = AP::gps();
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        const auto gps_type = gps.get_type(i);
        if (gps_type == AP_GPS::GPS_Type::GPS_TYPE_NONE) {
            continue;
        }
        update_topic(msg.transforms[i].header.stamp);
        char gps_frame_id[16];
        //! @todo should GPS frame ID's be 0 or 1 indexed in ROS?
        hal.util->snprintf(gps_frame_id, sizeof(gps_frame_id), "GPS_%u", i);
        STRCPY(msg.transforms[i].header.frame_id, BASE_LINK_FRAME_ID);
        STRCPY(msg.transforms[i].child_frame_id, gps_frame_id);
        // The body-frame offsets
        // X - Forward
        // Y - Right
        // Z - Down
        // https://ardupilot.org/copter/docs/common-sensor-offset-compensation.html#sensor-position-offset-compensation

        const auto offset = gps.get_antenna_offset(i);

        // In ROS REP 103, it follows this convention
        // X - Forward
        // Y - Left
        // Z - Up
        // https://www.ros.org/reps/rep-0103.html#axis-orientation

        msg.transforms[i].transform.translation.x = offset[0];
        msg.transforms[i].transform.translation.y = -1 * offset[1];
        msg.transforms[i].transform.translation.z = -1 * offset[2];

        // Ensure rotation is initialized.
        initialize(msg.transforms[i].transform.rotation);

        msg.transforms_size++;
    }

}
#endif // AP_DDS_STATIC_TF_PUB_ENABLED

#if AP_DDS_BATTERY_STATE_PUB_ENABLED
void AP_DDS_Client::update_topic(sensor_msgs_msg_BatteryState& msg, const uint8_t instance)
{
    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES) {
        return;
    }
    static_assert(AP_BATT_MONITOR_MAX_INSTANCES <= 99, "AP_BATT_MONITOR_MAX_INSTANCES is greater than 99");

    update_topic(msg.header.stamp);
    hal.util->snprintf(msg.header.frame_id, 2, "%u", instance);

    auto &battery = AP::battery();

    if (!battery.healthy(instance)) {
        msg.power_supply_status = 3; //POWER_SUPPLY_HEALTH_DEAD
        msg.present = false;
        return;
    }
    msg.present = true;

    msg.voltage = battery.voltage(instance);

    float temperature;
    msg.temperature = (battery.get_temperature(temperature, instance)) ? temperature : NAN;

    float current;
    msg.current = (battery.current_amps(current, instance)) ? -1 * current : NAN;

    const float design_capacity = (float)battery.pack_capacity_mah(instance) * 0.001;
    msg.design_capacity = design_capacity;

    uint8_t percentage;
    if (battery.capacity_remaining_pct(percentage, instance)) {
        msg.percentage = percentage * 0.01;
        msg.charge = (percentage * design_capacity) * 0.01;
    } else {
        msg.percentage = NAN;
        msg.charge = NAN;
    }

    msg.capacity = NAN;

    if (battery.current_amps(current, instance)) {
        if (percentage == 100) {
            msg.power_supply_status = 4;   //POWER_SUPPLY_STATUS_FULL
        } else if (current < 0.0) {
            msg.power_supply_status = 1;   //POWER_SUPPLY_STATUS_CHARGING
        } else if (current > 0.0) {
            msg.power_supply_status = 2;   //POWER_SUPPLY_STATUS_DISCHARGING
        } else {
            msg.power_supply_status = 3;   //POWER_SUPPLY_STATUS_NOT_CHARGING
        }
    } else {
        msg.power_supply_status = 0; //POWER_SUPPLY_STATUS_UNKNOWN
    }

    msg.power_supply_health = 1; //POWER_SUPPLY_HEALTH_OVERVOLTAGE or POWER_SUPPLY_HEALTH_GOOD

    msg.power_supply_technology = 0; //POWER_SUPPLY_TECHNOLOGY_UNKNOWN

    if (battery.has_cell_voltages(instance)) {
        const auto &cells = battery.get_cell_voltages(instance);
        const uint8_t ncells_max = MIN(ARRAY_SIZE(msg.cell_voltage), ARRAY_SIZE(cells.cells));
        for (uint8_t i=0; i< ncells_max; i++) {
            msg.cell_voltage[i] = cells.cells[i] * 0.001;
        }
    }
}
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED

#if AP_DDS_LOCAL_POSE_PUB_ENABLED
bool AP_DDS_Client::update_topic(geometry_msgs_msg_PoseStamped& msg)
{
    set_runtime_phase(RuntimePhase::AHRS_SNAPSHOT);
    auto &ahrs = AP::ahrs();
    auto &ahrs_semaphore = ahrs.get_semaphore();
    if (!take_topic_semaphore(ahrs_semaphore)) {
        return false;
    }
    ScopedSemaphoreRelease ahrs_lock(ahrs_semaphore);

    update_topic(msg.header.stamp);
    STRCPY(msg.header.frame_id, BASE_LINK_FRAME_ID);

    // ROS REP 103 uses the ENU convention:
    // X - East
    // Y - North
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // AP_AHRS uses the NED convention
    // X - North
    // Y - East
    // Z - Down
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z

    Vector3f position;
    if (ahrs.get_relative_position_NED_home(position)) {
        msg.pose.position.x = position[1];
        msg.pose.position.y = position[0];
        msg.pose.position.z = -position[2];
    }

    // In ROS REP 103, axis orientation uses the following convention:
    // X - Forward
    // Y - Left
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z (NED to ENU conversion) as well as a 90 degree rotation in the Z axis
    // for x to point forward
    Quaternion orientation;
    if (ahrs.get_quaternion(orientation)) {
        Quaternion aux(orientation[0], orientation[2], orientation[1], -orientation[3]); //NED to ENU transformation
        Quaternion transformation (sqrtF(2) * 0.5,0,0,sqrtF(2) * 0.5); // Z axis 90 degree rotation
        orientation = aux * transformation;
        msg.pose.orientation.w = orientation[0];
        msg.pose.orientation.x = orientation[1];
        msg.pose.orientation.y = orientation[2];
        msg.pose.orientation.z = orientation[3];
    } else {
        initialize(msg.pose.orientation);
    }
    return true;
}
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED

#if AP_DDS_LOCAL_VEL_PUB_ENABLED
bool AP_DDS_Client::update_topic(geometry_msgs_msg_TwistStamped& msg)
{
    set_runtime_phase(RuntimePhase::AHRS_SNAPSHOT);
    auto &ahrs = AP::ahrs();
    auto &ahrs_semaphore = ahrs.get_semaphore();
    if (!take_topic_semaphore(ahrs_semaphore)) {
        return false;
    }
    ScopedSemaphoreRelease ahrs_lock(ahrs_semaphore);

    update_topic(msg.header.stamp);
    STRCPY(msg.header.frame_id, BASE_LINK_FRAME_ID);

    // ROS REP 103 uses the ENU convention:
    // X - East
    // Y - North
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // AP_AHRS uses the NED convention
    // X - North
    // Y - East
    // Z - Down
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        msg.twist.linear.x = velocity[1];
        msg.twist.linear.y = velocity[0];
        msg.twist.linear.z = -velocity[2];
    }

    // In ROS REP 103, axis orientation uses the following convention:
    // X - Forward
    // Y - Left
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // The gyro data is received from AP_AHRS in body-frame
    // X - Forward
    // Y - Right
    // Z - Down
    // As a consequence, to follow ROS REP 103, it is necessary to invert Y and Z
    Vector3f angular_velocity = ahrs.get_gyro();
    msg.twist.angular.x = angular_velocity[0];
    msg.twist.angular.y = -angular_velocity[1];
    msg.twist.angular.z = -angular_velocity[2];
    return true;
}
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_GEOPOSE_PUB_ENABLED
bool AP_DDS_Client::update_topic(geographic_msgs_msg_GeoPoseStamped& msg)
{
    set_runtime_phase(RuntimePhase::AHRS_SNAPSHOT);
    auto &ahrs = AP::ahrs();
    auto &ahrs_semaphore = ahrs.get_semaphore();
    if (!take_topic_semaphore(ahrs_semaphore)) {
        return false;
    }
    ScopedSemaphoreRelease ahrs_lock(ahrs_semaphore);

    update_topic(msg.header.stamp);
    STRCPY(msg.header.frame_id, BASE_LINK_FRAME_ID);

    Location loc;
    if (ahrs.get_location(loc)) {
        msg.pose.position.latitude = loc.lat * 1E-7;
        msg.pose.position.longitude = loc.lng * 1E-7;
        // TODO this is assumed to be absolute frame in WGS-84 as per the GeoPose message definition in ROS.
        // Use loc.get_alt_frame() to convert if necessary.
        msg.pose.position.altitude = loc.alt * 0.01; // Transform from cm to m
    }

    // In ROS REP 103, axis orientation uses the following convention:
    // X - Forward
    // Y - Left
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z (NED to ENU conversion) as well as a 90 degree rotation in the Z axis
    // for x to point forward
    Quaternion orientation;
    if (ahrs.get_quaternion(orientation)) {
        Quaternion aux(orientation[0], orientation[2], orientation[1], -orientation[3]); //NED to ENU transformation
        Quaternion transformation(sqrtF(2) * 0.5, 0, 0, sqrtF(2) * 0.5); // Z axis 90 degree rotation
        orientation = aux * transformation;
        msg.pose.orientation.w = orientation[0];
        msg.pose.orientation.x = orientation[1];
        msg.pose.orientation.y = orientation[2];
        msg.pose.orientation.z = orientation[3];
    } else {
        initialize(msg.pose.orientation);
    }
    return true;
}
#endif // AP_DDS_GEOPOSE_PUB_ENABLED

#if AP_DDS_IMU_PUB_ENABLED
bool AP_DDS_Client::update_topic(sensor_msgs_msg_Imu& msg)
{
    set_runtime_phase(RuntimePhase::AHRS_SNAPSHOT);
    auto &imu = AP::ins();
    auto &ahrs = AP::ahrs();
    auto &ahrs_semaphore = ahrs.get_semaphore();
    if (!take_topic_semaphore(ahrs_semaphore)) {
        return false;
    }
    ScopedSemaphoreRelease ahrs_lock(ahrs_semaphore);

    update_topic(msg.header.stamp);
    STRCPY(msg.header.frame_id, BASE_LINK_NED_FRAME_ID);

    Quaternion orientation;
    const bool have_orientation = ahrs.get_quaternion(orientation);
    if (have_orientation) {
        // AP_Math::Quaternion stores components as w, x, y, z while
        // geometry_msgs/Quaternion uses x, y, z, w.
        msg.orientation.x = orientation[1];
        msg.orientation.y = orientation[2];
        msg.orientation.z = orientation[3];
        msg.orientation.w = orientation[0];
    } else {
        initialize(msg.orientation);
    }

    Matrix3f orientation_covariance;
    if (have_orientation && ahrs.get_orientation_covariance(orientation_covariance)) {
        copy_covariance(orientation_covariance, msg.orientation_covariance);
    } else if (have_orientation) {
        // Per sensor_msgs/Imu, all zeros means covariance unknown.
        set_unknown_covariance(msg.orientation_covariance);
    } else {
        // Per sensor_msgs/Imu, -1 in element 0 means no orientation estimate.
        set_unavailable_covariance(msg.orientation_covariance);
    }

    uint8_t accel_index = ahrs.get_primary_accel_index();
    uint8_t gyro_index = ahrs.get_primary_gyro_index();
    const bool have_accel = imu.get_accel_health(accel_index);
    const bool have_gyro = imu.get_gyro_health(gyro_index);

    Vector3f accel_data;
    Vector3f gyro_data;
    if (have_accel) {
        accel_data = imu.get_accel(accel_index);
    }
    if (have_gyro) {
        gyro_data = imu.get_gyro(gyro_index);
    }

    // Populate the message fields
    msg.linear_acceleration.x = accel_data.x;
    msg.linear_acceleration.y = accel_data.y;
    msg.linear_acceleration.z = accel_data.z;

    msg.angular_velocity.x = gyro_data.x;
    msg.angular_velocity.y = gyro_data.y;
    msg.angular_velocity.z = gyro_data.z;

    Vector3f gyro_variances;
    Vector3f accel_variances;
    const bool have_imu_covariance = ahrs.get_imu_noise_variances(gyro_variances, accel_variances);
    if (have_imu_covariance && have_gyro) {
        copy_covariance(diagonal_covariance(gyro_variances), msg.angular_velocity_covariance);
    } else if (have_gyro) {
        set_unknown_covariance(msg.angular_velocity_covariance);
    } else {
        set_unavailable_covariance(msg.angular_velocity_covariance);
    }

    if (have_imu_covariance && have_accel) {
        // Add measured accel vibration energy on top of the EKF-configured noise floor.
        const Vector3f vibration = imu.get_vibration_levels(accel_index);
        accel_variances.x += sq(vibration.x);
        accel_variances.y += sq(vibration.y);
        accel_variances.z += sq(vibration.z);

        copy_covariance(diagonal_covariance(accel_variances), msg.linear_acceleration_covariance);
    } else if (have_accel) {
        set_unknown_covariance(msg.linear_acceleration_covariance);
    } else {
        set_unavailable_covariance(msg.linear_acceleration_covariance);
    }
    return true;
}
#endif // AP_DDS_IMU_PUB_ENABLED

#if AP_DDS_UWB_PUB_ENABLED
bool AP_DDS_Client::update_topic(sensor_msgs_msg_uwb& msg)
{
    auto* alx = AP::alx_sensor();
    if (alx == nullptr) {
        return false;
    }

    Location target_loc;
    if (!alx->get_target_lat_lng(target_loc)) {
        return false;
    }

    msg.latitude = target_loc.lat * 1E-7;
    msg.longitude = target_loc.lng * 1E-7;

    int32_t alt_cm;
    if (target_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
        msg.altitude = alt_cm * 0.01;
    } else {
        msg.altitude = target_loc.alt * 0.01;
    }

    return true;
}
#endif // AP_DDS_UWB_PUB_ENABLED

#if AP_DDS_CLOCK_PUB_ENABLED
void AP_DDS_Client::update_topic(rosgraph_msgs_msg_Clock& msg)
{
    update_topic(msg.clock);
}
#endif // AP_DDS_CLOCK_PUB_ENABLED

#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
bool AP_DDS_Client::update_topic(geographic_msgs_msg_GeoPointStamped& msg)
{
    set_runtime_phase(RuntimePhase::AHRS_SNAPSHOT);
    auto &ahrs = AP::ahrs();
    auto &ahrs_semaphore = ahrs.get_semaphore();
    if (!take_topic_semaphore(ahrs_semaphore)) {
        return false;
    }
    ScopedSemaphoreRelease ahrs_lock(ahrs_semaphore);

    update_topic(msg.header.stamp);
    STRCPY(msg.header.frame_id, BASE_LINK_FRAME_ID);

    Location ekf_origin;
    // LLA is WGS-84 geodetic coordinate.
    // Altitude converted from cm to m.
    if (ahrs.get_origin(ekf_origin)) {
        msg.position.latitude = ekf_origin.lat * 1E-7;
        msg.position.longitude = ekf_origin.lng * 1E-7;
        msg.position.altitude = ekf_origin.alt * 0.01;
    }
    return true;
}
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED

/*
  start the DDS thread
 */
bool AP_DDS_Client::start(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::load_object_from_eeprom(this, var_info);

    if (enabled == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s disabled by DDS_ENABLE", msg_prefix);
        return true;
    }

    set_runtime_phase(RuntimePhase::IDLE);
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_DDS_Client::runtime_supervisor, void),
                                      "DDS-watch",
                                      2048, AP_HAL::Scheduler::PRIORITY_IO, 2)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s watchdog thread failed", msg_prefix);
        return false;
    }
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_DDS_Client::main_loop, void),
                                      "DDS",
                                      8192, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s thread create failed", msg_prefix);
        return false;
    }
    return true;
}

// read function triggered at every subscription callback
void AP_DDS_Client::on_topic_trampoline(uxrSession* uxr_session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length,
                                        void* args)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)args;
    const RuntimePhase previous_phase = static_cast<RuntimePhase>(dds->runtime_phase.load());
    dds->set_runtime_phase(RuntimePhase::TOPIC_CALLBACK);
    dds->on_topic(uxr_session, object_id, request_id, stream_id, ub, length);
    dds->set_runtime_phase(previous_phase);
}

void AP_DDS_Client::on_topic(uxrSession* uxr_session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length)
{
    /*
    TEMPLATE for reading to the subscribed topics
    1) Store the read contents into the ucdr buffer
    2) Deserialize the said contents into the topic instance
    */
    (void) uxr_session;
    (void) request_id;
    (void) stream_id;
    switch (object_id.id) {
#if AP_DDS_JOY_SUB_ENABLED
    case topics[to_underlying(TopicIndex::JOY_SUB)].dr_id.id: {
        if (!input_sem.take_nonblocking()) {
            break;
        }
        ScopedSemaphoreRelease input_lock(input_sem);
        if (sensor_msgs_msg_Joy_deserialize_topic(ub, &rx_joy_topic)) {
            joy_receive_time_ms = AP_HAL::millis();
            joy_epoch = session_epoch.load();
            joy_pending = true;
            topic_input_pending.store(true);
        }
        break;
    }
#endif // AP_DDS_JOY_SUB_ENABLED
#if AP_DDS_DYNAMIC_TF_SUB_ENABLED
    case topics[to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB)].dr_id.id: {
        if (!input_sem.take_nonblocking()) {
            break;
        }
        ScopedSemaphoreRelease input_lock(input_sem);
        if (tf2_msgs_msg_TFMessage_deserialize_topic(ub, &rx_dynamic_transforms_topic)) {
            dynamic_transforms_receive_time_ms = AP_HAL::millis();
            dynamic_transforms_epoch = session_epoch.load();
            dynamic_transforms_pending = true;
            topic_input_pending.store(true);
        }
        break;
    }
#endif // AP_DDS_DYNAMIC_TF_SUB_ENABLED
#if AP_DDS_VEL_CTRL_ENABLED
    case topics[to_underlying(TopicIndex::VELOCITY_CONTROL_SUB)].dr_id.id: {
        if (!input_sem.take_nonblocking()) {
            break;
        }
        ScopedSemaphoreRelease input_lock(input_sem);
        if (geometry_msgs_msg_TwistStamped_deserialize_topic(ub, &rx_velocity_control_topic)) {
            velocity_control_epoch = session_epoch.load();
            velocity_control_pending = true;
            topic_input_pending.store(true);
        }
        break;
    }
#endif // AP_DDS_VEL_CTRL_ENABLED
#if AP_DDS_EXTNAV_VEL_SUB_ENABLED
    case topics[to_underlying(TopicIndex::EXTNAV_VELOCITY_SUB)].dr_id.id: {
        if (!input_sem.take_nonblocking()) {
            break;
        }
        ScopedSemaphoreRelease input_lock(input_sem);
        if (geometry_msgs_msg_TwistStamped_deserialize_topic(ub, &rx_extnav_velocity_topic)) {
            extnav_velocity_receive_time_ms = AP_HAL::millis();
            extnav_velocity_epoch = session_epoch.load();
            extnav_velocity_pending = true;
            topic_input_pending.store(true);
        }
        break;
    }
#endif // AP_DDS_EXTNAV_VEL_SUB_ENABLED
#if AP_DDS_GLOBAL_POS_CTRL_ENABLED
    case topics[to_underlying(TopicIndex::GLOBAL_POSITION_SUB)].dr_id.id: {
        if (!input_sem.take_nonblocking()) {
            break;
        }
        ScopedSemaphoreRelease input_lock(input_sem);
        if (ardupilot_msgs_msg_GlobalPosition_deserialize_topic(ub, &rx_global_position_control_topic)) {
            global_position_control_epoch = session_epoch.load();
            global_position_control_pending = true;
            topic_input_pending.store(true);
        }
        break;
    }
#endif // AP_DDS_GLOBAL_POS_CTRL_ENABLED
    }

}

void AP_DDS_Client::update_main_thread()
{
    process_service_request_main_thread();

    if (!topic_input_pending.load() || !input_sem.take_nonblocking()) {
        return;
    }
    ScopedSemaphoreRelease input_lock(input_sem);
    const uint32_t current_epoch = session_epoch.load();

#if AP_DDS_JOY_SUB_ENABLED
    if (joy_pending) {
        if (joy_epoch == current_epoch && rx_joy_topic.axes_size >= 4) {
            for (uint8_t i = 0; i < MIN(8U, rx_joy_topic.axes_size); i++) {
                if (std::isnan(rx_joy_topic.axes[i])) {
                    RC_Channels::set_override(i, 0U, joy_receive_time_ms);
                } else {
                    const uint16_t mapped_data = static_cast<uint16_t>(
                        linear_interpolate(rc().channel(i)->get_radio_min(),
                                           rc().channel(i)->get_radio_max(),
                                           rx_joy_topic.axes[i],
                                           -1.0, 1.0));
                    RC_Channels::set_override(i, mapped_data, joy_receive_time_ms);
                }
            }
        }
        joy_pending = false;
    }
#endif

#if AP_DDS_DYNAMIC_TF_SUB_ENABLED
    if (dynamic_transforms_pending) {
        if (dynamic_transforms_epoch == current_epoch &&
            rx_dynamic_transforms_topic.transforms_size > 0) {
            (void)AP_DDS_ExternalNav::handle_tf(rx_dynamic_transforms_topic,
                                                dynamic_transforms_receive_time_ms);
        }
        dynamic_transforms_pending = false;
    }
#endif

#if AP_DDS_VEL_CTRL_ENABLED
    if (velocity_control_pending) {
#if AP_EXTERNAL_CONTROL_ENABLED
        if (velocity_control_epoch == current_epoch) {
            (void)AP_DDS_External_Control::handle_velocity_control(rx_velocity_control_topic);
        }
#endif
        velocity_control_pending = false;
    }
#endif

#if AP_DDS_EXTNAV_VEL_SUB_ENABLED
    if (extnav_velocity_pending) {
        if (extnav_velocity_epoch == current_epoch) {
            const int16_t configured_delay_ms = extnav_velocity_delay_ms.get();
            const uint16_t delay_ms = configured_delay_ms > 0 ? uint16_t(configured_delay_ms) : 0U;
            (void)AP_DDS_ExternalNav::handle_velocity(rx_extnav_velocity_topic,
                                                      extnav_velocity_error.get(),
                                                      delay_ms,
                                                      extnav_velocity_receive_time_ms);
        }
        extnav_velocity_pending = false;
    }
#endif

#if AP_DDS_GLOBAL_POS_CTRL_ENABLED
    if (global_position_control_pending) {
#if AP_EXTERNAL_CONTROL_ENABLED
        if (global_position_control_epoch == current_epoch) {
            (void)AP_DDS_External_Control::handle_global_position_control(rx_global_position_control_topic);
        }
#endif
        global_position_control_pending = false;
    }
#endif

    topic_input_pending.store(false);
}

/*
  callback on request completion
 */
void AP_DDS_Client::on_request_trampoline(uxrSession* uxr_session, uxrObjectId object_id, uint16_t request_id, SampleIdentity* sample_id, ucdrBuffer* ub, uint16_t length, void* args)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)args;
    const RuntimePhase previous_phase = static_cast<RuntimePhase>(dds->runtime_phase.load());
    dds->set_runtime_phase(RuntimePhase::SERVICE_CALLBACK);
    dds->on_request(uxr_session, object_id, request_id, sample_id, ub, length);
    dds->set_runtime_phase(previous_phase);
}

void AP_DDS_Client::on_request(uxrSession* uxr_session, uxrObjectId object_id, uint16_t request_id, SampleIdentity* sample_id, ucdrBuffer* ub, uint16_t length)
{
    (void)request_id;
    (void)length;
    if (sample_id == nullptr) {
        return;
    }

    switch (object_id.id) {
#if AP_DDS_ARM_SERVER_ENABLED
    case services[to_underlying(ServiceIndex::ARMING_MOTORS)].rep_id: {
        const uint8_t replier_id = services[to_underlying(ServiceIndex::ARMING_MOTORS)].rep_id;
        if (!begin_service_request(ServiceCommand::ARM, replier_id, *sample_id)) {
            send_service_failure(uxr_session, ServiceCommand::ARM, replier_id, *sample_id);
            break;
        }
        ardupilot_msgs_srv_ArmMotors_Request request {};
        if (!ardupilot_msgs_srv_ArmMotors_Request_deserialize_topic(ub, &request)) {
            cancel_service_request();
            break;
        }
        pending_service.arm = request.arm;
        commit_service_request();
        break;
    }
#endif // AP_DDS_ARM_SERVER_ENABLED
#if AP_DDS_MODE_SWITCH_SERVER_ENABLED
    case services[to_underlying(ServiceIndex::MODE_SWITCH)].rep_id: {
        const uint8_t replier_id = services[to_underlying(ServiceIndex::MODE_SWITCH)].rep_id;
        if (!begin_service_request(ServiceCommand::MODE_SWITCH, replier_id, *sample_id)) {
            send_service_failure(uxr_session, ServiceCommand::MODE_SWITCH, replier_id, *sample_id);
            break;
        }
        ardupilot_msgs_srv_ModeSwitch_Request request {};
        if (!ardupilot_msgs_srv_ModeSwitch_Request_deserialize_topic(ub, &request)) {
            cancel_service_request();
            break;
        }
        pending_service.mode = request.mode;
        commit_service_request();
        break;
    }
#endif // AP_DDS_MODE_SWITCH_SERVER_ENABLED
#if AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
    case services[to_underlying(ServiceIndex::TAKEOFF)].rep_id: {
        const uint8_t replier_id = services[to_underlying(ServiceIndex::TAKEOFF)].rep_id;
        if (!begin_service_request(ServiceCommand::TAKEOFF, replier_id, *sample_id)) {
            send_service_failure(uxr_session, ServiceCommand::TAKEOFF, replier_id, *sample_id);
            break;
        }
        ardupilot_msgs_srv_Takeoff_Request request {};
        if (!ardupilot_msgs_srv_Takeoff_Request_deserialize_topic(ub, &request)) {
            cancel_service_request();
            break;
        }
        pending_service.takeoff_alt = request.alt;
        commit_service_request();
        break;
    }
#endif // AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
#if AP_DDS_ARM_CHECK_SERVER_ENABLED
    case services[to_underlying(ServiceIndex::PREARM_CHECK)].rep_id: {
        const uint8_t replier_id = services[to_underlying(ServiceIndex::PREARM_CHECK)].rep_id;
        if (!begin_service_request(ServiceCommand::PREARM_CHECK, replier_id, *sample_id)) {
            send_service_failure(uxr_session, ServiceCommand::PREARM_CHECK, replier_id, *sample_id);
            break;
        }
        std_srvs_srv_Trigger_Request request {};
        if (!std_srvs_srv_Trigger_Request_deserialize_topic(ub, &request)) {
            cancel_service_request();
            break;
        }
        commit_service_request();
        break;
    }
#endif //AP_DDS_ARM_CHECK_SERVER_ENABLED
#if AP_DDS_PARAMETER_SERVER_ENABLED
    case services[to_underlying(ServiceIndex::SET_PARAMETERS)].rep_id: {
        const uint8_t replier_id = services[to_underlying(ServiceIndex::SET_PARAMETERS)].rep_id;
        if (!begin_service_request(ServiceCommand::SET_PARAMETERS, replier_id, *sample_id)) {
            send_service_failure(uxr_session, ServiceCommand::SET_PARAMETERS, replier_id, *sample_id);
            break;
        }
        if (!rcl_interfaces_srv_SetParameters_Request_deserialize_topic(ub, &set_parameter_request) ||
            set_parameter_request.parameters_size > 8U) {
            cancel_service_request();
            send_service_failure(uxr_session, ServiceCommand::SET_PARAMETERS, replier_id, *sample_id);
            break;
        }
        commit_service_request();
        break;
    }
    case services[to_underlying(ServiceIndex::GET_PARAMETERS)].rep_id: {
        const uint8_t replier_id = services[to_underlying(ServiceIndex::GET_PARAMETERS)].rep_id;
        if (!begin_service_request(ServiceCommand::GET_PARAMETERS, replier_id, *sample_id)) {
            send_service_failure(uxr_session, ServiceCommand::GET_PARAMETERS, replier_id, *sample_id);
            break;
        }
        if (!rcl_interfaces_srv_GetParameters_Request_deserialize_topic(ub, &get_parameters_request) ||
            get_parameters_request.names_size > 8U) {
            cancel_service_request();
            send_service_failure(uxr_session, ServiceCommand::GET_PARAMETERS, replier_id, *sample_id);
            break;
        }
        commit_service_request();
        break;
    }
#endif // AP_DDS_PARAMETER_SERVER_ENABLED
    default:
        break;
    }
}

/*
  main loop for DDS thread
 */
void AP_DDS_Client::main_loop(void)
{
    auto reset_transport = [this]() {
        if (comm == nullptr) {
            return;
        }

        set_runtime_phase(RuntimePhase::TRANSPORT_CLOSE);
        if (is_using_serial) {
            uxr_close_custom_transport(&serial.transport);
            serial.port = nullptr;
        }
#if AP_DDS_UDP_ENABLED
        else {
            uxr_close_custom_transport(&udp.transport);
        }
#endif
        comm = nullptr;
    };

    auto retry_delay = [this](uint32_t delay_ms) {
        set_runtime_phase(RuntimePhase::BACKOFF, delay_ms + RUNTIME_OPERATION_SLACK_MS);
        hal.scheduler->delay(delay_ms);
    };

    bool ping_failure_reported = false;
    runtime_owner_active.store(true);

    //! @todo check for request to stop task
    while (true) {
        discard_stale_service_request();

        if (comm == nullptr) {
            set_runtime_phase(RuntimePhase::TRANSPORT_OPEN);
            if (!init_transport()) {
                retry_delay(SERIAL_RETRY_DELAY_MS);
                continue;
            }
            if (is_using_serial) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "%s Serial transport SERIAL%d %lu baud",
                              msg_prefix,
                              (int)serial.port_num,
                              (unsigned long)serial.baud);
            }
        }

        // check ping
        bool ping_ok = false;
        const uint32_t configured_ping_timeout_ms =
            (uint32_t)constrain_int32((int32_t)ping_timeout_ms, 1, 10000);
        const int8_t configured_retries = ping_max_retry.get();
        const uint8_t startup_ping_attempts =
            (is_using_serial || configured_retries <= 0) ? 1U : (uint8_t)configured_retries;
        set_runtime_phase(RuntimePhase::STARTUP_PING,
                          configured_ping_timeout_ms * startup_ping_attempts +
                          RUNTIME_OPERATION_SLACK_MS);
        if (is_using_serial) {
            ping_ok = uxr_ping_agent_attempts(comm, (int)configured_ping_timeout_ms, 1);
        } else if (ping_max_retry == 0) {
            ping_ok = uxr_ping_agent(comm, ping_timeout_ms);
        } else {
            ping_ok = uxr_ping_agent_attempts(comm, ping_timeout_ms, ping_max_retry);
        }
        set_runtime_phase(RuntimePhase::IDLE);

        if (consume_reconnect_request() != ReconnectReason::NONE) {
            reset_transport();
            retry_delay(is_using_serial ? SERIAL_RETRY_DELAY_MS : 200U);
            continue;
        }

        if (!ping_ok) {
            if (!ping_failure_reported) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s No ping response, retrying", msg_prefix);
                ping_failure_reported = true;
            }
            if (is_using_serial) {
                // Keep the open UART and framing state for the next bounded
                // startup ping.  Reopen only after a Session-level failure.
                retry_delay(SERIAL_RETRY_DELAY_MS);
            }
            continue;
        }
        ping_failure_reported = false;

        // create session
        set_runtime_phase(RuntimePhase::SESSION_CREATE,
                          RUNTIME_DEFAULT_ALLOWED_MS + RUNTIME_OPERATION_SLACK_MS);
        if (!init_session()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Session create failed, reconnecting", msg_prefix);
            cleanup_session(false);
            reset_transport();
            retry_delay(is_using_serial ? SERIAL_RETRY_DELAY_MS : 200U);
            continue;
        }

        if (consume_reconnect_request() != ReconnectReason::NONE) {
            cleanup_session(false);
            reset_transport();
            retry_delay(is_using_serial ? SERIAL_RETRY_DELAY_MS : 200U);
            continue;
        }

        set_runtime_phase(RuntimePhase::ENTITY_CREATE,
                          RUNTIME_DEFAULT_ALLOWED_MS + RUNTIME_OPERATION_SLACK_MS);
        if (!create()) {
            cleanup_session(true);
            reset_transport();
            retry_delay(is_using_serial ? SERIAL_RETRY_DELAY_MS : 200U);
            continue;
        }

        if (consume_reconnect_request() != ReconnectReason::NONE) {
            cleanup_session(false);
            reset_transport();
            retry_delay(is_using_serial ? SERIAL_RETRY_DELAY_MS : 200U);
            continue;
        }

        connected = true;
        last_rx_activity_ms = AP_HAL::millis64();
#if AP_DDS_TIME_PUB_ENABLED
        time_ack_pending = false;
        last_time_ack_ms = AP_HAL::millis();
#endif
        set_runtime_phase(RuntimePhase::TOPIC_UPDATE);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Initialization passed", msg_prefix);
#if AP_DDS_STATIC_TF_PUB_ENABLED
        populate_static_transforms(tx_static_transforms_topic);
        write_static_transforms();
#endif // AP_DDS_STATIC_TF_PUB_ENABLED
        set_runtime_phase(RuntimePhase::IDLE);
        uint64_t last_ping_ms{0};
        uint8_t num_pings_missed{0};
        uint64_t status_fail_start_ms{0};
        while (connected) {
            if (consume_reconnect_request() != ReconnectReason::NONE) {
                connected = false;
                break;
            }

            hal.scheduler->delay(1);

            // publish topics
            update();
            drain_service_reply();

            if (consume_reconnect_request() != ReconnectReason::NONE) {
                connected = false;
                break;
            }

            const auto cur_time_ms = AP_HAL::millis64();
            uint32_t status_fail_ms = 0;
            if (!is_using_serial) {
                if (status_ok) {
                    status_fail_start_ms = 0;
                } else if (status_fail_start_ms == 0) {
                    status_fail_start_ms = cur_time_ms;
                }
                status_fail_ms = (status_fail_start_ms == 0) ? 0 :
                                 (uint32_t)MIN<uint64_t>(cur_time_ms - status_fail_start_ms, UINT32_MAX);
            }
            const bool recent_rx_activity = (last_rx_activity_ms != 0) &&
                                            ((cur_time_ms - last_rx_activity_ms) < RX_ACTIVITY_GRACE_MS);

#if AP_DDS_TIME_PUB_ENABLED
            if (is_using_serial &&
                (uint32_t)(AP_HAL::millis() - last_time_ack_ms) >= STATUS_FAIL_TIMEOUT_MS) {
                const uxrSeqNum latest_ack =
                    reliable_out.index < session.streams.output_reliable_size ?
                    session.streams.output_reliable[reliable_out.index].last_acknown : 0;
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                              "%s Time ACK timeout p=%u tx=%u ack=%u",
                              msg_prefix,
                              (unsigned)time_ack_pending,
                              (unsigned)time_ack_sequence,
                              (unsigned)latest_ack);
                connected = false;
                break;
            }
#endif

            // For serial, always check session health directly so an agent restart
            // is detected even when the transport itself is still reachable.
            const bool should_ping_session = is_using_serial || AP_DDS_SESSION_PING_ENABLED;
            const bool should_ping_agent = !is_using_serial &&
                                           !AP_DDS_SESSION_PING_ENABLED &&
                                           !recent_rx_activity;
            if ((should_ping_session || should_ping_agent) &&
                (cur_time_ms - last_ping_ms > DELAY_PING_MS)) {
                last_ping_ms = cur_time_ms;

                const int ping_agent_timeout_ms = MAX(1, (int)MIN((uint32_t)ping_timeout_ms, (uint32_t)DELAY_PING_MS));
                const uint8_t ping_agent_attempts{1};
                set_runtime_phase(RuntimePhase::HEALTH_PING,
                                  (uint32_t)ping_agent_timeout_ms + RUNTIME_OPERATION_SLACK_MS);
                const bool session_ping_ok = should_ping_session
                                                 ? uxr_ping_agent_session(&session, ping_agent_timeout_ms, ping_agent_attempts)
                                                 : uxr_ping_agent_attempts(comm, ping_agent_timeout_ms, ping_agent_attempts);
                const uint64_t ping_complete_ms = AP_HAL::millis64();
                set_runtime_phase(RuntimePhase::IDLE);
                if (session_ping_ok) {
                    num_pings_missed = 0;
                    last_rx_activity_ms = ping_complete_ms;
                } else if (is_using_serial || !recent_rx_activity) {
                    if (num_pings_missed < UINT8_MAX) {
                        ++num_pings_missed;
                    }
                } else {
                    // Preserve the existing non-serial activity grace period.
                    num_pings_missed = 0;
                }
            }

            if (num_pings_missed >= MAX_MISSED_PING_COUNT) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                              "%s No ping response, disconnecting", msg_prefix);
                connected = false;
            } else if (status_fail_ms >= STATUS_FAIL_TIMEOUT_MS) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                              "%s Session status timeout, disconnecting", msg_prefix);
                connected = false;
            }
        }

        cleanup_session(false);
        reset_transport();
        if (is_using_serial) {
            retry_delay(SERIAL_RETRY_DELAY_MS);
        }
    }
}

bool AP_DDS_Client::init_transport()
{
    // serial init will fail if the SERIALn_PROTOCOL is not setup
    is_using_serial = ddsSerialInit();

    if (!is_using_serial) {
        return false;
    }

    return true;
}

bool AP_DDS_Client::init_session()
{
    cleanup_session(false);
    set_runtime_phase(RuntimePhase::SESSION_CREATE,
                      RUNTIME_DEFAULT_ALLOWED_MS + RUNTIME_OPERATION_SLACK_MS);

    // init session
    uxr_init_session(&session, comm, key);

    // Register topic callbacks
    uxr_set_topic_callback(&session, AP_DDS_Client::on_topic_trampoline, this);

    // ROS-2 Service : Register service request callbacks
    uxr_set_request_callback(&session, AP_DDS_Client::on_request_trampoline, this);

    if (is_using_serial) {
        if (!uxr_create_session(&session)) {
            return false;
        }
    } else {
        while (!uxr_create_session(&session)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Initialization waiting...", msg_prefix);
            hal.scheduler->delay(1000);
        }
    }
    session_created = true;

    // setup reliable stream buffers
    input_reliable_stream = NEW_NOTHROW uint8_t[DDS_BUFFER_SIZE];
    output_reliable_stream = NEW_NOTHROW uint8_t[DDS_BUFFER_SIZE];
    output_best_effort_stream = NEW_NOTHROW uint8_t[DDS_BEST_EFFORT_BUFFER_SIZE];
    if (input_reliable_stream == nullptr || output_reliable_stream == nullptr || output_best_effort_stream == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Allocation failed", msg_prefix);
        cleanup_session(true);
        return false;
    }

    reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream, DDS_BUFFER_SIZE, DDS_STREAM_HISTORY);
    reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream, DDS_BUFFER_SIZE, DDS_STREAM_HISTORY);
    best_effort_in = uxr_create_input_best_effort_stream(&session);
    best_effort_out = uxr_create_output_best_effort_stream(&session, output_best_effort_stream, DDS_BEST_EFFORT_BUFFER_SIZE);

    return true;
}

bool AP_DDS_Client::create()
{
    set_runtime_phase(RuntimePhase::ENTITY_CREATE,
                      RUNTIME_DEFAULT_ALLOWED_MS + RUNTIME_OPERATION_SLACK_MS);
    WITH_SEMAPHORE(csem);

    // Participant
    const uxrObjectId participant_id = {
        .id = 0x01,
        .type = UXR_PARTICIPANT_ID
    };
    const char* participant_name = AP_DDS_PARTICIPANT_NAME;
    const auto participant_req_id = uxr_buffer_create_participant_bin(&session, reliable_out, participant_id,
                                    static_cast<uint16_t>(domain_id), participant_name, UXR_REPLACE);

    //Participant requests
    constexpr uint8_t nRequestsParticipant = 1;
    const uint16_t requestsParticipant[nRequestsParticipant] = {participant_req_id};

    const uint16_t maxTimeMsPerRequestMs = is_using_serial ? 1000 : 300;
    const uint16_t requestTimeoutParticipantMs = (uint16_t) nRequestsParticipant * maxTimeMsPerRequestMs;
    uint8_t statusParticipant[nRequestsParticipant];
    set_runtime_phase(RuntimePhase::ENTITY_CREATE,
                      requestTimeoutParticipantMs + RUNTIME_OPERATION_SLACK_MS);
    if (!uxr_run_session_until_all_status(&session, requestTimeoutParticipantMs, requestsParticipant, statusParticipant, nRequestsParticipant)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                      "%s Entity create failed: participant (%u)",
                      msg_prefix,
                      (unsigned)statusParticipant[0]);
        return false;
    }

    for (uint16_t i = 0 ; i < ARRAY_SIZE(topics); i++) {
        // Topic
        const uxrObjectId topic_id = {
            .id = topics[i].topic_id,
            .type = UXR_TOPIC_ID
        };
        const auto topic_req_id = uxr_buffer_create_topic_bin(&session, reliable_out, topic_id,
                                  participant_id, topics[i].topic_name, topics[i].type_name, UXR_REPLACE);

        // Status requests
        constexpr uint8_t nRequests = 3;
        uint16_t requests[nRequests];
        const uint16_t requestTimeoutMs = nRequests * maxTimeMsPerRequestMs;
        uint8_t status[nRequests];

        if (topics[i].topic_rw == Topic_rw::DataWriter) {
            // Publisher
            const uxrObjectId pub_id = {
                .id = topics[i].pub_id,
                .type = UXR_PUBLISHER_ID
            };
            const auto pub_req_id = uxr_buffer_create_publisher_bin(&session, reliable_out, pub_id,
                                    participant_id, UXR_REPLACE);

            // Data Writer
            const auto dwriter_req_id = uxr_buffer_create_datawriter_bin(&session, reliable_out, topics[i].dw_id,
                                        pub_id, topic_id, topics[i].qos, UXR_REPLACE);

            // save the request statuses
            requests[0] = topic_req_id;
            requests[1] = pub_req_id;
            requests[2] = dwriter_req_id;

            set_runtime_phase(RuntimePhase::ENTITY_CREATE,
                              requestTimeoutMs + RUNTIME_OPERATION_SLACK_MS);
            if (!uxr_run_session_until_all_status(&session, requestTimeoutMs, requests, status, nRequests)) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                              "%s Entity create failed: writer %u",
                              msg_prefix,
                              (unsigned)i);
                return false;
            }
        } else if (topics[i].topic_rw == Topic_rw::DataReader) {
            // Subscriber
            const uxrObjectId sub_id = {
                .id = topics[i].sub_id,
                .type = UXR_SUBSCRIBER_ID
            };
            const auto sub_req_id = uxr_buffer_create_subscriber_bin(&session, reliable_out, sub_id,
                                    participant_id, UXR_REPLACE);

            // Data Reader
            const auto dreader_req_id = uxr_buffer_create_datareader_bin(&session, reliable_out, topics[i].dr_id,
                                        sub_id, topic_id, topics[i].qos, UXR_REPLACE);

            // save the request statuses
            requests[0] = topic_req_id;
            requests[1] = sub_req_id;
            requests[2] = dreader_req_id;

            set_runtime_phase(RuntimePhase::ENTITY_CREATE,
                              requestTimeoutMs + RUNTIME_OPERATION_SLACK_MS);
            if (!uxr_run_session_until_all_status(&session, requestTimeoutMs, requests, status, nRequests)) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                              "%s Entity create failed: reader %u",
                              msg_prefix,
                              (unsigned)i);
                return false;
            } else {
                uxr_buffer_request_data(&session,
                                        reliable_out,
                                        topics[i].dr_id,
                                        input_stream_for_qos(topics[i].qos),
                                        &delivery_control);
            }
        }
    }

    // ROS-2 Service : else case for service requests

    for (uint16_t i = 0; i < ARRAY_SIZE(services); i++) {

        const uint16_t requestTimeoutMs = maxTimeMsPerRequestMs;

        if (services[i].service_rr == Service_rr::Replier) {
            const uxrObjectId rep_id = {
                .id = services[i].rep_id,
                .type = UXR_REPLIER_ID
            };
            const auto replier_req_id = uxr_buffer_create_replier_bin(&session, reliable_out, rep_id,
                                        participant_id, services[i].service_name, services[i].request_type, services[i].reply_type,
                                        services[i].request_topic_name, services[i].reply_topic_name, services[i].qos, UXR_REPLACE);

            uint16_t request = replier_req_id;
            uint8_t status;

            set_runtime_phase(RuntimePhase::ENTITY_CREATE,
                              requestTimeoutMs + RUNTIME_OPERATION_SLACK_MS);
            if (!uxr_run_session_until_all_status(&session, requestTimeoutMs, &request, &status, 1)) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                              "%s Entity create failed: service %u",
                              msg_prefix,
                              (unsigned)i);
                return false;
            } else {
                uxr_buffer_request_data(&session, reliable_out, rep_id, reliable_in, &delivery_control);
            }

        } else if (services[i].service_rr == Service_rr::Requester) {
            // TODO : Add Similar Code for Requester Profile
        }
    }

    return true;
}

void AP_DDS_Client::write_time_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        const auto& topic_info = topics[to_underlying(TopicIndex::TIME_PUB)];
        ucdrBuffer ub {};
        const uint32_t topic_size = builtin_interfaces_msg_Time_size_of_topic(&time_topic, 0);
        if (!prepare_topic_stream(ub, topic_info.dw_id, topic_size, "time", topic_info.qos)) {
            return;
        }
        const bool success = builtin_interfaces_msg_Time_serialize_topic(&ub, &time_topic);
        if (!success) {
            return;
        }
        if (!time_ack_pending && reliable_out.index < session.streams.output_reliable_size) {
            time_ack_sequence = session.streams.output_reliable[reliable_out.index].last_written;
            time_ack_pending = true;
        }
        finalize_topic_write(topic_info.qos, topic_size);
    }
}

#if AP_DDS_NAVSATFIX_PUB_ENABLED
bool AP_DDS_Client::write_nav_sat_fix_topic()
{
    WITH_SEMAPHORE(csem);
    if (!connected) {
        return false;
    }

    const auto& topic_info = topics[to_underlying(TopicIndex::NAV_SAT_FIX_PUB)];
    ucdrBuffer ub {};
    const uint32_t topic_size = sensor_msgs_msg_NavSatFix_size_of_topic(&nav_sat_fix_topic, 0);
    if (!prepare_topic_stream(ub, topic_info.dw_id, topic_size, "nav", topic_info.qos)) {
        return false;
    }
    const bool success = sensor_msgs_msg_NavSatFix_serialize_topic(&ub, &nav_sat_fix_topic);
    if (!success) {
        return false;
    }
    finalize_topic_write(topic_info.qos, topic_size);
    return true;
}
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED

#if AP_DDS_STATIC_TF_PUB_ENABLED
void AP_DDS_Client::write_static_transforms()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        const auto& topic_info = topics[to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB)];
        ucdrBuffer ub {};
        const uint32_t topic_size = tf2_msgs_msg_TFMessage_size_of_topic(&tx_static_transforms_topic, 0);
        if (!prepare_topic_stream(ub, topic_info.dw_id, topic_size, "tfs", topic_info.qos)) {
            return;
        }
        const bool success = tf2_msgs_msg_TFMessage_serialize_topic(&ub, &tx_static_transforms_topic);
        if (!success) {
            return;
        }
        finalize_topic_write(topic_info.qos, topic_size);
    }
}
#endif // AP_DDS_STATIC_TF_PUB_ENABLED

#if AP_DDS_BATTERY_STATE_PUB_ENABLED
void AP_DDS_Client::write_battery_state_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        const auto& topic_info = topics[to_underlying(TopicIndex::BATTERY_STATE_PUB)];
        ucdrBuffer ub {};
        const uint32_t topic_size = sensor_msgs_msg_BatteryState_size_of_topic(&battery_state_topic, 0);
        if (!prepare_topic_stream(ub, topic_info.dw_id, topic_size, "bat", topic_info.qos)) {
            return;
        }
        const bool success = sensor_msgs_msg_BatteryState_serialize_topic(&ub, &battery_state_topic);
        if (!success) {
            return;
        }
        finalize_topic_write(topic_info.qos, topic_size);
    }
}
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED

#if AP_DDS_LOCAL_POSE_PUB_ENABLED
bool AP_DDS_Client::write_local_pose_topic()
{
    WITH_SEMAPHORE(csem);
    if (!connected) {
        return false;
    }

    const auto& topic_info = topics[to_underlying(TopicIndex::LOCAL_POSE_PUB)];
    ucdrBuffer ub {};
    const uint32_t topic_size = geometry_msgs_msg_PoseStamped_size_of_topic(&local_pose_topic, 0);
    if (!prepare_topic_stream(ub, topic_info.dw_id, topic_size, "pose", topic_info.qos)) {
        return false;
    }
    const bool success = geometry_msgs_msg_PoseStamped_serialize_topic(&ub, &local_pose_topic);
    if (!success) {
        return false;
    }
    finalize_topic_write(topic_info.qos, topic_size);
    return true;
}
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED

#if AP_DDS_LOCAL_VEL_PUB_ENABLED
bool AP_DDS_Client::write_tx_local_velocity_topic()
{
    WITH_SEMAPHORE(csem);
    if (!connected) {
        return false;
    }

    const auto& topic_info = topics[to_underlying(TopicIndex::LOCAL_VELOCITY_PUB)];
    ucdrBuffer ub {};
    const uint32_t topic_size = geometry_msgs_msg_TwistStamped_size_of_topic(&tx_local_velocity_topic, 0);
    if (!prepare_topic_stream(ub, topic_info.dw_id, topic_size, "vel", topic_info.qos)) {
        return false;
    }
    const bool success = geometry_msgs_msg_TwistStamped_serialize_topic(&ub, &tx_local_velocity_topic);
    if (!success) {
        return false;
    }
    finalize_topic_write(topic_info.qos, topic_size);
    return true;
}
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_IMU_PUB_ENABLED
bool AP_DDS_Client::write_imu_topic()
{
    WITH_SEMAPHORE(csem);
    if (!connected) {
        return false;
    }

    const auto& topic_info = topics[to_underlying(TopicIndex::IMU_PUB)];
    ucdrBuffer ub {};
    const uint32_t topic_size = sensor_msgs_msg_Imu_size_of_topic(&imu_topic, 0);
    if (!prepare_topic_stream(ub, topic_info.dw_id, topic_size, "imu", topic_info.qos)) {
        return false;
    }
    const bool success = sensor_msgs_msg_Imu_serialize_topic(&ub, &imu_topic);
    if (!success) {
        return false;
    }
    finalize_topic_write(topic_info.qos, topic_size);
    return true;
}
#endif // AP_DDS_IMU_PUB_ENABLED

#if AP_DDS_UWB_PUB_ENABLED
void AP_DDS_Client::write_uwb_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = sensor_msgs_msg_uwb_size_of_topic(&uwb_topic, 0);
        if (!prepare_topic_stream(ub, topics[to_underlying(TopicIndex::UWB_PUB)].dw_id, topic_size, "Uwb",
                                  topics[to_underlying(TopicIndex::UWB_PUB)].qos)) {
            return;
        }
        const bool success = sensor_msgs_msg_uwb_serialize_topic(&ub, &uwb_topic);
        if (!success) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Uwb : XRCE serialize failed", msg_prefix);
            return;
        }
        finalize_topic_write(topics[to_underlying(TopicIndex::UWB_PUB)].qos, topic_size);
    }
}
#endif // AP_DDS_UWB_PUB_ENABLED

#if AP_DDS_GEOPOSE_PUB_ENABLED
bool AP_DDS_Client::write_geo_pose_topic()
{
    WITH_SEMAPHORE(csem);
    if (!connected) {
        return false;
    }

    const auto& topic_info = topics[to_underlying(TopicIndex::GEOPOSE_PUB)];
    ucdrBuffer ub {};
    const uint32_t topic_size = geographic_msgs_msg_GeoPoseStamped_size_of_topic(&geo_pose_topic, 0);
    if (!prepare_topic_stream(ub, topic_info.dw_id, topic_size, "geo", topic_info.qos)) {
        return false;
    }
    const bool success = geographic_msgs_msg_GeoPoseStamped_serialize_topic(&ub, &geo_pose_topic);
    if (!success) {
        return false;
    }
    finalize_topic_write(topic_info.qos, topic_size);
    return true;
}
#endif // AP_DDS_GEOPOSE_PUB_ENABLED

#if AP_DDS_CLOCK_PUB_ENABLED
void AP_DDS_Client::write_clock_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        const auto& topic_info = topics[to_underlying(TopicIndex::CLOCK_PUB)];
        ucdrBuffer ub {};
        const uint32_t topic_size = rosgraph_msgs_msg_Clock_size_of_topic(&clock_topic, 0);
        if (!prepare_topic_stream(ub, topic_info.dw_id, topic_size, "clk", topic_info.qos)) {
            return;
        }
        const bool success = rosgraph_msgs_msg_Clock_serialize_topic(&ub, &clock_topic);
        if (!success) {
            return;
        }
        finalize_topic_write(topic_info.qos, topic_size);
    }
}
#endif // AP_DDS_CLOCK_PUB_ENABLED

#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
bool AP_DDS_Client::write_gps_global_origin_topic()
{
    WITH_SEMAPHORE(csem);
    if (!connected) {
        return false;
    }

    const auto& topic_info = topics[to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB)];
    ucdrBuffer ub {};
    const uint32_t topic_size = geographic_msgs_msg_GeoPointStamped_size_of_topic(&gps_global_origin_topic, 0);
    if (!prepare_topic_stream(ub, topic_info.dw_id, topic_size, "gori", topic_info.qos)) {
        return false;
    }
    const bool success = geographic_msgs_msg_GeoPointStamped_serialize_topic(&ub, &gps_global_origin_topic);
    if (!success) {
        return false;
    }
    finalize_topic_write(topic_info.qos, topic_size);
    return true;
}
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED

void AP_DDS_Client::update()
{
    set_runtime_phase(RuntimePhase::TOPIC_UPDATE);
    WITH_SEMAPHORE(csem);
    const auto cur_time_ms = AP_HAL::millis64();

#if AP_DDS_TIME_PUB_ENABLED
    if (cur_time_ms - last_time_time_ms > DELAY_TIME_TOPIC_MS) {
        update_topic(time_topic);
        last_time_time_ms = cur_time_ms;
        write_time_topic();
    }
#endif // AP_DDS_TIME_PUB_ENABLED
#if AP_DDS_NAVSATFIX_PUB_ENABLED
    constexpr uint8_t gps_instance = 0;
    if (cur_time_ms - last_nav_sat_fix_time_ms >= AP_DDS_DELAY_NAVSATFIX_TOPIC_MS) {
        const bool snapshot_ok = update_topic(nav_sat_fix_topic, gps_instance);
        set_runtime_phase(RuntimePhase::TOPIC_UPDATE);
        if (snapshot_ok &&
            write_nav_sat_fix_topic()) {
            last_nav_sat_fix_time_ms = cur_time_ms;
        }
    }
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED
#if AP_DDS_BATTERY_STATE_PUB_ENABLED
    if (cur_time_ms - last_battery_state_time_ms > DELAY_BATTERY_STATE_TOPIC_MS) {
        for (uint8_t battery_instance = 0; battery_instance < AP_BATT_MONITOR_MAX_INSTANCES; battery_instance++) {
            update_topic(battery_state_topic, battery_instance);
            if (battery_state_topic.present) {
                write_battery_state_topic();
            }
        }
        last_battery_state_time_ms = cur_time_ms;
    }
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED
#if AP_DDS_LOCAL_POSE_PUB_ENABLED
    if (cur_time_ms - last_local_pose_time_ms >= DELAY_LOCAL_POSE_TOPIC_MS) {
        const bool snapshot_ok = update_topic(local_pose_topic);
        set_runtime_phase(RuntimePhase::TOPIC_UPDATE);
        if (snapshot_ok && write_local_pose_topic()) {
            last_local_pose_time_ms = cur_time_ms;
        }
    }
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED
#if AP_DDS_LOCAL_VEL_PUB_ENABLED
    if (cur_time_ms - last_local_velocity_time_ms > DELAY_LOCAL_VELOCITY_TOPIC_MS) {
        const bool snapshot_ok = update_topic(tx_local_velocity_topic);
        set_runtime_phase(RuntimePhase::TOPIC_UPDATE);
        if (snapshot_ok) {
            const bool write_ok = write_tx_local_velocity_topic();
            if (write_ok || !is_using_serial) {
                last_local_velocity_time_ms = cur_time_ms;
            }
        }
    }
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_IMU_PUB_ENABLED
    if (cur_time_ms - last_imu_time_ms >= DELAY_IMU_TOPIC_MS) {
        const bool snapshot_ok = update_topic(imu_topic);
        set_runtime_phase(RuntimePhase::TOPIC_UPDATE);
        if (snapshot_ok && write_imu_topic()) {
            last_imu_time_ms = cur_time_ms;
        }
    }
#endif // AP_DDS_IMU_PUB_ENABLED
#if AP_DDS_UWB_PUB_ENABLED
    if (cur_time_ms - last_uwb_time_ms > DELAY_UWB_TOPIC_MS) {
        last_uwb_time_ms = cur_time_ms;
        if (update_topic(uwb_topic)) {
            write_uwb_topic();
        }
    }
#endif // AP_DDS_UWB_PUB_ENABLED
#if AP_DDS_GEOPOSE_PUB_ENABLED
    if (cur_time_ms - last_geo_pose_time_ms > DELAY_GEO_POSE_TOPIC_MS) {
        const bool snapshot_ok = update_topic(geo_pose_topic);
        set_runtime_phase(RuntimePhase::TOPIC_UPDATE);
        if (snapshot_ok) {
            const bool write_ok = write_geo_pose_topic();
            if (write_ok || !is_using_serial) {
                last_geo_pose_time_ms = cur_time_ms;
            }
        }
    }
#endif // AP_DDS_GEOPOSE_PUB_ENABLED
#if AP_DDS_CLOCK_PUB_ENABLED
    if (cur_time_ms - last_clock_time_ms > DELAY_CLOCK_TOPIC_MS) {
        update_topic(clock_topic);
        last_clock_time_ms = cur_time_ms;
        write_clock_topic();
    }
#endif // AP_DDS_CLOCK_PUB_ENABLED
#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
    if (cur_time_ms - last_gps_global_origin_time_ms > DELAY_GPS_GLOBAL_ORIGIN_TOPIC_MS) {
        const bool snapshot_ok = update_topic(gps_global_origin_topic);
        set_runtime_phase(RuntimePhase::TOPIC_UPDATE);
        if (snapshot_ok) {
            const bool write_ok = write_gps_global_origin_topic();
            if (write_ok || !is_using_serial) {
                last_gps_global_origin_time_ms = cur_time_ms;
            }
        }
    }
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED

    // On serial, bound the whole receive/dispatch pass. uxr_run_session_time()
    // waits for a continuous idle interval and can run forever under steady RX.
    // Retain the existing API for the non-serial transport.
    set_runtime_phase(RuntimePhase::XRCE_RUN);
    status_ok = is_using_serial ? uxr_run_session_timeout(&session, 20)
                                : uxr_run_session_time(&session, 20);
#if AP_DDS_TIME_PUB_ENABLED
    if (time_ack_pending && reliable_out.index < session.streams.output_reliable_size &&
        reliable_sequence_acked(session.streams.output_reliable[reliable_out.index].last_acknown,
                                time_ack_sequence)) {
        last_time_ack_ms = AP_HAL::millis();
        time_ack_pending = false;
    }
#endif
    set_runtime_phase(RuntimePhase::IDLE);
}

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
extern "C" {
    int clock_gettime(clockid_t clockid, struct timespec *ts);
}

int clock_gettime(clockid_t clockid, struct timespec *ts)
{
    (void)clockid;

    // Micro-XRCE uses this clock for elapsed-time deadlines.  Switching from
    // boot time to UTC when GPS establishes the RTC makes those deadlines jump
    // and can turn a millisecond receive timeout into a multi-day wait after
    // the library narrows the delta to int.  ROS time topics read the RTC
    // directly, so keeping this protocol clock monotonic does not alter their
    // timestamps.
    const uint64_t monotonic_usec = AP_HAL::micros64();
    ts->tv_sec = monotonic_usec / 1000000ULL;
    ts->tv_nsec = (monotonic_usec % 1000000ULL) * 1000UL;
    return 0;
}
#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#endif // AP_DDS_ENABLED
