#pragma once

#include "AP_DDS_config.h"

#if AP_DDS_ENABLED

#include "uxr/client/client.h"
#include "ucdr/microcdr.h"

#if AP_DDS_GLOBAL_POS_CTRL_ENABLED
#include "ardupilot_msgs/msg/GlobalPosition.h"
#endif // AP_DDS_GLOBAL_POS_CTRL_ENABLED
#if AP_DDS_TIME_PUB_ENABLED
#include "builtin_interfaces/msg/Time.h"
#endif // AP_DDS_TIME_PUB_ENABLED
#if AP_DDS_NAVSATFIX_PUB_ENABLED
#include "sensor_msgs/msg/NavSatFix.h"
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED
#if AP_DDS_NEEDS_TRANSFORMS
#include "tf2_msgs/msg/TFMessage.h"
#endif // AP_DDS_NEEDS_TRANSFORMS
#if AP_DDS_BATTERY_STATE_PUB_ENABLED
#include "sensor_msgs/msg/BatteryState.h"
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED
#if AP_DDS_IMU_PUB_ENABLED
#include "sensor_msgs/msg/Imu.h"
#endif // AP_DDS_IMU_PUB_ENABLED
#if AP_DDS_UWB_PUB_ENABLED
#include "sensor_msgs/msg/uwb.h"
#endif // AP_DDS_UWB_PUB_ENABLED
#if AP_DDS_JOY_SUB_ENABLED
#include "sensor_msgs/msg/Joy.h"
#endif // AP_DDS_JOY_SUB_ENABLED
#if AP_DDS_LOCAL_POSE_PUB_ENABLED
#include "geometry_msgs/msg/PoseStamped.h"
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED
#if AP_DDS_NEEDS_TWIST
#include "geometry_msgs/msg/TwistStamped.h"
#endif // AP_DDS_NEEDS_TWIST
#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
#include "geographic_msgs/msg/GeoPointStamped.h"
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
#if AP_DDS_GEOPOSE_PUB_ENABLED
#include "geographic_msgs/msg/GeoPoseStamped.h"
#endif // AP_DDS_GEOPOSE_PUB_ENABLED
#if AP_DDS_CLOCK_PUB_ENABLED
#include "rosgraph_msgs/msg/Clock.h"
#endif // AP_DDS_CLOCK_PUB_ENABLED
#if AP_DDS_PARAMETER_SERVER_ENABLED
#include "rcl_interfaces/srv/SetParameters.h"
#include "rcl_interfaces/msg/Parameter.h"
#include "rcl_interfaces/msg/SetParametersResult.h"
#include "rcl_interfaces/msg/ParameterValue.h"
#include "rcl_interfaces/msg/ParameterType.h"
#include "rcl_interfaces/srv/GetParameters.h"
#endif //AP_DDS_PARAMETER_SERVER_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>

#include <atomic>

#include "fcntl.h"

#include <AP_Param/AP_Param.h>

#define DDS_MTU             512
#define DDS_STREAM_HISTORY  16
#define DDS_BUFFER_SIZE     (DDS_MTU * DDS_STREAM_HISTORY)
#define DDS_BEST_EFFORT_BUFFER_SIZE  (DDS_MTU * 4)

#if AP_DDS_UDP_ENABLED
#include <AP_HAL/utility/Socket.h>
#include <AP_Networking/AP_Networking_address.h>
#endif

extern const AP_HAL::HAL& hal;

class AP_DDS_Client
{

private:

    AP_Int8 enabled;

    // Serial Allocation
    uxrSession session{}; // Session
    bool is_using_serial{false}; // true when using serial transport

    // input and output stream
    uint8_t *input_reliable_stream{nullptr};
    uint8_t *output_reliable_stream{nullptr};
    uint8_t *output_best_effort_stream{nullptr};
    uxrStreamId reliable_in;
    uxrStreamId reliable_out;
    uxrStreamId best_effort_in;
    uxrStreamId best_effort_out;

    // Outgoing Sensor and AHRS data

#if AP_DDS_TIME_PUB_ENABLED
    builtin_interfaces_msg_Time time_topic;
    // The last ms timestamp AP_DDS wrote a Time message
    uint64_t last_time_time_ms{};
    // Sequence and reception time of the last confirmed Time sample.  This is
    // the application-data heartbeat and cannot be refreshed by raw UART RX.
    bool time_ack_pending{};
    uxrSeqNum time_ack_sequence{};
    uint32_t last_time_ack_ms{};
    //! @brief Serialize the current time state and publish to the IO stream(s)
    void write_time_topic();
    static void update_topic(builtin_interfaces_msg_Time& msg);
#endif // AP_DDS_TIME_PUB_ENABLED

#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
    geographic_msgs_msg_GeoPointStamped gps_global_origin_topic;
    // The last ms timestamp AP_DDS wrote a gps global origin message
    uint64_t last_gps_global_origin_time_ms;
    //! @brief Serialize the current gps global origin and publish to the IO stream(s)
    bool write_gps_global_origin_topic() WARN_IF_UNUSED;
    bool update_topic(geographic_msgs_msg_GeoPointStamped& msg) WARN_IF_UNUSED;
# endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED

#if AP_DDS_GEOPOSE_PUB_ENABLED
    geographic_msgs_msg_GeoPoseStamped geo_pose_topic;
    // The last ms timestamp AP_DDS wrote a GeoPose message
    uint64_t last_geo_pose_time_ms;
    //! @brief Serialize the current geo_pose and publish to the IO stream(s)
    bool write_geo_pose_topic() WARN_IF_UNUSED;
    bool update_topic(geographic_msgs_msg_GeoPoseStamped& msg) WARN_IF_UNUSED;
#endif // AP_DDS_GEOPOSE_PUB_ENABLED

#if AP_DDS_LOCAL_POSE_PUB_ENABLED
    geometry_msgs_msg_PoseStamped local_pose_topic;
    // The last ms timestamp AP_DDS wrote a Local Pose message
    uint64_t last_local_pose_time_ms;
    //! @brief Serialize the current local_pose and publish to the IO stream(s)
    bool write_local_pose_topic() WARN_IF_UNUSED;
    bool update_topic(geometry_msgs_msg_PoseStamped& msg) WARN_IF_UNUSED;
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED

#if AP_DDS_LOCAL_VEL_PUB_ENABLED
    geometry_msgs_msg_TwistStamped tx_local_velocity_topic;
    // The last ms timestamp AP_DDS wrote a Local Velocity message
    uint64_t last_local_velocity_time_ms;
    //! @brief Serialize the current local velocity and publish to the IO stream(s)
    bool write_tx_local_velocity_topic() WARN_IF_UNUSED;
    bool update_topic(geometry_msgs_msg_TwistStamped& msg) WARN_IF_UNUSED;
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED

#if AP_DDS_BATTERY_STATE_PUB_ENABLED
    sensor_msgs_msg_BatteryState battery_state_topic;
    // The last ms timestamp AP_DDS wrote a BatteryState message
    uint64_t last_battery_state_time_ms;
    //! @brief Serialize the current nav_sat_fix state and publish it to the IO stream(s)
    void write_battery_state_topic();
    static void update_topic(sensor_msgs_msg_BatteryState& msg, const uint8_t instance);
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED

#if AP_DDS_NAVSATFIX_PUB_ENABLED
    sensor_msgs_msg_NavSatFix nav_sat_fix_topic {};
    // The last ms timestamp AP_DDS wrote a NavSatFix message
    uint64_t last_nav_sat_fix_time_ms;
    //! @brief Serialize the current nav_sat_fix state and publish to the IO stream(s)
    bool write_nav_sat_fix_topic() WARN_IF_UNUSED;
    bool update_topic(sensor_msgs_msg_NavSatFix& msg, const uint8_t instance) WARN_IF_UNUSED;
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED

#if AP_DDS_IMU_PUB_ENABLED
    sensor_msgs_msg_Imu imu_topic;
    // The last ms timestamp AP_DDS wrote an IMU message
    uint64_t last_imu_time_ms;
    bool update_topic(sensor_msgs_msg_Imu& msg) WARN_IF_UNUSED;
    //! @brief Serialize the current IMU data and publish to the IO stream(s)
    bool write_imu_topic() WARN_IF_UNUSED;
#endif // AP_DDS_IMU_PUB_ENABLED

#if AP_DDS_UWB_PUB_ENABLED
    sensor_msgs_msg_uwb uwb_topic {};
    // The last ms timestamp AP_DDS wrote a UWB message
    uint64_t last_uwb_time_ms{};
    static bool update_topic(sensor_msgs_msg_uwb& msg);
    //! @brief Serialize the current UWB target position and publish to the IO stream(s)
    void write_uwb_topic();
#endif // AP_DDS_UWB_PUB_ENABLED

#if AP_DDS_CLOCK_PUB_ENABLED
    rosgraph_msgs_msg_Clock clock_topic;
    // The last ms timestamp AP_DDS wrote a Clock message
    uint64_t last_clock_time_ms;
    //! @brief Serialize the current clock and publish to the IO stream(s)
    void write_clock_topic();
    static void update_topic(rosgraph_msgs_msg_Clock& msg);
#endif // AP_DDS_CLOCK_PUB_ENABLED

#if AP_DDS_STATIC_TF_PUB_ENABLED
    // outgoing transforms
    tf2_msgs_msg_TFMessage tx_static_transforms_topic;
    //! @brief Serialize the static transforms and publish to the IO stream(s)
    void write_static_transforms();
    static void populate_static_transforms(tf2_msgs_msg_TFMessage& msg);
#endif // AP_DDS_STATIC_TF_PUB_ENABLED

#if AP_DDS_JOY_SUB_ENABLED
    // incoming joystick data
    static sensor_msgs_msg_Joy rx_joy_topic;
    bool joy_pending{};
    uint32_t joy_epoch{};
    uint32_t joy_receive_time_ms{};
#endif // AP_DDS_JOY_SUB_ENABLED
#if AP_DDS_VEL_CTRL_ENABLED
    // incoming REP147 velocity control
    static geometry_msgs_msg_TwistStamped rx_velocity_control_topic;
    bool velocity_control_pending{};
    uint32_t velocity_control_epoch{};
#endif // AP_DDS_VEL_CTRL_ENABLED
#if AP_DDS_EXTNAV_VEL_SUB_ENABLED
    // incoming external navigation velocity observation
    static geometry_msgs_msg_TwistStamped rx_extnav_velocity_topic;
    bool extnav_velocity_pending{};
    uint32_t extnav_velocity_epoch{};
    uint32_t extnav_velocity_receive_time_ms{};
#endif // AP_DDS_EXTNAV_VEL_SUB_ENABLED
#if AP_DDS_GLOBAL_POS_CTRL_ENABLED
    // incoming REP147 goal interface global position
    static ardupilot_msgs_msg_GlobalPosition rx_global_position_control_topic;
    bool global_position_control_pending{};
    uint32_t global_position_control_epoch{};
#endif // AP_DDS_GLOBAL_POS_CTRL_ENABLED
#if AP_DDS_DYNAMIC_TF_SUB_ENABLED
    // incoming transforms
    static tf2_msgs_msg_TFMessage rx_dynamic_transforms_topic;
    bool dynamic_transforms_pending{};
    uint32_t dynamic_transforms_epoch{};
    uint32_t dynamic_transforms_receive_time_ms{};
#endif // AP_DDS_DYNAMIC_TF_SUB_ENABLED

    // DDS callbacks are run by the DDS owner thread.  They only update these
    // fixed-size mailboxes while the vehicle thread consumes them without
    // ever making the DDS thread wait for vehicle/EKF locks.
    HAL_Semaphore input_sem;
    std::atomic<bool> topic_input_pending{};
    HAL_Semaphore csem;

#if AP_DDS_PARAMETER_SERVER_ENABLED
    static rcl_interfaces_srv_SetParameters_Request set_parameter_request;
    static rcl_interfaces_srv_SetParameters_Response set_parameter_response;
    static rcl_interfaces_srv_GetParameters_Request get_parameters_request;
    static rcl_interfaces_srv_GetParameters_Response get_parameters_response;
    static rcl_interfaces_msg_Parameter param;
#endif

    // connection parametrics
    bool status_ok{false};
    bool connected{false};
    bool session_created{false};
    std::atomic<uint32_t> session_epoch{};

    enum class ServiceCommand : uint8_t {
        NONE,
        ARM,
        MODE_SWITCH,
        TAKEOFF,
        PREARM_CHECK,
        SET_PARAMETERS,
        GET_PARAMETERS,
    };

    enum class ServiceState : uint8_t {
        IDLE,
        FILLING,
        QUEUED,
        PROCESSING,
        COMPLETED,
    };

    struct PendingServiceRequest {
        ServiceCommand command{ServiceCommand::NONE};
        SampleIdentity sample_id{};
        uint8_t replier_id{};
        uint32_t epoch{};
        bool arm{};
        uint8_t mode{};
        float takeoff_alt{};
        bool result{};
        uint8_t current_mode{};
    } pending_service;
    std::atomic<uint8_t> pending_service_state{static_cast<uint8_t>(ServiceState::IDLE)};

    enum class RuntimePhase : uint8_t {
        IDLE,
        TRANSPORT_OPEN,
        STARTUP_PING,
        SESSION_CREATE,
        ENTITY_CREATE,
        TOPIC_UPDATE,
        GPS_SNAPSHOT,
        AHRS_SNAPSHOT,
        XRCE_RUN,
        TOPIC_CALLBACK,
        SERVICE_CALLBACK,
        HEALTH_PING,
        SESSION_CLEANUP,
        TRANSPORT_CLOSE,
        BACKOFF,
    };

    enum class ReconnectReason : uint8_t {
        NONE,
        WATCHDOG_STALL,
    };

    void cleanup_session(bool notify_agent);
    void note_transport_rx(size_t wire_bytes);
    bool take_topic_semaphore(AP_HAL::Semaphore &semaphore) WARN_IF_UNUSED;
    void set_runtime_phase(RuntimePhase phase, uint32_t allowed_ms = 3000U);
    void runtime_supervisor();
    void request_reconnect(ReconnectReason reason);
    ReconnectReason consume_reconnect_request();
    static const char *runtime_phase_name(RuntimePhase phase);
    bool begin_service_request(ServiceCommand command, uint8_t replier_id, const SampleIdentity &sample_id) WARN_IF_UNUSED;
    void commit_service_request();
    void cancel_service_request();
    void send_service_failure(uxrSession *uxr_session, ServiceCommand command, uint8_t replier_id, const SampleIdentity &sample_id);
    void process_service_request_main_thread();
    void drain_service_reply();
    void discard_stale_service_request();
    uxrStreamId output_stream_for_qos(const uxrQoS_t& qos) const;
    uxrStreamId input_stream_for_qos(const uxrQoS_t& qos) const;
    void finalize_topic_write(const uxrQoS_t& qos, uint32_t payload_bytes);
    bool prepare_topic_stream(ucdrBuffer& ub, uxrObjectId datawriter_id, uint32_t topic_size, const char* topic_name, const uxrQoS_t& qos, uint16_t* request_id = nullptr);
    uint64_t last_rx_activity_ms{};
    std::atomic<uint8_t> runtime_phase{static_cast<uint8_t>(RuntimePhase::IDLE)};
    std::atomic<uint32_t> runtime_progress_ms{};
    std::atomic<uint32_t> runtime_allowed_ms{3000U};
    std::atomic<bool> runtime_owner_active{};
    std::atomic<bool> runtime_stall_reported{};
    std::atomic<uint8_t> reconnect_reason{static_cast<uint8_t>(ReconnectReason::NONE)};

    // subscription callback function
    static void on_topic_trampoline(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length, void* args);
    void on_topic(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length);

    // service replier callback function
    static void on_request_trampoline(uxrSession* session, uxrObjectId object_id, uint16_t request_id, SampleIdentity* sample_id, ucdrBuffer* ub, uint16_t length, void* args);
    void on_request(uxrSession* session, uxrObjectId object_id, uint16_t request_id, SampleIdentity* sample_id, ucdrBuffer* ub, uint16_t length);

    // delivery control parameters
    uxrDeliveryControl delivery_control {
        .max_samples = UXR_MAX_SAMPLES_UNLIMITED,
        .max_elapsed_time = 0,
        .max_bytes_per_second = 0,
        .min_pace_period = 0
    };

    // functions for serial transport
    bool ddsSerialInit();
    static bool serial_transport_open(uxrCustomTransport* args);
    static bool serial_transport_close(uxrCustomTransport* transport);
    static size_t serial_transport_write(uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* error);
    static size_t serial_transport_read(uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* error);
    struct {
        AP_HAL::UARTDriver *port{nullptr};
        uxrCustomTransport transport;
        int8_t port_num{-1};
        uint32_t baud{};
        uint32_t last_write_error_report_ms{};
        bool read_cancelled{};
    } serial;

#if AP_DDS_UDP_ENABLED
    // functions for udp transport
    bool ddsUdpInit();
    static bool udp_transport_open(uxrCustomTransport* args);
    static bool udp_transport_close(uxrCustomTransport* transport);
    static size_t udp_transport_write(uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* error);
    static size_t udp_transport_read(uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* error);

    struct {
        AP_Int32 port;
        // UDP endpoint
        AP_Networking_IPV4 ip{AP_DDS_DEFAULT_UDP_IP_ADDR};
        // UDP Allocation
        uxrCustomTransport transport;
        SocketAPM *socket{nullptr};
    } udp;
#endif
    // pointer to transport's communication structure
    uxrCommunication *comm{nullptr};

    // client key we present
    static constexpr uint32_t key = 0xAAAABBBB;

public:
    ~AP_DDS_Client();

    bool start(void);
    void main_loop(void);

    //! @brief Initialize the client's transport
    //! @return True on successful initialization, false on failure
    bool init_transport() WARN_IF_UNUSED;

    //! @brief Initialize the client's uxr session and IO stream(s)
    //! @return True on successful initialization, false on failure
    bool init_session() WARN_IF_UNUSED;

    //! @brief Set up the client's participants, data read/writes,
    //         publishers, subscribers
    //! @return True on successful creation, false on failure
    bool create() WARN_IF_UNUSED;

    //! @brief Update the internally stored DDS messages with latest data
    void update();

    //! @brief Apply queued DDS input from the vehicle main thread
    void update_main_thread();

    //! @brief GCS message prefix
    static constexpr const char* msg_prefix = "DDS:";

    //! @brief Parameter storage
    static const struct AP_Param::GroupInfo var_info[];

    //! @brief ROS_DOMAIN_ID
    AP_Int32 domain_id;

    //! @brief Timeout in milliseconds when pinging the XRCE agent
    AP_Int32 ping_timeout_ms;

    //! @brief Maximum number of attempts to ping the XRCE agent before exiting
    AP_Int8 ping_max_retry;

    //! @brief 1-sigma uncertainty for DDS external navigation velocity observations
    AP_Float extnav_velocity_error;

    //! @brief Average delay of DDS external navigation velocity observations
    AP_Int16 extnav_velocity_delay_ms;

    //! @brief Enum used to mark a topic as a data reader or writer
    enum class Topic_rw : uint8_t {
        DataReader = 0,
        DataWriter = 1,
    };

    //! @brief Convenience grouping for a single "channel" of data
    struct Topic_table {
        const uint8_t topic_id;
        const uint8_t pub_id;
        const uint8_t sub_id;    // added sub_id fields to avoid confusion
        const uxrObjectId dw_id;
        const uxrObjectId dr_id; // added dr_id fields to avoid confusion
        const Topic_rw topic_rw;
        const char* topic_name;
        const char* type_name;
        const uxrQoS_t qos;
    };
    static const struct Topic_table topics[];

    //! @brief Enum used to mark a service as a requester or replier
    enum class Service_rr : uint8_t {
        Requester = 0,
        Replier = 1,
    };

    //! @brief Convenience grouping for a single "channel" of services
    struct Service_table {
        //! @brief Request ID for the service
        const uint8_t req_id;

        //! @brief Reply ID for the service
        const uint8_t rep_id;

        //! @brief Service is requester or replier
        const Service_rr service_rr;

        //! @brief Service name as it appears in ROS
        const char* service_name;

        //! @brief Service requester message type
        const char* request_type;

        //! @brief Service replier message type
        const char* reply_type;

        //! @brief Service requester topic name
        const char* request_topic_name;

        //! @brief Service replier topic name
        const char* reply_topic_name;

        //! @brief QoS for the service
        const uxrQoS_t qos;
    };
    static const struct Service_table services[];
};

#endif // AP_DDS_ENABLED
