#pragma once

#include "AP_DDS_config.h"

#if AP_DDS_ENABLED && (AP_DDS_EXTNAV_VEL_SUB_ENABLED || AP_DDS_DYNAMIC_TF_SUB_ENABLED)

#if AP_DDS_EXTNAV_VEL_SUB_ENABLED
#include "geometry_msgs/msg/TwistStamped.h"
#endif
#if AP_DDS_DYNAMIC_TF_SUB_ENABLED
#include "tf2_msgs/msg/TFMessage.h"
#endif

class AP_DDS_ExternalNav
{
public:
#if AP_DDS_DYNAMIC_TF_SUB_ENABLED
    // External navigation pose observation from ROS odom/map -> base_link TF
    static bool handle_tf(const tf2_msgs_msg_TFMessage& tf, uint32_t receive_time_ms);
#endif
#if AP_DDS_EXTNAV_VEL_SUB_ENABLED
    // External navigation current velocity observation in ROS FLU base_link frame
    static bool handle_velocity(const geometry_msgs_msg_TwistStamped& velocity,
                                float velocity_error,
                                uint16_t delay_ms,
                                uint32_t receive_time_ms);
#endif
};

#endif // AP_DDS_ENABLED && (AP_DDS_EXTNAV_VEL_SUB_ENABLED || AP_DDS_DYNAMIC_TF_SUB_ENABLED)
