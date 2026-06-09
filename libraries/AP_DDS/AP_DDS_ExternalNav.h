#pragma once

#include "AP_DDS_config.h"

#if AP_DDS_ENABLED && AP_DDS_EXTNAV_VEL_SUB_ENABLED

#include "geometry_msgs/msg/TwistStamped.h"

class AP_DDS_ExternalNav
{
public:
    // External navigation current velocity observation in ROS FLU base_link frame
    static bool handle_velocity(const geometry_msgs_msg_TwistStamped& velocity, float velocity_error, uint16_t delay_ms);
};

#endif // AP_DDS_ENABLED && AP_DDS_EXTNAV_VEL_SUB_ENABLED
