#include "AP_DDS_config.h"

#if AP_DDS_ENABLED && AP_DDS_EXTNAV_VEL_SUB_ENABLED

#include "AP_DDS_ExternalNav.h"
#include "AP_DDS_Frames.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

bool AP_DDS_ExternalNav::handle_velocity(const geometry_msgs_msg_TwistStamped& velocity, float velocity_error, uint16_t delay_ms)
{
    if (strcmp(velocity.header.frame_id, BASE_LINK_FRAME_ID) != 0) {
        return false;
    }

    const Vector3f velocity_flu {
        float(velocity.twist.linear.x),
        float(velocity.twist.linear.y),
        float(velocity.twist.linear.z)
    };

    if (!isfinite(velocity_flu.x) ||
        !isfinite(velocity_flu.y) ||
        !isfinite(velocity_flu.z) ||
        !isfinite(velocity_error) ||
        velocity_error <= 0.0f) {
        return false;
    }

    // ROS base_link uses FLU. ArduPilot body frame uses FRD.
    const Vector3f velocity_frd {
        velocity_flu.x,
        -velocity_flu.y,
        -velocity_flu.z
    };

    auto &ahrs = AP::ahrs();
    const Vector3f velocity_ned = ahrs.body_to_earth(velocity_frd);
    const uint32_t now_ms = AP_HAL::millis();
    ahrs.writeExtNavVelData(velocity_ned, velocity_error, now_ms, delay_ms);
    return true;
}

#endif // AP_DDS_ENABLED && AP_DDS_EXTNAV_VEL_SUB_ENABLED
