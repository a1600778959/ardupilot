#include "AP_DDS_config.h"

#if AP_DDS_ENABLED && (AP_DDS_EXTNAV_VEL_SUB_ENABLED || AP_DDS_DYNAMIC_TF_SUB_ENABLED)

#include "AP_DDS_ExternalNav.h"
#include "AP_DDS_Frames.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <string.h>

#if AP_DDS_DYNAMIC_TF_SUB_ENABLED
static constexpr float EXTNAV_TF_POS_ERROR_M = 0.2f;
static constexpr float EXTNAV_TF_ANG_ERROR_RAD = 0.2f;
static constexpr uint16_t EXTNAV_TF_DELAY_MS = 10;

static bool is_valid_quaternion(const Quaternion& quat)
{
    return !quat.is_nan() &&
           isfinite(quat[0]) &&
           isfinite(quat[1]) &&
           isfinite(quat[2]) &&
           isfinite(quat[3]) &&
           !quat.is_zero();
}

bool AP_DDS_ExternalNav::handle_tf(const tf2_msgs_msg_TFMessage& tf, uint32_t receive_time_ms)
{
    for (uint32_t i = 0; i < tf.transforms_size; i++) {
        const geometry_msgs_msg_TransformStamped& transform = tf.transforms[i];
        const bool supported_parent_frame = (strcmp(transform.header.frame_id, ODOM_FRAME) == 0) ||
                                            (strcmp(transform.header.frame_id, MAP_FRAME) == 0);
        if (!supported_parent_frame || strcmp(transform.child_frame_id, BASE_LINK_FRAME_ID) != 0) {
            continue;
        }

        const Vector3f position_enu {
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            float(transform.transform.translation.z)
        };
        if (!isfinite(position_enu.x) ||
            !isfinite(position_enu.y) ||
            !isfinite(position_enu.z)) {
            return false;
        }

        const Quaternion orientation_ros {
            float(transform.transform.rotation.w),
            float(transform.transform.rotation.x),
            float(transform.transform.rotation.y),
            float(transform.transform.rotation.z)
        };
        if (!is_valid_quaternion(orientation_ros)) {
            return false;
        }

        const Vector3f position_ned {
            position_enu.y,
            position_enu.x,
            -position_enu.z
        };

        const Quaternion transformation(sqrtF(2) * 0.5f, 0.0f, 0.0f, sqrtF(2) * 0.5f);
        const Quaternion aux = orientation_ros * transformation.inverse();
        Quaternion orientation_ap(aux[0], aux[2], aux[1], -aux[3]);
        if (!is_valid_quaternion(orientation_ap)) {
            return false;
        }
        orientation_ap.normalize();

        AP::ahrs().writeExtNavData(position_ned, orientation_ap, EXTNAV_TF_POS_ERROR_M, EXTNAV_TF_ANG_ERROR_RAD, receive_time_ms, EXTNAV_TF_DELAY_MS, 0);
        return true;
    }

    return false;
}
#endif // AP_DDS_DYNAMIC_TF_SUB_ENABLED

#if AP_DDS_EXTNAV_VEL_SUB_ENABLED
bool AP_DDS_ExternalNav::handle_velocity(const geometry_msgs_msg_TwistStamped& velocity,
                                         float velocity_error,
                                         uint16_t delay_ms,
                                         uint32_t receive_time_ms)
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
    ahrs.writeExtNavVelData(velocity_ned, velocity_error, receive_time_ms, delay_ms);
    ahrs.writeExtNavForwardSpeedData(velocity_frd.x);
    return true;
}
#endif // AP_DDS_EXTNAV_VEL_SUB_ENABLED

#endif // AP_DDS_ENABLED && (AP_DDS_EXTNAV_VEL_SUB_ENABLED || AP_DDS_DYNAMIC_TF_SUB_ENABLED)
