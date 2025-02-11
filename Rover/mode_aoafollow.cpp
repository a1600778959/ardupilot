// mode_aoafollow.cpp
#include "Rover.h"
const AP_Param::GroupInfo ModeAoafllow::var_info[] = {
    // PID参数
    AP_GROUPINFO("DIST_KP", 1, ModeAoafllow, _dist_kp, 0.8f),
    AP_GROUPINFO("DIST_KI", 2, ModeAoafllow, _dist_ki, 0.05f),
    AP_GROUPINFO("DIST_KD", 3, ModeAoafllow, _dist_kd, 0.2f),
    AP_GROUPINFO("ANGLE_KP", 4, ModeAoafllow, _angle_kp, 0.6f),
    AP_GROUPINFO("ANGLE_KD", 5, ModeAoafllow, _angle_kd, 0.15f),
    // 运行参数
    AP_GROUPINFO("TARGET_DIST", 6, ModeAoafllow, _target_dist, 1.0f),
    AP_GROUPINFO("MAX_SPEED", 7, ModeAoafllow, _max_speed, 1.0f),
    AP_GROUPINFO("STEER_LIM", 8, ModeAoafllow, _steer_limit, 30.0f),
    AP_GROUPEND};

ModeAoafllow::ModeAoafllow() : Mode(), // 必须首先初始化基类
                               _last_update_ms(0),
                               _data_timeout_ms(0),
                               _throttle_out(0.0f),
                               _steering_out(0.0f),
                                _emergency_stop(false)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeAoafllow::_enter()
{
    // 初始化传感器
    aoa_sensor.init();

    // 重置控制器状态
    reset_controllers();

    // 显示模式信息
    gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow ENGAGED");
    return true;
}

void ModeAoafllow::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const float dt = (now_ms - _last_update_ms) * 0.001f;
    _last_update_ms = now_ms;
    // gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow update start work");
    // 1. 获取原始传感器数据
    float raw_dist, raw_angle;
    aoa_sensor.update();
    if (!aoa_sensor.get_raw_data(raw_dist, raw_angle))
    {
        _handle_data_loss(dt);
        return;
    }
    // gcs().send_named_float("dist",raw_dist);
    // gcs().send_named_float("raw_angle", raw_angle);
    // gcs().send_text(MAV_SEVERITY_INFO, "传感器测量值:%f , %f", raw_dist, raw_angle);
    // 2. 卡尔曼滤波更新
    _kalman_filter.predict(dt);
    _kalman_filter.update(raw_dist, raw_angle);
    _data_timeout_ms = now_ms;

    // 3. 获取滤波状态
    const float filtered_dist = _kalman_filter.get_distance();
    const float filtered_angle = _kalman_filter.get_angle();

    gcs().send_named_float("filtered_dist", filtered_dist);
    gcs().send_named_float("filtered_angle", filtered_angle);

    // 4. 安全监测
    if (!_safety_check(filtered_dist))
    {
        return;
    }

    // 5. PID控制计算
    Vector2f control_out = _calculate_control(filtered_dist, filtered_angle, dt);

    // 6. 执行器输出
    _set_actuators(control_out);

    // 7. 调试输出
    _send_debug_info(now_ms, filtered_dist, filtered_angle, control_out);
}

void ModeAoafllow::_handle_data_loss(float dt)
{
    // 数据超时处理（超过1秒无数据）
    if (AP_HAL::millis() - _data_timeout_ms > 1000)
    {
        gcs().send_text(MAV_SEVERITY_WARNING, "AOA Data Timeout!");
        // 缓降速处理
        _throttle_out *= 0.8f;
        _steering_out *= 0.8f;
        _set_actuators(Vector2f(_throttle_out, _steering_out));
        // rover.set_mode(rover.mode_hold, ModeReason::FAILSAFE);
        // gcs().send_text(MAV_SEVERITY_WARNING, "AOA Data Timeout!");
        return;
    }

}

bool ModeAoafllow::_safety_check(float current_dist)
{
    // 紧急制动检查
    if (current_dist < 0.5f)
    {
        _emergency_stop = true;
        rover.g2.motors.set_throttle(0);
        rover.g2.motors.set_steering(0);

        gcs().send_text(MAV_SEVERITY_EMERGENCY, "EMERGENCY STOP!");
        return false;
    }

    // 重置急停状态
    if (_emergency_stop && current_dist > 1.0f)
    {
        _emergency_stop = false;
        reset_controllers();
    }
    return true;
}

Vector2f ModeAoafllow::_calculate_control(float dist, float angle, float dt)
{
    // 距离控制
    float dist_error = _target_dist - dist;
    _throttle_out = _dist_pid.get_pid(dist_error, dt, 1.0f / _max_speed);

    // 角度控制
    _steering_out = _angle_pid.get_pid(angle, dt, 1.0f / _steer_limit);

    // 输出限幅
    _throttle_out = constrain_float(_throttle_out, -1.0f, 1.0f);
    _steering_out = constrain_float(_steering_out, -1.0f, 1.0f);

    return Vector2f(_throttle_out, _steering_out);
}

void ModeAoafllow::_set_actuators(const Vector2f &control)
{
    if (_emergency_stop)
    {
        rover.g2.motors.set_throttle(0);
        rover.g2.motors.set_steering(0);
        return;
    }

    // 设置转向和油门
    rover.g2.motors.set_steering(control.y * _steer_limit);
    rover.g2.motors.set_throttle(control.x * _max_speed);
}

void ModeAoafllow::_send_debug_info(uint32_t timestamp, float dist, float angle, const Vector2f &control)
{
    // static uint32_t _last_debug_ms;
#define AOA_DEBUG 0
#if AOA_DEBUG
        // 发送MAVLink调试信息（每200ms）
    if (timestamp - _last_debug_ms > 200)
    {
        _last_debug_ms = timestamp;
        mavlink_msg_aoa_debug_send(
            MAVLINK_COMM_0,
            timestamp,
            dist,
            angle,
            control.x,
            control.y,
            _kalman_filter.get_variance(0),
            _kalman_filter.get_variance(1));
    }
#endif
}

void ModeAoafllow::reset_controllers()
{
    _dist_pid.reset();
    _angle_pid.reset();
    // 重置积分项和微分项
    _throttle_out = 0.0f;
    _steering_out = 0.0f;
    // _last_debug_ms = 0;
    _kalman_filter.reset();
}

// 模式退出处理
void ModeAoafllow::_exit()
{
    rover.g2.motors.set_throttle(0);
    rover.g2.motors.set_steering(0);
    gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow DISENGAGED");
}

// // 在AP_Mission中注册模式
// const struct AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
//     // ...
//     AP_GROUPINFO("MODE_AOA_FOLLOW", 23, GCS_MAVLINK_Parameters, mode_aoafollow, 0),
//     // ...
// };
