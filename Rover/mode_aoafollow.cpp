// mode_aoafollow.cpp
#include "Rover.h"

static constexpr float AOA_DT_MIN_S = 0.001f;
static constexpr float AOA_DT_MAX_S = 0.2f;
static constexpr float AOA_MIN_LIMIT = 0.01f;
static constexpr float AOA_REVERSE_SWITCH_ANGLE_DEG = 90.0f;
static constexpr uint32_t AOA_DEBUG_MIN_PERIOD_MS = 50U;

static float shortest_drive_heading_error(float target_heading_error_deg, bool &backing_to_target)
{
    target_heading_error_deg = wrap_180(target_heading_error_deg);
    backing_to_target = fabsf(target_heading_error_deg) > AOA_REVERSE_SWITCH_ANGLE_DEG;
    if (backing_to_target) {
        return wrap_180(target_heading_error_deg + 180.0f);
    }
    return target_heading_error_deg;
}

const AP_Param::GroupInfo ModeAoafllow::var_info[] = {
    // PID参数
    AP_GROUPINFO("DIST_KP", 1, ModeAoafllow, _dist_kp, 0.8f),
    AP_GROUPINFO("DIST_KI", 2, ModeAoafllow, _dist_ki, 0.05f),
    AP_GROUPINFO("DIST_KD", 3, ModeAoafllow, _dist_kd, 0.2f),
    AP_GROUPINFO("ANGLE_KP", 4, ModeAoafllow, _angle_kp, 0.6f),
    AP_GROUPINFO("ANGLE_KI", 5, ModeAoafllow, _angle_ki, 0.1f),
    AP_GROUPINFO("ANGLE_KD", 6, ModeAoafllow, _angle_kd, 0.15f),
    // 运行参数
    AP_GROUPINFO("TARGET_DIST", 7, ModeAoafllow, _target_dist, 1.0f),
    AP_GROUPINFO("MAX_SPEED", 8, ModeAoafllow, _max_speed, 1.0f),
    AP_GROUPINFO("STEER_LIM", 9, ModeAoafllow, _steer_limit, 1.0f),
    AP_GROUPINFO("SERIAL", 10, ModeAoafllow, _serial_port, 6),
    AP_GROUPINFO("LOST_TOUT", 11, ModeAoafllow, _lost_timeout_s, 0.25f),
    AP_GROUPINFO("HOLD_TOUT", 12, ModeAoafllow, _hold_timeout_s, 1.0f),
    AP_GROUPINFO("ESTOP_BUF", 13, ModeAoafllow, _estop_buffer_m, 0.0f),
    AP_GROUPINFO("ESTOP_HYST", 14, ModeAoafllow, _estop_release_hyst_m, 0.5f),
    AP_GROUPINFO("MAX_DERR", 15, ModeAoafllow, _max_dist_err_m, 20.0f),
    AP_GROUPINFO("MAX_AERR", 16, ModeAoafllow, _max_angle_err_deg, 60.0f),
    AP_GROUPINFO("THR_DZ", 17, ModeAoafllow, _thr_deadband, 0.02f),
    AP_GROUPINFO("STR_DZ", 18, ModeAoafllow, _steer_deadband, 0.06f),
    AP_GROUPINFO("THR_KICK", 19, ModeAoafllow, _thr_friction_offset, 10.0f),
    AP_GROUPINFO("STR_KICK", 20, ModeAoafllow, _steer_friction_offset, 450.0f),
    AP_GROUPINFO("LOSS_DECAY", 21, ModeAoafllow, _loss_decay, 0.8f),
    AP_GROUPINFO("DBG_RATE", 22, ModeAoafllow, _debug_rate_hz, 5.0f),
    AP_GROUPEND
};

ModeAoafllow::ModeAoafllow() : Mode(), // 必须首先初始化基类
                               _last_update_ms(0),
                               _emergency_stop(false),
                               _throttle_out(0.0f),
                               _steering_out(0.0f),
                               _backing_to_target(false),
                               _track_state(TrackState::ACQUIRE),
                               _last_data_ms(0),
                               _last_debug_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeAoafllow::_enter()
{
    aoa_sensor1.init(_serial_port.get());
    _dist_pid.set_gains(_dist_kp.get(), _dist_ki.get(), _dist_kd.get(), 0.01f);
    _angle_pid.set_gains(_angle_kp.get(), _angle_ki.get(), _angle_kd.get(), 0.01f);

    _last_update_ms = AP_HAL::millis();
    _last_data_ms = 0;
    _last_debug_ms = 0;
    _set_track_state(TrackState::ACQUIRE);
    reset_controllers();

    gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow ENGAGED");
    return true;
}

void ModeAoafllow::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const float dt = constrain_float((now_ms - _last_update_ms) * 0.001f, AOA_DT_MIN_S, AOA_DT_MAX_S);
    _last_update_ms = now_ms;

    float raw_dist = 0.0f;
    float raw_angle = 0.0f;
    aoa_sensor1.update();

    if (!aoa_sensor1.get_raw_data(raw_dist, raw_angle)) {
        _handle_data_loss(now_ms);
        _send_debug_throttled(now_ms,
                              false,
                              0.0f,
                              0.0f,
                              _kalman_filter.get_distance(),
                              _kalman_filter.get_angle(),
                              nullptr);
        return;
    }

    _last_data_ms = now_ms;

    // 数据恢复后重新归一化积分与滤波状态，避免掉数恢复时出现突发命令
    if ((_track_state == TrackState::ACQUIRE) || (_track_state == TrackState::LOST)) {
        reset_controllers();
    }

    _kalman_filter.predict(dt);
    _kalman_filter.update(raw_dist, raw_angle);

    const float filtered_dist = _kalman_filter.get_distance();
    const float filtered_angle = _kalman_filter.get_angle();

    if (!_safety_check(filtered_dist)) {
        _send_debug_throttled(now_ms, true, raw_dist, raw_angle, filtered_dist, filtered_angle, nullptr);
        return;
    }

    _set_track_state(TrackState::TRACK);

    const float target_heading_error_deg = wrap_180(filtered_angle - 90.0f); // 前方为0°
    float forward_dist_m = filtered_dist * cosf(radians(target_heading_error_deg));

    bool backing_to_target = false;
    float heading_error_deg = shortest_drive_heading_error(target_heading_error_deg, backing_to_target);
    if (backing_to_target != _backing_to_target) {
        _angle_pid.reset();
        _backing_to_target = backing_to_target;
    }

    const float max_dist_err = MAX(fabsf(_max_dist_err_m.get()), 0.1f);
    const float max_angle_err = MAX(fabsf(_max_angle_err_deg.get()), 1.0f);
    forward_dist_m = constrain_float(forward_dist_m, -max_dist_err, max_dist_err);
    heading_error_deg = constrain_float(heading_error_deg, -max_angle_err, max_angle_err);

    const Vector2f control_out = _calculate_control(forward_dist_m, heading_error_deg, dt);
    _set_actuators(control_out);

    _send_debug_throttled(now_ms, true, raw_dist, raw_angle, filtered_dist, filtered_angle, &control_out);
}

void ModeAoafllow::_handle_data_loss(uint32_t now_ms)
{
    if (_track_state == TrackState::ESTOP || _emergency_stop) {
        g2.motors.set_throttle(0);
        g2.motors.set_steering(0);
        return;
    }

    if (_last_data_ms == 0U) {
        _set_track_state(TrackState::ACQUIRE);
        _set_actuators(Vector2f(0.0f, 0.0f));
        return;
    }

    const float data_age_s = (now_ms - _last_data_ms) * 0.001f;
    const float lost_timeout_s = MAX(_lost_timeout_s.get(), 0.05f);
    const float hold_timeout_s = MAX(_hold_timeout_s.get(), lost_timeout_s);

    if (data_age_s < lost_timeout_s) {
        return;
    }

    if (data_age_s < hold_timeout_s) {
        _set_track_state(TrackState::LOST);
        const float decay = constrain_float(_loss_decay.get(), 0.0f, 1.0f);
        _throttle_out *= decay;
        _steering_out *= decay;
        _set_actuators(Vector2f(_throttle_out, _steering_out));
        return;
    }

    _set_track_state(TrackState::ACQUIRE);
    reset_controllers();
    _set_actuators(Vector2f(0.0f, 0.0f));
}

bool ModeAoafllow::_safety_check(float current_dist)
{
    const float estop_distance = MAX(0.0f, _target_dist.get() + MAX(0.0f, _estop_buffer_m.get()));
    const float estop_release_dist = estop_distance + MAX(0.0f, _estop_release_hyst_m.get());

    if (_track_state == TrackState::ESTOP || _emergency_stop) {
        if (current_dist > estop_release_dist) {
            _emergency_stop = false;
            _set_track_state(TrackState::ACQUIRE);
            reset_controllers();
            return true;
        }
        _emergency_stop = true;
        g2.motors.set_throttle(0);
        g2.motors.set_steering(0);
        return false;
    }

    if (current_dist < estop_distance) {
        _emergency_stop = true;
        _set_track_state(TrackState::ESTOP);
        g2.motors.set_throttle(0);
        g2.motors.set_steering(0);
        return false;
    }

    return true;
}

Vector2f ModeAoafllow::_calculate_control(float forward_dist, float heading_error_deg, float dt)
{
    const float target_dist = _target_dist.get();
    const float speed_limit = MAX(_max_speed.get(), AOA_MIN_LIMIT);
    const float steer_limit = MAX(_steer_limit.get(), AOA_MIN_LIMIT);

    // 目标在前方且距离偏大时输出正油门，距离偏小时输出负油门
    const float dist_error = forward_dist - target_dist;
    _throttle_out = _dist_pid.get_pid(dist_error, dt, 1.0f / speed_limit);
    _steering_out = _angle_pid.get_pid(heading_error_deg, dt, 1.0f / steer_limit);

    _throttle_out = constrain_float(_throttle_out, -1.0f, 1.0f);
    _steering_out = constrain_float(_steering_out, -1.0f, 1.0f);
    return Vector2f(_throttle_out, _steering_out);
}

void ModeAoafllow::_set_actuators(const Vector2f &control)
{
    if (_emergency_stop) {
        g2.motors.set_throttle(0);
        g2.motors.set_steering(0);
        return;
    }

    const float steer_deadband = constrain_float(fabsf(_steer_deadband.get()), 0.0f, 1.0f);
    const float thr_deadband = constrain_float(fabsf(_thr_deadband.get()), 0.0f, 1.0f);
    const float steer_limit = MAX(_steer_limit.get(), AOA_MIN_LIMIT);
    const float speed_limit = MAX(_max_speed.get(), AOA_MIN_LIMIT);
    const float steer_kick = MAX(0.0f, _steer_friction_offset.get());
    const float thr_kick = MAX(0.0f, _thr_friction_offset.get());

    if (fabsf(control.y) > steer_deadband) {
        float steering_cmd = constrain_float((control.y * steer_limit) * 4500.0f, -4500.0f, 4500.0f);
        steering_cmd += copysignf(steer_kick, steering_cmd);
        g2.motors.set_steering(constrain_float(steering_cmd, -4500.0f, 4500.0f));
    } else {
        g2.motors.set_steering(0);
    }

    if (fabsf(control.x) > thr_deadband) {
        float throttle_cmd = constrain_float((control.x * speed_limit) * 100.0f, -100.0f, 100.0f);
        throttle_cmd += copysignf(thr_kick, throttle_cmd);
        g2.motors.set_throttle(constrain_float(throttle_cmd, -100.0f, 100.0f));
    } else {
        g2.motors.set_throttle(0);
    }
}

void ModeAoafllow::_send_debug_throttled(uint32_t now_ms,
                                         bool have_data,
                                         float raw_dist,
                                         float raw_angle,
                                         float filtered_dist,
                                         float filtered_angle,
                                         const Vector2f *control)
{
    const float debug_rate_hz = _debug_rate_hz.get();
    if (!(debug_rate_hz > 0.0f)) {
        return;
    }

    const uint32_t period_ms = MAX(static_cast<uint32_t>(1000.0f / debug_rate_hz), AOA_DEBUG_MIN_PERIOD_MS);
    if ((now_ms - _last_debug_ms) < period_ms) {
        return;
    }
    _last_debug_ms = now_ms;

    gcs().send_named_float("aoa_st", static_cast<float>(static_cast<uint8_t>(_track_state)));
    if (have_data) {
        gcs().send_named_float("aoa_rd", raw_dist);
        gcs().send_named_float("aoa_ra", raw_angle);
    }
    gcs().send_named_float("aoa_fd", filtered_dist);
    gcs().send_named_float("aoa_fa", filtered_angle);
    if (control != nullptr) {
        gcs().send_named_float("aoa_tx", control->x);
        gcs().send_named_float("aoa_ty", control->y);
    }
}

void ModeAoafllow::_set_track_state(TrackState new_state)
{
    if (_track_state == new_state) {
        return;
    }
    _track_state = new_state;
    gcs().send_text(MAV_SEVERITY_INFO, "AOA state: %s", _state_to_string(new_state));
}

const char *ModeAoafllow::_state_to_string(TrackState state)
{
    switch (state) {
    case TrackState::ACQUIRE:
        return "ACQUIRE";
    case TrackState::TRACK:
        return "TRACK";
    case TrackState::LOST:
        return "LOST";
    case TrackState::ESTOP:
        return "ESTOP";
    default:
        return "UNKNOWN";
    }
}

void ModeAoafllow::reset_controllers()
{
    _dist_pid.reset();
    _angle_pid.reset();
    _throttle_out = 0.0f;
    _steering_out = 0.0f;
    _backing_to_target = false;
    _kalman_filter.reset();
}

void ModeAoafllow::_exit()
{
    _emergency_stop = false;
    _set_track_state(TrackState::ACQUIRE);
    reset_controllers();
    g2.motors.set_throttle(0);
    g2.motors.set_steering(0);
    gcs().send_text(MAV_SEVERITY_INFO, "AOA Follow DISENGAGED");
}
