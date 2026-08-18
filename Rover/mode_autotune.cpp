#include "Rover.h"

#include <AP_GPS/AP_GPS.h>
#include <SRV_Channel/SRV_Channel.h>

namespace {

constexpr uint32_t readiness_time_ms = 1000U;
constexpr uint32_t sample_period_ms = 20U;
constexpr uint32_t stopped_confirm_ms = 500U;
constexpr uint32_t maximum_run_time_ms = 15U * 60U * 1000U;
constexpr uint32_t rate_test_segment_ms = 3000U;
constexpr uint32_t rate_identify_min_ms = 8000U;
constexpr uint32_t rate_identify_max_ms = 20000U;
constexpr uint32_t straight_test_min_ms = 8000U;
constexpr uint32_t straight_test_max_ms = 30000U;
constexpr uint32_t straight_experiment_timeout_ms = 180000U;
constexpr uint32_t angle_step_timeout_ms = 12000U;
constexpr uint32_t stopping_timeout_ms = 10000U;
constexpr uint32_t continuous_saturation_timeout_ms = 1000U;
constexpr uint16_t minimum_model_samples = 150U;
constexpr float maximum_model_nrmse = 0.20f;
constexpr float maximum_saturation_fraction = 0.10f;
constexpr float maximum_overshoot = 0.10f;
constexpr float minimum_crawl_speed = 0.10f;
constexpr float initial_crawl_speed = 0.20f;
constexpr float minimum_x_limit = 1.0f;
constexpr float minimum_y_limit = 0.25f;
constexpr float heading_tolerance_rad = radians(3.0f);
constexpr float stopped_yaw_rate_rads = radians(5.0f);
constexpr float rls_forgetting_factor = 0.995f;
constexpr float rls_initial_covariance = 1000.0f;

const char *const managed_param_names[] = {
    "CRUISE_SPEED",
    "CRUISE_THROTTLE",
    "ATC_STR_RAT_FF",
    "ATC_STR_RAT_P",
    "ATC_STR_RAT_I",
    "ATC_STR_RAT_D",
    "ATC_STR_RAT_D_FF",
    "ATC_STR_RAT_IMAX",
    "ATC_STR_RAT_FLTT",
    "ATC_STR_RAT_FLTE",
    "ATC_STR_RAT_FLTD",
    "ATC_STR_RAT_SMAX",
    "ATC_STR_RAT_PDMX",
    "ATC_SPEED_FF",
    "ATC_SPEED_P",
    "ATC_SPEED_I",
    "ATC_SPEED_D",
    "ATC_SPEED_D_FF",
    "ATC_SPEED_IMAX",
    "ATC_SPEED_FLTT",
    "ATC_SPEED_FLTE",
    "ATC_SPEED_FLTD",
    "ATC_SPEED_SMAX",
    "ATC_SPEED_PDMX",
    "ATC_STR_ANG_P",
    "PSC_POS_P",
    "PSC_VEL_P",
    "PSC_VEL_I",
    "PSC_VEL_D",
    "PSC_VEL_FF",
    "PSC_VEL_IMAX",
    "PSC_VEL_FLTE",
    "PSC_VEL_FLTD",
};

static_assert(ARRAY_SIZE(managed_param_names) == 33,
              "AutoTune managed parameter table mismatch");

bool scalar_param_type(ap_var_type type)
{
    return type == AP_PARAM_INT8 || type == AP_PARAM_INT16 ||
           type == AP_PARAM_INT32 || type == AP_PARAM_FLOAT;
}

float parameter_tolerance(ap_var_type type, float expected)
{
    if (type != AP_PARAM_FLOAT) {
        return 0.1f;
    }
    return MAX(1.0e-5f, fabsf(expected) * 1.0e-4f);
}

} // namespace

const AP_Param::GroupInfo ModeAutoTune::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Rover full-loop AutoTune enable
    // @Description: Allows entry into the built-in Rover full control-loop AutoTune mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, ModeAutoTune, _enable, 0),

    // @Param: AREA_LEN
    // @DisplayName: AutoTune area length
    // @Description: Length of the rectangular tuning area along the vehicle heading captured when tuning starts. Zero means not configured
    // @Units: m
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("AREA_LEN", 2, ModeAutoTune, _area_length, 0.0f),

    // @Param: AREA_WID
    // @DisplayName: AutoTune area width
    // @Description: Width of the rectangular tuning area perpendicular to the vehicle heading captured when tuning starts. Zero means not configured
    // @Units: m
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("AREA_WID", 3, ModeAutoTune, _area_width, 0.0f),

    // @Param: CLEAR
    // @DisplayName: AutoTune vehicle clearance radius
    // @Description: Vehicle half diagonal plus required physical clearance. Position uncertainty is added automatically. Zero means not configured
    // @Units: m
    // @Range: 0 100
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("CLEAR", 4, ModeAutoTune, _clearance, 0.0f),

    // @Param: ACT_MAX
    // @DisplayName: AutoTune actuator output maximum
    // @Description: Maximum absolute final left or right motor output allowed while AutoTune is active. Zero means not configured
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACT_MAX", 5, ModeAutoTune, _actuator_max, 0.0f),

    AP_GROUPEND
};

ModeAutoTune::ModeAutoTune() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void ModeAutoTune::RLSModel::reset()
{
    memset(theta, 0, sizeof(theta));
    memset(covariance, 0, sizeof(covariance));
    memset(y_hist, 0, sizeof(y_hist));
    memset(u_hist, 0, sizeof(u_hist));
    for (uint8_t i = 0; i < 4; i++) {
        covariance[i][i] = rls_initial_covariance;
    }
    squared_error = 0.0f;
    output_min = 0.0f;
    output_max = 0.0f;
    samples = 0;
    saturated_samples = 0;
    history_valid = false;
}

bool ModeAutoTune::RLSModel::update(float input, float output, bool saturated)
{
    if (!isfinite(input) || !isfinite(output)) {
        return false;
    }

    if (samples == 0U) {
        output_min = output_max = output;
    } else {
        output_min = MIN(output_min, output);
        output_max = MAX(output_max, output);
    }
    saturated_samples += saturated ? 1U : 0U;

    if (samples >= 2U) {
        const float phi[4] { y_hist[0], y_hist[1], u_hist[0], u_hist[1] };
        float p_phi[4]{};
        float denominator = rls_forgetting_factor;
        for (uint8_t row = 0; row < 4; row++) {
            for (uint8_t column = 0; column < 4; column++) {
                p_phi[row] += covariance[row][column] * phi[column];
            }
            denominator += phi[row] * p_phi[row];
        }
        if (!isfinite(denominator) || denominator < 1.0e-9f) {
            return false;
        }

        float prediction = 0.0f;
        for (uint8_t i = 0; i < 4; i++) {
            prediction += phi[i] * theta[i];
        }
        const float error = output - prediction;
        if (!isfinite(error)) {
            return false;
        }
        squared_error += sq(error);

        const float inv_denominator = 1.0f / denominator;
        float gain[4]{};
        for (uint8_t i = 0; i < 4; i++) {
            gain[i] = p_phi[i] * inv_denominator;
            theta[i] += gain[i] * error;
            if (!isfinite(theta[i])) {
                return false;
            }
        }

        float next_covariance[4][4]{};
        for (uint8_t row = 0; row < 4; row++) {
            for (uint8_t column = 0; column < 4; column++) {
                float phi_p = 0.0f;
                for (uint8_t k = 0; k < 4; k++) {
                    phi_p += phi[k] * covariance[k][column];
                }
                next_covariance[row][column] =
                    (covariance[row][column] - gain[row] * phi_p) /
                    rls_forgetting_factor;
                if (!isfinite(next_covariance[row][column])) {
                    return false;
                }
            }
        }
        memcpy(covariance, next_covariance, sizeof(covariance));
    }

    y_hist[1] = y_hist[0];
    y_hist[0] = output;
    u_hist[1] = u_hist[0];
    u_hist[0] = input;
    samples++;
    history_valid = samples >= 2U;
    return true;
}

float ModeAutoTune::RLSModel::nrmse() const
{
    if (samples <= 2U) {
        return INFINITY;
    }
    const float output_range = output_max - output_min;
    if (!isfinite(output_range) || output_range < 1.0e-3f) {
        return INFINITY;
    }
    return safe_sqrt(squared_error / float(samples - 2U)) / output_range;
}

float ModeAutoTune::RLSModel::saturation_fraction() const
{
    return samples == 0U ? 0.0f : float(saturated_samples) / float(samples);
}

bool ModeAutoTune::RLSModel::stable(float &largest_pole) const
{
    const float discriminant = sq(theta[0]) + 4.0f * theta[1];
    if (!isfinite(discriminant)) {
        return false;
    }

    if (discriminant >= 0.0f) {
        const float root = safe_sqrt(discriminant);
        const float pole1 = 0.5f * (theta[0] + root);
        const float pole2 = 0.5f * (theta[0] - root);
        largest_pole = MAX(fabsf(pole1), fabsf(pole2));
    } else {
        largest_pole = safe_sqrt(MAX(-theta[1], 0.0f));
    }
    return isfinite(largest_pole) && largest_pole < 1.0f;
}

bool ModeAutoTune::RLSModel::plant(float &gain, float &time_constant) const
{
    float pole;
    if (!stable(pole)) {
        return false;
    }

    const float denominator = 1.0f - theta[0] - theta[1];
    if (fabsf(denominator) < 1.0e-4f) {
        return false;
    }
    gain = (theta[2] + theta[3]) / denominator;
    if (!isfinite(gain) || fabsf(gain) < 1.0e-4f) {
        return false;
    }

    pole = constrain_float(pole, 0.01f, 0.999f);
    time_constant = -model_sample_dt / logf(pole);
    return isfinite(time_constant) && time_constant > 0.0f;
}

bool ModeAutoTune::RLSModel::qualified(FailureReason &reason) const
{
    if (samples < minimum_model_samples ||
        !isfinite(output_max - output_min) ||
        (output_max - output_min) < 1.0e-3f) {
        reason = FailureReason::ModelData;
        return false;
    }
    float pole;
    if (!stable(pole)) {
        reason = FailureReason::ModelUnstable;
        return false;
    }
    if (nrmse() > maximum_model_nrmse) {
        reason = FailureReason::ModelFit;
        return false;
    }
    if (saturation_fraction() > maximum_saturation_fraction) {
        reason = FailureReason::Saturation;
        return false;
    }
    reason = FailureReason::None;
    return true;
}

void ModeAutoTune::Performance::reset(float normalising_scale)
{
    squared_error = 0.0f;
    scale = MAX(fabsf(normalising_scale), 1.0e-3f);
    peak = 0.0f;
    step_target = 0.0f;
    step_start_actual = 0.0f;
    maximum_overshoot = 0.0f;
    samples = 0;
    saturated_samples = 0;
}

void ModeAutoTune::Performance::update(float target, float actual, bool saturated)
{
    if (!isfinite(target) || !isfinite(actual)) {
        return;
    }
    squared_error += sq(target - actual);
    samples++;
    saturated_samples += saturated ? 1U : 0U;

    if (!is_equal(target, step_target)) {
        step_target = target;
        step_start_actual = actual;
        peak = 0.0f;
    }
    const float step_delta = step_target - step_start_actual;
    if (fabsf(step_delta) > 1.0e-3f) {
        const float direction = is_negative(step_delta) ? -1.0f : 1.0f;
        const float projected_response =
            direction * (actual - step_start_actual);
        peak = MAX(peak, projected_response);
        maximum_overshoot = MAX(maximum_overshoot,
                                MAX(peak - fabsf(step_delta), 0.0f) /
                                fabsf(step_delta));
    }
}

float ModeAutoTune::Performance::cost() const
{
    if (samples == 0U) {
        return INFINITY;
    }
    return safe_sqrt(squared_error / float(samples)) / scale;
}

float ModeAutoTune::Performance::saturation_fraction() const
{
    return samples == 0U ? 0.0f : float(saturated_samples) / float(samples);
}

const char *ModeAutoTune::failure_text(FailureReason reason)
{
    switch (reason) {
    case FailureReason::None: return "none";
    case FailureReason::Configuration: return "configuration";
    case FailureReason::VehicleType: return "vehicle type";
    case FailureReason::ControlLimits: return "control limits";
    case FailureReason::Estimator: return "estimator";
    case FailureReason::GPS: return "GPS";
    case FailureReason::ExternalVelocity: return "external velocity";
    case FailureReason::Failsafe: return "failsafe";
    case FailureReason::Boundary: return "area boundary";
    case FailureReason::Saturation: return "saturation";
    case FailureReason::ModelData: return "model data";
    case FailureReason::ModelUnstable: return "unstable model";
    case FailureReason::ModelFit: return "model fit";
    case FailureReason::Overshoot: return "overshoot";
    case FailureReason::CandidateWorse: return "candidate worse";
    case FailureReason::Persistence: return "parameter save";
    case FailureReason::Disarmed: return "disarmed";
    case FailureReason::Timeout: return "timeout";
    case FailureReason::Internal: return "internal";
    case FailureReason::Operator: return "operator";
    }
    return "unknown";
}

bool ModeAutoTune::static_checks(FailureReason &reason) const
{
    if (_enable.get() != 1 || !is_positive(_area_length) ||
        !is_positive(_area_width) || !is_positive(_clearance) ||
        !is_positive(_actuator_max) || _area_length > 1000.0f ||
        _area_width > 1000.0f || _clearance > 100.0f ||
        _actuator_max > 100.0f) {
        reason = FailureReason::Configuration;
        return false;
    }
    if (!g2.motors.have_skid_steering() || !g2.motors.pre_arm_check(false)) {
        reason = FailureReason::VehicleType;
        return false;
    }
    if (!is_positive(attitude_control.get_steer_rate_max()) ||
        !is_positive(attitude_control.get_steer_accel_max()) ||
        !is_positive(attitude_control.get_accel_max()) ||
        !is_positive(g2.motors.get_throttle_max())) {
        reason = FailureReason::ControlLimits;
        return false;
    }
    reason = FailureReason::None;
    return true;
}

bool ModeAutoTune::quicktune_active() const
{
    return _quicktune_enable_param != nullptr &&
           scalar_param_type(_quicktune_enable_type) &&
           _quicktune_enable_param->cast_to_float(_quicktune_enable_type) > 0.5f;
}

bool ModeAutoTune::standard_failsafe_active() const
{
    if (rover.failsafe.bits != 0U || rover.failsafe.triggered != 0U ||
        rover.failsafe.ekf || rover.battery.has_failsafed() ||
        SRV_Channels::get_emergency_stop()) {
        return true;
    }
#if AP_FENCE_ENABLED
    if (rover.fence.get_breaches() != 0U) {
        return true;
    }
#endif
    return false;
}

bool ModeAutoTune::update_position_uncertainty(float &uncertainty) const
{
    bool available = false;
    float maximum = 0.0f;

    float gps_accuracy;
    if (rover.gps.horizontal_accuracy(gps_accuracy) &&
        isfinite(gps_accuracy) && is_positive(gps_accuracy)) {
        maximum = gps_accuracy;
        available = true;
    }

    float ekf_accuracy;
    if (ahrs.get_horizontal_position_uncertainty(ekf_accuracy) &&
        isfinite(ekf_accuracy) && is_positive(ekf_accuracy)) {
        maximum = available ? MAX(maximum, ekf_accuracy) : ekf_accuracy;
        available = true;
    }

    uncertainty = maximum;
    return available;
}

Vector2f ModeAutoTune::ne_to_field(const Vector2f &ne) const
{
    const Vector2f delta = ne - _center_ne;
    const float cosine = cosf(_field_yaw_rad);
    const float sine = sinf(_field_yaw_rad);
    return Vector2f(delta.x * cosine + delta.y * sine,
                    -delta.x * sine + delta.y * cosine);
}

Vector2f ModeAutoTune::field_to_ne(float x, float y) const
{
    const float cosine = cosf(_field_yaw_rad);
    const float sine = sinf(_field_yaw_rad);
    return _center_ne + Vector2f(x * cosine - y * sine,
                                x * sine + y * cosine);
}

float ModeAutoTune::heading_for_direction(int8_t direction) const
{
    return wrap_PI(_field_yaw_rad + (direction < 0 ? M_PI : 0.0f));
}

bool ModeAutoTune::update_navigation_state()
{
    Vector2f position_ne;
    Vector3f velocity_ned;
    if (!ahrs.get_relative_position_NE_origin(position_ne) ||
        !ahrs.get_velocity_NED(velocity_ned)) {
        _speed_feedback_reliable = false;
        return false;
    }

    const Vector2f field_position = ne_to_field(position_ne);
    _field_x = field_position.x;
    _field_y = field_position.y;

    const float cosine = cosf(_field_yaw_rad);
    const float sine = sinf(_field_yaw_rad);
    _field_vx = velocity_ned.x * cosine + velocity_ned.y * sine;
    _field_vy = -velocity_ned.x * sine + velocity_ned.y * cosine;
    _ground_speed = velocity_ned.xy().length();
    if (!attitude_control.get_forward_speed(_speed)) {
        _speed_feedback_reliable = false;
        return false;
    }
    _yaw_rate = ahrs.get_yaw_rate_earth();
    _speed_feedback_reliable = isfinite(_speed) &&
                               isfinite(_ground_speed) &&
                               isfinite(_yaw_rate);

    float uncertainty;
    if (!update_position_uncertainty(uncertainty)) {
        return false;
    }
    // Never expand the accepted area during one run merely because the
    // reported covariance improved after the centre was captured.
    _position_uncertainty = MAX(_position_uncertainty, uncertainty);
    const float area_length = _transaction_active ?
                              _run_area_length : float(_area_length);
    const float area_width = _transaction_active ?
                             _run_area_width : float(_area_width);
    const float clearance = _transaction_active ?
                            _run_clearance : float(_clearance);
    _x_limit = 0.5f * area_length - clearance -
               2.0f * _position_uncertainty;
    _y_limit = 0.5f * area_width - clearance -
               2.0f * _position_uncertainty;
    _margin_x = _x_limit - fabsf(_field_x);
    _margin_y = _y_limit - fabsf(_field_y);
    return _speed_feedback_reliable && is_positive(_x_limit) && is_positive(_y_limit);
}

bool ModeAutoTune::vehicle_stopped()
{
    const bool stopped_now = _speed_feedback_reliable &&
        _ground_speed <= attitude_control.get_stop_speed() &&
        fabsf(_yaw_rate) <= stopped_yaw_rate_rads;
    const uint32_t now = AP_HAL::millis();
    if (!stopped_now) {
        _stopped_start_ms = 0U;
        return false;
    }
    if (_stopped_start_ms == 0U) {
        _stopped_start_ms = MAX(now, 1U);
        return false;
    }
    return now - _stopped_start_ms >= stopped_confirm_ms;
}

void ModeAutoTune::reset_stop_confirmation()
{
    _stopped_start_ms = 0U;
}

bool ModeAutoTune::heading_reached(float target_yaw)
{
    const bool reached_now = fabsf(wrap_PI(target_yaw - ahrs.get_yaw())) <=
                             heading_tolerance_rad &&
                             fabsf(_yaw_rate) <= stopped_yaw_rate_rads &&
                             _ground_speed <= attitude_control.get_stop_speed();
    const uint32_t now = AP_HAL::millis();
    if (!reached_now) {
        _angle_settled_ms = 0U;
        return false;
    }
    if (_angle_settled_ms == 0U) {
        _angle_settled_ms = MAX(now, 1U);
        return false;
    }
    return now - _angle_settled_ms >= stopped_confirm_ms;
}

bool ModeAutoTune::dynamic_checks(FailureReason &reason, bool require_stopped)
{
    if (standard_failsafe_active()) {
        reason = FailureReason::Failsafe;
        return false;
    }
    if (quicktune_active()) {
        reason = FailureReason::Configuration;
        return false;
    }
    if (ahrs.has_recent_extnav_velocity(500U)) {
        reason = FailureReason::ExternalVelocity;
        return false;
    }
    if (rover.gps.status() < AP_GPS::GPS_OK_FIX_3D || !rover.gps.is_healthy()) {
        reason = FailureReason::GPS;
        return false;
    }

    nav_filter_status filter_status{};
    ahrs.get_filter_status(filter_status);
    if (!rover.ekf_position_ok() || !filter_status.flags.attitude ||
        !filter_status.flags.horiz_vel || filter_status.flags.const_pos_mode ||
        filter_status.flags.dead_reckoning) {
        reason = FailureReason::Estimator;
        return false;
    }

    if (!update_navigation_state()) {
        reason = FailureReason::Estimator;
        return false;
    }
    if (_x_limit <= minimum_x_limit || _y_limit <= minimum_y_limit) {
        reason = FailureReason::Configuration;
        return false;
    }
    if (require_stopped &&
        (_ground_speed > attitude_control.get_stop_speed() ||
         fabsf(_yaw_rate) > stopped_yaw_rate_rads)) {
        reason = FailureReason::ControlLimits;
        return false;
    }
    reason = FailureReason::None;
    return true;
}

float ModeAutoTune::configured_deceleration() const
{
    return MAX(attitude_control.get_decel_max(), 0.01f);
}

float ModeAutoTune::conservative_deceleration() const
{
    float deceleration = configured_deceleration();
    if (_decel_samples >= 20U && is_positive(_decel_mean)) {
        const float variance = _decel_samples > 1U ?
            _decel_m2 / float(_decel_samples - 1U) : 0.0f;
        const float lower_bound = _decel_mean -
            2.0f * safe_sqrt(MAX(variance, 0.0f) / float(_decel_samples));
        deceleration = MIN(deceleration, MAX(lower_bound, 0.01f));
    }
    return MAX(deceleration, 0.01f);
}

float ModeAutoTune::stopping_distance(float speed) const
{
    const float distance = 0.5f * sq(fabsf(speed)) / conservative_deceleration();
    return _decel_samples >= 20U ? distance : 1.5f * distance;
}

float ModeAutoTune::straight_remaining_distance(int8_t direction) const
{
    return direction > 0 ? _x_limit - _field_x : _x_limit + _field_x;
}

bool ModeAutoTune::straight_needs_turn(int8_t direction) const
{
    const float reserve = stopping_distance(_speed) +
        MAX(0.20f, fabsf(_speed) * 0.5f);
    return straight_remaining_distance(direction) <= reserve;
}

float ModeAutoTune::space_limited_speed() const
{
    const float distance = MAX(0.8f * _x_limit, 0.0f);
    const float accel = MAX(attitude_control.get_accel_max(), 0.01f);
    const float decel = conservative_deceleration();
    // Reserve three seconds for an observable steady segment and 20% of the
    // usable half-leg after acceleration and braking.
    const float usable = 0.8f * distance;
    const float quadratic = (0.5f / accel) + (0.5f / decel);
    const float linear = 3.0f;
    const float discriminant = sq(linear) + 4.0f * quadratic * usable;
    if (!is_positive(quadratic) || !is_positive(discriminant)) {
        return 0.0f;
    }
    return MAX((-linear + safe_sqrt(discriminant)) / (2.0f * quadratic), 0.0f);
}

bool ModeAutoTune::runtime_safety_check(FailureReason &reason)
{
    FailureReason static_reason;
    if (!static_checks(static_reason) ||
        !is_equal(float(_area_length), _run_area_length) ||
        !is_equal(float(_area_width), _run_area_width) ||
        !is_equal(float(_clearance), _run_clearance) ||
        !is_equal(float(_actuator_max), _run_actuator_max)) {
        reason = static_reason == FailureReason::None ?
                 FailureReason::Configuration : static_reason;
        return false;
    }

    for (uint8_t i = 0; i < managed_param_count; i++) {
        const ManagedParam &managed = _managed[i];
        if (managed.param == nullptr) {
            reason = FailureReason::Internal;
            return false;
        }
        const float expected = managed.candidate_applied ?
                               managed.candidate : managed.baseline;
        const float current = managed.param->cast_to_float(managed.type);
        if (!isfinite(current) ||
            fabsf(current - expected) >
                parameter_tolerance(managed.type, expected)) {
            reason = FailureReason::Configuration;
            return false;
        }
    }

    const float current_actuator_max = MIN(_run_actuator_max,
                                            g2.motors.get_throttle_max());
    if (!is_positive(current_actuator_max)) {
        reason = FailureReason::ControlLimits;
        return false;
    }
    if (current_actuator_max < _effective_actuator_max) {
        _effective_actuator_max = current_actuator_max;
        _action_actuator_max = MIN(_action_actuator_max,
                                   _effective_actuator_max);
        _steer_action_max = MIN(_steer_action_max,
                                _effective_actuator_max);
        g2.motors.set_actuator_output_limit_pct(_effective_actuator_max);
    }

    if (!hal.util->get_soft_armed()) {
        reason = FailureReason::Disarmed;
        return false;
    }
    if (standard_failsafe_active()) {
        reason = FailureReason::Failsafe;
        return false;
    }
    if (quicktune_active()) {
        reason = FailureReason::Configuration;
        return false;
    }
    if (ahrs.has_recent_extnav_velocity(500U)) {
        reason = FailureReason::ExternalVelocity;
        return false;
    }
    if (rover.gps.status() < AP_GPS::GPS_OK_FIX_3D || !rover.gps.is_healthy()) {
        reason = FailureReason::GPS;
        return false;
    }
    nav_filter_status filter_status{};
    ahrs.get_filter_status(filter_status);
    if (!rover.ekf_position_ok() || !filter_status.flags.attitude ||
        !filter_status.flags.horiz_vel || filter_status.flags.const_pos_mode ||
        filter_status.flags.dead_reckoning || !update_navigation_state()) {
        reason = FailureReason::Estimator;
        return false;
    }

    const float velocity = safe_sqrt(sq(_field_vx) + sq(_field_vy));
    const float stop_time = velocity / conservative_deceleration();
    const float uncertainty_factor = _decel_samples >= 20U ? 1.0f : 1.5f;
    const float predicted_x = _field_x +
        0.5f * _field_vx * stop_time * uncertainty_factor;
    const float predicted_y = _field_y +
        0.5f * _field_vy * stop_time * uncertainty_factor;
    if (fabsf(_field_x) >= _x_limit || fabsf(_field_y) >= _y_limit ||
        fabsf(predicted_x) >= _x_limit || fabsf(predicted_y) >= _y_limit) {
        reason = FailureReason::Boundary;
        return false;
    }
    const uint32_t now = AP_HAL::millis();
    if (_command_saturated || g2.motors.actuator_output_limited()) {
        if (_saturation_start_ms == 0U) {
            _saturation_start_ms = MAX(now, 1U);
        } else if (now - _saturation_start_ms >= continuous_saturation_timeout_ms) {
            reason = FailureReason::Saturation;
            return false;
        }
    } else {
        _saturation_start_ms = 0U;
    }
    if (now - _run_start_ms > maximum_run_time_ms) {
        reason = FailureReason::Timeout;
        return false;
    }
    reason = FailureReason::None;
    return true;
}

bool ModeAutoTune::initialise_managed_params()
{
    for (uint8_t i = 0; i < managed_param_count; i++) {
        ap_var_type type;
        AP_Param *const param = AP_Param::find(managed_param_names[i], &type);
        if (param == nullptr || !scalar_param_type(type)) {
            return false;
        }
        _managed[i].param = param;
        _managed[i].type = type;
        _managed[i].baseline = param->cast_to_float(type);
        _managed[i].candidate = _managed[i].baseline;
        _managed[i].reason = ParamReason::Baseline;
        _managed[i].candidate_applied = false;
        if (!isfinite(_managed[i].baseline)) {
            return false;
        }
    }

    for (uint8_t i = 0; i < managed_param_count; i++) {
        log_param(i, ParamAction::Baseline, ParamReason::Baseline, 1U,
                  _managed[i].baseline);
    }
    return true;
}

void ModeAutoTune::set_candidate(ParamIndex index, float value, ParamReason reason)
{
    const uint8_t i = uint8_t(index);
    if (i >= managed_param_count || !isfinite(value)) {
        return;
    }
    _managed[i].candidate = value;
    _managed[i].reason = reason;
}

void ModeAutoTune::apply_candidate_range(ParamIndex first, ParamIndex last)
{
    const uint8_t first_index = uint8_t(first);
    const uint8_t last_index = uint8_t(last);
    for (uint8_t i = first_index; i <= last_index && i < managed_param_count; i++) {
        ManagedParam &managed = _managed[i];
        managed.param->set_float(managed.candidate, managed.type);
        managed.candidate = managed.param->cast_to_float(managed.type);
        managed.candidate_applied = true;
        log_param(i, ParamAction::Candidate, managed.reason, 1U,
                  managed.candidate);
    }
    reset_controllers();
}

void ModeAutoTune::restore_ram_baseline()
{
    for (uint8_t i = 0; i < managed_param_count; i++) {
        ManagedParam &managed = _managed[i];
        if (managed.param != nullptr) {
            managed.param->set_float(managed.baseline, managed.type);
            managed.candidate_applied = false;
            log_param(i, ParamAction::Rollback, ParamReason::Rollback, 1U,
                      managed.param->cast_to_float(managed.type));
        }
    }
    reset_controllers();
}

void ModeAutoTune::reset_controllers()
{
    AC_PID &steering = attitude_control.get_steering_rate_pid();
    steering.reset_I();
    steering.reset_filter();
    AC_PID &speed = attitude_control.get_throttle_speed_pid();
    speed.reset_I();
    speed.reset_filter();
    AC_PID_2D &velocity = g2.pos_control.get_vel_pid();
    velocity.reset_I();
    velocity.reset_filter();
    attitude_control.relax_I();
}

void ModeAutoTune::log_param(uint8_t index, ParamAction action,
                             ParamReason reason, uint8_t result,
                             float readback) const
{
#if HAL_LOGGING_ENABLED
    if (index >= managed_param_count) {
        return;
    }
    LogParam snapshot{};
    snapshot.index = index;
    snapshot.run_id = _run_id;
    memcpy(snapshot.name, managed_param_names[index],
           MIN(sizeof(snapshot.name), strlen(managed_param_names[index])));
    snapshot.action = uint8_t(action);
    snapshot.reason = uint8_t(reason);
    snapshot.result = result;
    snapshot.baseline = _managed[index].baseline;
    snapshot.candidate = _managed[index].candidate;
    snapshot.readback = readback;
    rover.Log_Write_AutoTune_Param(snapshot, action == ParamAction::Rollback || result == 0U);
#else
    (void)index;
    (void)action;
    (void)reason;
    (void)result;
    (void)readback;
#endif
}

void ModeAutoTune::start_persistence(bool baseline)
{
    _persistence_index = 0U;
    _persistence_had_error = false;
    _persistence_failure_reported = false;
    if (baseline) {
        _persistence_state = PersistenceState::SaveBaseline;
    } else {
        _candidate_persistence_started = true;
        _persistence_state = PersistenceState::SaveCandidate;
    }
}

void ModeAutoTune::background_save_update()
{
    const bool mode_active = rover.control_mode == this;
    if (_persistence_state == PersistenceState::BaselineDone && !mode_active) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "AutoTune: EEPROM baseline restored");
        _persistence_state = PersistenceState::Idle;
        _candidate_persistence_started = false;
        return;
    }
    if (_persistence_state == PersistenceState::Failed && !mode_active) {
        if (!_persistence_failure_reported) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,
                            "AutoTune: EEPROM rollback failed");
            _persistence_failure_reported = true;
        }
        return;
    }

    switch (_persistence_state) {
    case PersistenceState::Idle:
    case PersistenceState::CandidateDone:
    case PersistenceState::BaselineDone:
    case PersistenceState::Failed:
        return;

    case PersistenceState::SaveCandidate:
    case PersistenceState::SaveBaseline: {
        if (!AP_Param::save_queue_empty()) {
            return;
        }
        if (_persistence_index >= managed_param_count) {
            _persistence_index = 0U;
            _persistence_state =
                _persistence_state == PersistenceState::SaveCandidate ?
                PersistenceState::VerifyCandidate :
                PersistenceState::VerifyBaseline;
            return;
        }

        ManagedParam &managed = _managed[_persistence_index];
        const bool baseline = _persistence_state == PersistenceState::SaveBaseline;
        const float expected = baseline ? managed.baseline : managed.candidate;
        managed.param->set_float(expected, managed.type);
        managed.param->save(true);
        log_param(_persistence_index, ParamAction::Save,
                  baseline ? ParamReason::Rollback : managed.reason,
                  1U, managed.param->cast_to_float(managed.type));
        _persistence_index++;
        return;
    }

    case PersistenceState::VerifyCandidate:
    case PersistenceState::VerifyBaseline: {
        if (!AP_Param::save_queue_empty()) {
            return;
        }
        const bool baseline = _persistence_state == PersistenceState::VerifyBaseline;
        if (_persistence_index >= managed_param_count) {
            _persistence_state = baseline ? PersistenceState::BaselineDone :
                                            PersistenceState::CandidateDone;
            return;
        }

        ManagedParam &managed = _managed[_persistence_index];
        const float expected = baseline ? managed.baseline : managed.candidate;
        const bool loaded = managed.param->load();
        const float readback = managed.param->cast_to_float(managed.type);
        const bool matches = loaded && isfinite(readback) &&
            fabsf(readback - expected) <= parameter_tolerance(managed.type, expected);
        log_param(_persistence_index, ParamAction::Verify,
                  baseline ? ParamReason::Rollback : managed.reason,
                  matches ? 1U : 0U, readback);
        if (!matches) {
            if (!baseline) {
                _persistence_had_error = true;
                restore_ram_baseline();
                _persistence_index = 0U;
                _persistence_state = PersistenceState::SaveBaseline;
                gcs().send_text(MAV_SEVERITY_WARNING,
                                "AutoTune: save failed, restoring baseline");
            } else {
                _persistence_state = PersistenceState::Failed;
            }
            return;
        }
        _persistence_index++;
        return;
    }
    }
}

void ModeAutoTune::command_mixed(float throttle_pct, float steering_normalised)
{
    float throttle = constrain_float(throttle_pct * 0.01f, -1.0f, 1.0f);
    float steering = constrain_float(steering_normalised, -1.0f, 1.0f);
    const float output_limit = constrain_float(_effective_actuator_max * 0.01f,
                                               0.0f, 1.0f);
    const float action_limit = constrain_float(_action_actuator_max * 0.01f,
                                               0.0f, output_limit);
    const float mixed_peak = fabsf(throttle) + fabsf(steering);
    _action_limited = mixed_peak > action_limit + 1.0e-4f;
    _command_saturated = mixed_peak > output_limit + 1.0e-4f ||
                         g2.motors.actuator_output_limited();
    if (mixed_peak > action_limit && is_positive(mixed_peak)) {
        const float scale = action_limit / mixed_peak;
        throttle *= scale;
        steering *= scale;
    }

    g2.motors.set_throttle(throttle * 100.0f);
    g2.motors.set_steering(steering * 4500.0f);
    _last_output = fabsf(throttle) + fabsf(steering);
}

float ModeAutoTune::limit_turn_rate_target(float target_rate_rads) const
{
    const float current_rate = attitude_control.get_desired_turn_rate();
    const bool reversing = current_rate * target_rate_rads < 0.0f;
    const bool decelerating = reversing ||
                              fabsf(target_rate_rads) < fabsf(current_rate);
    const float limit_degss = decelerating ?
        attitude_control.get_steer_decel_max() :
        attitude_control.get_steer_accel_max();
    const float change_max = radians(MAX(limit_degss, 0.0f)) *
                             constrain_float(rover.G_Dt, 0.0f, 1.0f);
    const float target_this_cycle = reversing ? 0.0f : target_rate_rads;
    return constrain_float(target_this_cycle,
                           current_rate - change_max,
                           current_rate + change_max);
}

void ModeAutoTune::command_rate_speed(float turn_rate_rads, float target_speed)
{
    const float desired_speed = attitude_control.get_desired_speed_accel_limited(target_speed,
                                                                                  rover.G_Dt);
    float throttle_normalised;
    if (is_zero(desired_speed)) {
        bool stopped;
        throttle_normalised = attitude_control.get_throttle_out_stop(
            g2.motors.limit.throttle_lower,
            g2.motors.limit.throttle_upper,
            g.speed_cruise,
            g.throttle_cruise * 0.01f,
            rover.G_Dt,
            stopped);
    } else {
        throttle_normalised = attitude_control.get_throttle_out_speed(
            desired_speed,
            g2.motors.limit.throttle_lower,
            g2.motors.limit.throttle_upper,
            g.speed_cruise,
            g.throttle_cruise * 0.01f,
            rover.G_Dt);
    }
    const float steering_normalised = attitude_control.get_steering_out_rate(
        limit_turn_rate_target(turn_rate_rads),
        g2.motors.limit.steer_left,
        g2.motors.limit.steer_right,
        rover.G_Dt);
    command_mixed(throttle_normalised * 100.0f, steering_normalised);
    _last_target = turn_rate_rads;
    _last_actual = _yaw_rate;
}

void ModeAutoTune::command_heading_speed(float heading_rad, float target_speed)
{
    const float desired_speed = attitude_control.get_desired_speed_accel_limited(target_speed,
                                                                                  rover.G_Dt);
    float throttle_normalised;
    if (is_zero(desired_speed)) {
        bool stopped;
        throttle_normalised = attitude_control.get_throttle_out_stop(
            g2.motors.limit.throttle_lower,
            g2.motors.limit.throttle_upper,
            g.speed_cruise,
            g.throttle_cruise * 0.01f,
            rover.G_Dt,
            stopped);
    } else {
        throttle_normalised = attitude_control.get_throttle_out_speed(
            desired_speed,
            g2.motors.limit.throttle_lower,
            g2.motors.limit.throttle_upper,
            g.speed_cruise,
            g.throttle_cruise * 0.01f,
            rover.G_Dt);
    }
    const float steering_normalised = attitude_control.get_steering_out_heading(
        heading_rad,
        0.0f,
        g2.motors.limit.steer_left,
        g2.motors.limit.steer_right,
        rover.G_Dt);
    command_mixed(throttle_normalised * 100.0f, steering_normalised);
    _last_target = target_speed;
    _last_actual = _speed;
}

void ModeAutoTune::command_position_target(const Vector2p &position,
                                           const Vector2f &velocity,
                                           const Vector2f &acceleration)
{
    g2.pos_control.set_pos_vel_accel_target(position, velocity, acceleration);
    g2.pos_control.update(rover.G_Dt);
    command_rate_speed(g2.pos_control.get_desired_turn_rate_rads(),
                       g2.pos_control.get_desired_speed());
    _last_target = g2.pos_control.get_desired_speed();
    _last_actual = _speed;
}

bool ModeAutoTune::command_stop()
{
    bool controller_stopped = false;
    const float throttle = attitude_control.get_throttle_out_stop(
        g2.motors.limit.throttle_lower,
        g2.motors.limit.throttle_upper,
        g.speed_cruise,
        g.throttle_cruise * 0.01f,
        rover.G_Dt,
        controller_stopped);
    const float steering = attitude_control.get_steering_out_rate(
        0.0f,
        g2.motors.limit.steer_left,
        g2.motors.limit.steer_right,
        rover.G_Dt);
    command_mixed(throttle * 100.0f, steering);
    _last_target = 0.0f;
    _last_actual = _speed;
    return controller_stopped;
}

void ModeAutoTune::command_zero()
{
    g2.motors.set_throttle(0.0f);
    g2.motors.set_steering(0.0f);
    _command_saturated = false;
    _action_limited = false;
    _last_target = 0.0f;
    _last_actual = _speed;
    _last_output = 0.0f;
    attitude_control.relax_I();
}

void ModeAutoTune::update_deceleration_estimate(float command_pct)
{
    if (_sample_due && fabsf(command_pct) < 0.5f &&
        _previous_speed > _speed &&
        _speed > attitude_control.get_stop_speed()) {
        const float observed = (_previous_speed - _speed) / model_sample_dt;
        if (isfinite(observed) && observed > 0.02f && observed < 20.0f) {
            _decel_samples++;
            const float delta = observed - _decel_mean;
            _decel_mean += delta / float(_decel_samples);
            _decel_m2 += delta * (observed - _decel_mean);
        }
    }
    if (_sample_due) {
        _previous_speed = _speed;
    }
}

void ModeAutoTune::start_straight_experiment(StraightKind kind, float target)
{
    const uint32_t now = AP_HAL::millis();
    _experiment_failure_reason = FailureReason::None;
    _straight_kind = kind;
    _straight_state = StraightState::Align;
    _travel_direction = _field_x > 0.0f ? -1 : 1;
    _straight_folds = 0U;
    _straight_samples = 0U;
    _straight_start_ms = now;
    _straight_state_start_ms = now;
    _straight_run_ms = 0U;
    _straight_target = target;
    _identify_speed_limit = _crawl_speed;
    _raw_amplitude_pct = _effective_actuator_max * 0.10f;
    _raw_command_pct = 0.0f;
    _previous_speed = _speed;
    _angle_settled_ms = 0U;
    reset_stop_confirmation();
}

ModeAutoTune::ExperimentResult ModeAutoTune::update_straight_experiment()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _straight_start_ms > straight_experiment_timeout_ms) {
        _experiment_failure_reason = FailureReason::Timeout;
        return ExperimentResult::Failed;
    }

    switch (_straight_state) {
    case StraightState::Idle:
        _experiment_failure_reason = FailureReason::Internal;
        return ExperimentResult::Failed;

    case StraightState::Align: {
        const float heading = heading_for_direction(_travel_direction);
        command_heading_speed(heading, 0.0f);
        update_deceleration_estimate(g2.motors.get_throttle());
        if (heading_reached(heading)) {
            _straight_state = StraightState::Run;
            _straight_state_start_ms = now;
            _straight_run_ms = now;
            reset_stop_confirmation();
            _angle_settled_ms = 0U;
        } else if (now - _straight_state_start_ms > angle_step_timeout_ms) {
            _experiment_failure_reason = FailureReason::Timeout;
            return ExperimentResult::Failed;
        }
        return ExperimentResult::Running;
    }

    case StraightState::Run: {
        if (straight_needs_turn(_travel_direction)) {
            _straight_state = StraightState::StopForTurn;
            _straight_state_start_ms = now;
            reset_stop_confirmation();
            return ExperimentResult::Running;
        }

        const uint32_t run_elapsed = now - _straight_run_ms;
        if (_straight_kind == StraightKind::Identify) {
            const uint8_t segment = uint8_t((run_elapsed / 1500U) & 1U);
            const float target_speed = segment == 0U ?
                                       _identify_speed_limit : 0.0f;
            const float desired_speed =
                attitude_control.get_desired_speed_accel_limited(target_speed,
                                                                  rover.G_Dt);
            float throttle_request;
            if (is_zero(desired_speed)) {
                bool stopped;
                throttle_request = attitude_control.get_throttle_out_stop(
                    g2.motors.limit.throttle_lower,
                    g2.motors.limit.throttle_upper,
                    g.speed_cruise,
                    g.throttle_cruise * 0.01f,
                    rover.G_Dt,
                    stopped);
            } else {
                throttle_request = attitude_control.get_throttle_out_speed(
                    desired_speed,
                    g2.motors.limit.throttle_lower,
                    g2.motors.limit.throttle_upper,
                    g.speed_cruise,
                    g.throttle_cruise * 0.01f,
                    rover.G_Dt);
            }
            const float throttle_cap = _raw_amplitude_pct * 0.01f;
            const float throttle = constrain_float(throttle_request,
                                                   -throttle_cap,
                                                   throttle_cap);
            if (!is_equal(throttle, throttle_request)) {
                attitude_control.get_throttle_speed_pid().reset_I();
            }
            _raw_command_pct = throttle * 100.0f;
            const float steering = attitude_control.get_steering_out_heading(
                heading_for_direction(_travel_direction),
                0.0f,
                g2.motors.limit.steer_left,
                g2.motors.limit.steer_right,
                rover.G_Dt);
            command_mixed(_raw_command_pct, steering);
            _last_target = target_speed;
            _last_actual = _speed;

            if (_sample_due) {
                const float actual_command_pct = g2.motors.get_throttle();
                if (!_speed_model.update(actual_command_pct * 0.01f,
                                         _speed,
                                         output_limited_for_sample())) {
                    _experiment_failure_reason = FailureReason::ModelUnstable;
                    return ExperimentResult::Failed;
                }
                _straight_samples++;
                const float acceleration = (_speed - _previous_speed) / model_sample_dt;
                if (actual_command_pct > 0.5f && fabsf(acceleration) < 0.08f) {
                    _steady_speed_sum += _speed;
                    _steady_throttle_sum += actual_command_pct;
                    _steady_samples++;
                }
                if ((_straight_samples % 150U) == 0U &&
                    _straight_samples <= 900U) {
                    const bool weak_response =
                        (_speed_model.output_max - _speed_model.output_min) < 0.10f;
                    const float growth = weak_response ? 1.75f : 1.50f;
                    _raw_amplitude_pct = MIN(_effective_actuator_max,
                                             _raw_amplitude_pct * growth);
                    _action_actuator_max = MAX(_action_actuator_max,
                                               _raw_amplitude_pct);
                    _identify_speed_limit = MIN(space_limited_speed(),
                                                _identify_speed_limit * growth);
                    if (is_positive(g2.speed_max)) {
                        _identify_speed_limit = MIN(_identify_speed_limit,
                                                    float(g2.speed_max));
                    }
                }
            }
            update_deceleration_estimate(g2.motors.get_throttle());

            FailureReason model_reason;
            const bool qualified = _speed_model.qualified(model_reason) &&
                                   _steady_samples >= 20U;
            const uint32_t active_time_ms =
                uint32_t(_straight_samples) * sample_period_ms;
            if ((active_time_ms >= rate_identify_min_ms && qualified) ||
                active_time_ms >= straight_test_max_ms) {
                _straight_state = StraightState::FinishStop;
                _straight_state_start_ms = now;
                reset_stop_confirmation();
            }
        } else {
            const uint8_t segment = uint8_t((run_elapsed / 3000U) & 1U);
            const float target = _straight_target * (segment == 0U ? 0.60f : 1.0f);
            command_heading_speed(heading_for_direction(_travel_direction), target);
            _last_target = target;
            _last_actual = _speed;
            if (_sample_due) {
                Performance &performance =
                    _straight_kind == StraightKind::Baseline ?
                    _baseline_speed_perf : _candidate_speed_perf;
                performance.update(target, _speed,
                                   output_limited_for_sample());
                _straight_samples++;
            }
            update_deceleration_estimate(g2.motors.get_throttle());
            const uint32_t active_time_ms =
                uint32_t(_straight_samples) * sample_period_ms;
            if (active_time_ms >= straight_test_min_ms &&
                _straight_samples >= minimum_model_samples) {
                _straight_state = StraightState::FinishStop;
                _straight_state_start_ms = now;
                reset_stop_confirmation();
            }
        }
        return ExperimentResult::Running;
    }

    case StraightState::StopForTurn:
        command_stop();
        update_deceleration_estimate(g2.motors.get_throttle());
        if (vehicle_stopped()) {
            _travel_direction = -_travel_direction;
            _straight_folds++;
            if (_straight_folds > 4U) {
                _experiment_failure_reason = FailureReason::Boundary;
                return ExperimentResult::Failed;
            }
            _straight_state = StraightState::Turn;
            _straight_state_start_ms = now;
            _angle_settled_ms = 0U;
        } else if (now - _straight_state_start_ms > stopping_timeout_ms) {
            _experiment_failure_reason = FailureReason::Timeout;
            return ExperimentResult::Failed;
        }
        return ExperimentResult::Running;

    case StraightState::Turn: {
        const float heading = heading_for_direction(_travel_direction);
        command_heading_speed(heading, 0.0f);
        update_deceleration_estimate(g2.motors.get_throttle());
        if (heading_reached(heading)) {
            _straight_state = StraightState::Run;
            _straight_state_start_ms = now;
            _straight_run_ms = now;
            _angle_settled_ms = 0U;
            reset_stop_confirmation();
        } else if (now - _straight_state_start_ms > angle_step_timeout_ms) {
            _experiment_failure_reason = FailureReason::Timeout;
            return ExperimentResult::Failed;
        }
        return ExperimentResult::Running;
    }

    case StraightState::FinishStop:
        command_stop();
        update_deceleration_estimate(g2.motors.get_throttle());
        if (vehicle_stopped()) {
            _straight_state = StraightState::Idle;
            return ExperimentResult::Complete;
        }
        if (now - _straight_state_start_ms > stopping_timeout_ms) {
            _experiment_failure_reason = FailureReason::Timeout;
            return ExperimentResult::Failed;
        }
        return ExperimentResult::Running;
    }

    _experiment_failure_reason = FailureReason::Internal;
    return ExperimentResult::Failed;
}

void ModeAutoTune::start_path_experiment(bool figure_eight, bool collect_model)
{
    const uint32_t now = AP_HAL::millis();
    _experiment_failure_reason = FailureReason::None;
    _path_figure_eight = figure_eight;
    _path_collect_model = collect_model;
    _path_state = PathState::MoveToStart;
    _path_start_ms = now;
    _path_state_start_ms = now;
    _path_samples = 0U;
    _path_squared_error = 0.0f;
    _path_saturated_samples = 0U;
    const bool reuse_baseline_geometry = !figure_eight && !collect_model &&
                                         _path_reference_valid;
    if (reuse_baseline_geometry) {
        _path_half_length = _path_reference_half_length;
        _path_amplitude = _path_reference_amplitude;
        _path_speed = _path_reference_speed;
        if (_path_half_length >= _x_limit || _path_amplitude >= _y_limit) {
            _path_state = PathState::Idle;
            _experiment_failure_reason = FailureReason::Boundary;
            return;
        }
    } else {
        _path_half_length = MIN(0.60f * _x_limit,
                                MAX(1.0f, 3.0f * MAX(_selected_speed, _crawl_speed)));
        _path_amplitude = MIN(0.45f * _y_limit, 0.75f);
        _path_speed = MIN(MAX(_selected_speed, _crawl_speed), space_limited_speed());
        if (is_positive(g2.speed_max)) {
            _path_speed = MIN(_path_speed, float(g2.speed_max));
        }
    }

    if (_path_half_length < 0.5f || _path_amplitude < 0.20f ||
        _path_speed < minimum_crawl_speed) {
        _path_state = PathState::Idle;
        _experiment_failure_reason = FailureReason::Configuration;
        return;
    }

    const float accel = MAX(attitude_control.get_accel_max(), 0.1f);
    const float lat_accel = MAX(attitude_control.get_turn_lat_accel_max(), 0.1f);
    g2.pos_control.set_limits(_path_speed, accel, lat_accel, accel);
    g2.pos_control.set_turn_params(g2.turn_radius, true);
    if (!g2.pos_control.init()) {
        _path_state = PathState::Idle;
        _experiment_failure_reason = FailureReason::Estimator;
        return;
    }

    if (!figure_eight && collect_model) {
        _path_reference_valid = true;
        _path_reference_half_length = _path_half_length;
        _path_reference_amplitude = _path_amplitude;
        _path_reference_speed = _path_speed;
    }

    if (figure_eight) {
        const float speed_radius = safe_sqrt(sq(_path_half_length) +
                                              4.0f * sq(_path_amplitude));
        const float accel_radius = safe_sqrt(sq(_path_half_length) +
                                              16.0f * sq(_path_amplitude));
        const float omega_speed = _path_speed / MAX(speed_radius, 0.1f);
        const float omega_accel = safe_sqrt(lat_accel / MAX(accel_radius, 0.1f));
        const float omega = MIN(omega_speed, omega_accel);
        if (!is_positive(omega) || !isfinite(omega)) {
            _path_state = PathState::Idle;
            _experiment_failure_reason = FailureReason::Configuration;
            return;
        }
        _path_duration_ms = uint32_t(MAX(M_2PI / omega, 8.0f) * 1000.0f);
        _path_target_field.zero();
    } else {
        const float path_length_bound = safe_sqrt(sq(2.0f * _path_half_length) +
                                                    sq(M_2PI * _path_amplitude));
        const float speed_time = path_length_bound / MAX(_path_speed, 0.1f);
        const float accel_time = safe_sqrt((4.0f * sq(M_PI) * _path_amplitude) /
                                            lat_accel);
        _path_duration_ms = uint32_t(MAX(MAX(speed_time, accel_time), 8.0f) * 1000.0f);
        _path_target_field = Vector2f(-_path_half_length, 0.0f);
    }
    reset_stop_confirmation();
}

ModeAutoTune::ExperimentResult ModeAutoTune::update_path_experiment(Performance &performance)
{
    if (_path_state == PathState::Idle) {
        if (_experiment_failure_reason == FailureReason::None) {
            _experiment_failure_reason = FailureReason::Internal;
        }
        return ExperimentResult::Failed;
    }

    const uint32_t now = AP_HAL::millis();
    const Vector2f start_ne = field_to_ne(_path_target_field.x,
                                          _path_target_field.y);

    switch (_path_state) {
    case PathState::Idle:
        return ExperimentResult::Failed;

    case PathState::MoveToStart: {
        command_position_target(start_ne.topostype(), Vector2f{}, Vector2f{});
        const float distance = safe_sqrt(sq(_field_x - _path_target_field.x) +
                                         sq(_field_y - _path_target_field.y));
        if (distance < 0.20f && vehicle_stopped()) {
            _path_state = PathState::Run;
            _path_start_ms = now;
            _path_state_start_ms = now;
            reset_stop_confirmation();
        } else if (now - _path_state_start_ms > 30000U) {
            _experiment_failure_reason = FailureReason::Timeout;
            return ExperimentResult::Failed;
        }
        return ExperimentResult::Running;
    }

    case PathState::Run: {
        const float elapsed_s = float(now - _path_start_ms) * 0.001f;
        const float duration_s = float(_path_duration_ms) * 0.001f;
        const float progress = constrain_float(elapsed_s / duration_s, 0.0f, 1.0f);

        Vector2f target_field;
        Vector2f velocity_field;
        Vector2f acceleration_field;
        if (_path_figure_eight) {
            const float omega = M_2PI / duration_s;
            const float phase = omega * elapsed_s;
            target_field = Vector2f(_path_half_length * sinf(phase),
                                    _path_amplitude * sinf(2.0f * phase));
            velocity_field = Vector2f(_path_half_length * omega * cosf(phase),
                                      2.0f * _path_amplitude * omega * cosf(2.0f * phase));
            acceleration_field = Vector2f(-_path_half_length * sq(omega) * sinf(phase),
                                          -4.0f * _path_amplitude * sq(omega) * sinf(2.0f * phase));
        } else {
            const float phase = M_2PI * progress;
            target_field = Vector2f(-_path_half_length +
                                    2.0f * _path_half_length * progress,
                                    _path_amplitude * sinf(phase));
            velocity_field = Vector2f(2.0f * _path_half_length / duration_s,
                                      _path_amplitude * M_2PI / duration_s * cosf(phase));
            acceleration_field = Vector2f(0.0f,
                                          -_path_amplitude * sq(M_2PI / duration_s) * sinf(phase));
        }

        _path_target_field = target_field;
        const Vector2f target_ne = field_to_ne(target_field.x, target_field.y);
        const Vector2f velocity_ne = field_to_ne(velocity_field.x, velocity_field.y) -
                                     _center_ne;
        const Vector2f acceleration_ne = field_to_ne(acceleration_field.x,
                                                      acceleration_field.y) -
                                         _center_ne;
        command_position_target(target_ne.topostype(), velocity_ne, acceleration_ne);

        if (_sample_due) {
            const float error = safe_sqrt(sq(_field_x - target_field.x) +
                                           sq(_field_y - target_field.y));
            performance.update(0.0f, error, output_limited_for_sample());
            _path_squared_error += sq(error);
            _path_samples++;
            _path_saturated_samples += output_limited_for_sample() ? 1U : 0U;

            if (_path_collect_model) {
                Vector3f velocity_ned;
                if (!ahrs.get_velocity_NED(velocity_ned)) {
                    _experiment_failure_reason = FailureReason::Estimator;
                    return ExperimentResult::Failed;
                }
                const float lateral_velocity =
                    ahrs.earth_to_body2D(velocity_ned.xy()).y;
                const float lat_accel_max = MAX(attitude_control.get_turn_lat_accel_max(),
                                                0.1f);
                if (!_position_model.update(
                        g2.pos_control.get_desired_lat_accel() / lat_accel_max,
                        lateral_velocity,
                        output_limited_for_sample())) {
                    _experiment_failure_reason = FailureReason::ModelUnstable;
                    return ExperimentResult::Failed;
                }
            }
        }

        if (progress >= 1.0f) {
            _path_state = PathState::FinishStop;
            _path_state_start_ms = now;
            reset_stop_confirmation();
        }
        return ExperimentResult::Running;
    }

    case PathState::FinishStop:
        command_stop();
        update_deceleration_estimate(g2.motors.get_throttle());
        if (vehicle_stopped()) {
            _path_state = PathState::Idle;
            return ExperimentResult::Complete;
        }
        if (now - _path_state_start_ms > stopping_timeout_ms) {
            _experiment_failure_reason = FailureReason::Timeout;
            return ExperimentResult::Failed;
        }
        return ExperimentResult::Running;
    }

    _experiment_failure_reason = FailureReason::Internal;
    return ExperimentResult::Failed;
}

bool ModeAutoTune::derive_steering_candidate()
{
    FailureReason reason;
    if (!_steer_model.qualified(reason)) {
        return false;
    }
    float gain;
    float time_constant;
    if (!_steer_model.plant(gain, time_constant) || !is_positive(gain)) {
        return false;
    }
    const float closed_loop_time = MAX(2.5f * time_constant, 0.50f);
    const float feedforward = constrain_float(1.0f / gain, 0.0f, 3.0f);
    const float proportional = constrain_float(
        0.5f * time_constant / (gain * closed_loop_time), 0.0f, 2.0f);
    const float integral = constrain_float(
        0.5f * proportional / MAX(time_constant, 0.20f), 0.0f, 2.0f);
    const float cutoff = constrain_float(
        0.5f / (M_2PI * MAX(time_constant, 0.02f)), 1.0f, 20.0f);

    set_candidate(ParamIndex::SteerFF, feedforward, ParamReason::Identified);
    set_candidate(ParamIndex::SteerP, proportional, ParamReason::ConservativePI);
    set_candidate(ParamIndex::SteerI, integral, ParamReason::ConservativePI);
    set_candidate(ParamIndex::SteerD, 0.0f, ParamReason::ModelUnsupported);
    set_candidate(ParamIndex::SteerDFF, 0.0f, ParamReason::ModelUnsupported);
    const float steer_imax_limit = MIN(1.0f,
                                       _effective_actuator_max * 0.01f);
    set_candidate(ParamIndex::SteerIMax,
                  constrain_float(0.5f * _effective_actuator_max * 0.01f,
                                  MIN(0.10f, steer_imax_limit),
                                  steer_imax_limit),
                  ParamReason::ConservativePI);
    set_candidate(ParamIndex::SteerFiltT, cutoff, ParamReason::Identified);
    set_candidate(ParamIndex::SteerFiltE, cutoff, ParamReason::Identified);
    set_candidate(ParamIndex::SteerFiltD, 0.0f, ParamReason::ModelUnsupported);
    set_candidate(ParamIndex::SteerSMax, 0.0f, ParamReason::ModelUnsupported);
    set_candidate(ParamIndex::SteerPDMax, 0.0f, ParamReason::ModelUnsupported);
    set_candidate(ParamIndex::SteerAngleP,
                  constrain_float(1.0f / MAX(3.0f * time_constant, 0.10f),
                                  1.0f, 10.0f),
                  ParamReason::Identified);
    write_model_log(_steer_model, 2U, FailureReason::None, 0.0f,
                    gain, time_constant);
    return true;
}

bool ModeAutoTune::derive_speed_candidate()
{
    FailureReason reason;
    if (!_speed_model.qualified(reason) || _steady_samples < 20U) {
        return false;
    }

    float gain;
    float time_constant;
    if (!_speed_model.plant(gain, time_constant) || !is_positive(gain)) {
        return false;
    }
    const float steady_speed = _steady_speed_sum / float(_steady_samples);
    const float steady_throttle = _steady_throttle_sum / float(_steady_samples);
    if (!isfinite(steady_speed) || !isfinite(steady_throttle) ||
        steady_speed <= attitude_control.get_stop_speed() ||
        steady_throttle < 0.5f) {
        return false;
    }

    float selected = MIN(0.70f * steady_speed, space_limited_speed());
    if (is_positive(g2.speed_max)) {
        selected = MIN(selected, float(g2.speed_max));
    }
    if (selected < minimum_crawl_speed) {
        return false;
    }
    _selected_speed = selected;

    const float cruise_throttle_max = MIN(g2.motors.get_throttle_max(),
                                          _effective_actuator_max);
    if (!is_positive(cruise_throttle_max)) {
        return false;
    }
    const float cruise_throttle = constrain_float(
        steady_throttle * selected / steady_speed,
        MIN(5.0f, cruise_throttle_max),
        cruise_throttle_max);
    const float closed_loop_time = MAX(3.0f * time_constant, 0.60f);
    const float proportional = constrain_float(
        time_constant / (gain * closed_loop_time), 0.01f, 2.0f);
    const float integral = constrain_float(
        0.5f * proportional / MAX(time_constant, 0.25f), 0.0f, 2.0f);
    const float cutoff = constrain_float(
        0.5f / (M_2PI * MAX(time_constant, 0.02f)), 0.5f, 20.0f);

    set_candidate(ParamIndex::CruiseSpeed, selected, ParamReason::CruiseMap);
    set_candidate(ParamIndex::CruiseThrottle, cruise_throttle, ParamReason::CruiseMap);
    set_candidate(ParamIndex::SpeedFF, 0.0f, ParamReason::CruiseMap);
    set_candidate(ParamIndex::SpeedP, proportional, ParamReason::ConservativePI);
    set_candidate(ParamIndex::SpeedI, integral, ParamReason::ConservativePI);
    set_candidate(ParamIndex::SpeedD, 0.0f, ParamReason::ModelUnsupported);
    set_candidate(ParamIndex::SpeedDFF, 0.0f, ParamReason::ModelUnsupported);
    set_candidate(ParamIndex::SpeedIMax,
                  constrain_float(0.5f * cruise_throttle * 0.01f,
                                  MIN(0.05f, 0.5f * cruise_throttle_max * 0.01f),
                                  MIN(1.0f, cruise_throttle_max * 0.01f)),
                  ParamReason::ConservativePI);
    set_candidate(ParamIndex::SpeedFiltT, cutoff, ParamReason::Identified);
    set_candidate(ParamIndex::SpeedFiltE, cutoff, ParamReason::Identified);
    set_candidate(ParamIndex::SpeedFiltD, 0.0f, ParamReason::ModelUnsupported);
    set_candidate(ParamIndex::SpeedSMax, 0.0f, ParamReason::ModelUnsupported);
    set_candidate(ParamIndex::SpeedPDMax, 0.0f, ParamReason::ModelUnsupported);
    write_model_log(_speed_model, 2U, FailureReason::None, 0.0f,
                    gain, time_constant);
    return true;
}

bool ModeAutoTune::derive_position_candidate()
{
    FailureReason reason;
    if (!_position_model.qualified(reason)) {
        return false;
    }
    float gain;
    float time_constant;
    if (!_position_model.plant(gain, time_constant) || !is_positive(gain)) {
        return false;
    }
    const float baseline_cost = _baseline_position_perf.cost();
    if (!isfinite(baseline_cost)) {
        return false;
    }
    const float correction = constrain_float(1.0f + 0.5f * baseline_cost,
                                             0.80f, 1.40f);
    const float base_pos_p = _managed[uint8_t(ParamIndex::PosP)].baseline;
    const float base_vel_p = _managed[uint8_t(ParamIndex::VelP)].baseline;
    const float base_vel_i = _managed[uint8_t(ParamIndex::VelI)].baseline;
    const float base_imax = _managed[uint8_t(ParamIndex::VelIMax)].baseline;
    const float cutoff = constrain_float(
        0.5f / (M_2PI * MAX(time_constant, 0.02f)), 0.5f, 10.0f);
    const float vel_p = constrain_float(MAX(base_vel_p, 0.1f) * correction,
                                        0.1f, 6.0f);

    set_candidate(ParamIndex::PosP,
                  constrain_float(MAX(base_pos_p, 0.2f) * correction,
                                  0.2f, 2.0f),
                  ParamReason::PathFit);
    set_candidate(ParamIndex::VelP, vel_p, ParamReason::PathFit);
    set_candidate(ParamIndex::VelI,
                  constrain_float(MAX(base_vel_i, 0.10f * vel_p), 0.0f, 1.0f),
                  ParamReason::PathFit);
    set_candidate(ParamIndex::VelD, 0.0f, ParamReason::ModelUnsupported);
    set_candidate(ParamIndex::VelFF, 0.0f, ParamReason::ModelUnsupported);
    set_candidate(ParamIndex::VelIMax,
                  constrain_float(MAX(base_imax, 0.25f),
                                  0.0f,
                                  MAX(attitude_control.get_turn_lat_accel_max(), 0.25f)),
                  ParamReason::PathFit);
    set_candidate(ParamIndex::VelFiltE, cutoff, ParamReason::Identified);
    set_candidate(ParamIndex::VelFiltD, cutoff, ParamReason::Identified);
    write_model_log(_position_model, 2U, FailureReason::None, 0.0f,
                    gain, time_constant);
    return true;
}

bool ModeAutoTune::evaluate_candidate() const
{
    const float baseline_cost = _baseline_rate_perf.cost() +
                                _baseline_speed_perf.cost() +
                                _baseline_position_perf.cost();
    const float candidate_cost = _candidate_rate_perf.cost() +
                                 _candidate_speed_perf.cost() +
                                 _candidate_position_perf.cost();
    if (!isfinite(baseline_cost) || !isfinite(candidate_cost) ||
        candidate_cost > baseline_cost) {
        return false;
    }
    if (_candidate_rate_perf.maximum_overshoot > maximum_overshoot ||
        _candidate_speed_perf.maximum_overshoot > maximum_overshoot ||
        _angle_overshoot > maximum_overshoot) {
        return false;
    }
    if (_candidate_rate_perf.saturation_fraction() > maximum_saturation_fraction ||
        _candidate_speed_perf.saturation_fraction() > maximum_saturation_fraction ||
        _candidate_position_perf.saturation_fraction() > maximum_saturation_fraction ||
        _final_perf.saturation_fraction() > maximum_saturation_fraction) {
        return false;
    }
    return _candidate_position_perf.cost() <= maximum_model_nrmse &&
           _final_perf.cost() <= maximum_model_nrmse;
}

void ModeAutoTune::write_model_log(const RLSModel &model, uint8_t event,
                                   FailureReason reason, float overshoot,
                                   float gain, float time_constant) const
{
#if HAL_LOGGING_ENABLED
    LogModel snapshot{};
    snapshot.stage = uint8_t(_stage);
    snapshot.event = event;
    snapshot.reason = uint8_t(reason);
    snapshot.run_id = _run_id;
    snapshot.samples = model.samples;
    memcpy(snapshot.theta, model.theta, sizeof(snapshot.theta));
    snapshot.fit = model.nrmse();
    snapshot.saturation = model.saturation_fraction();
    snapshot.overshoot = overshoot;
    snapshot.gain = gain;
    snapshot.time_constant = time_constant;
    rover.Log_Write_AutoTune_Model(snapshot, event != 0U || reason != FailureReason::None);
#else
    (void)model;
    (void)event;
    (void)reason;
    (void)overshoot;
    (void)gain;
    (void)time_constant;
#endif
}

void ModeAutoTune::write_sample_log() const
{
#if HAL_LOGGING_ENABLED
    LogSample snapshot{};
    snapshot.stage = uint8_t(_stage);
    snapshot.source = 1U;
    snapshot.flags = 0U;
    snapshot.flags |= hal.util->get_soft_armed() ? 1U << 0 : 0U;
    snapshot.flags |= rover.gps.is_healthy() ? 1U << 1 : 0U;
    snapshot.flags |= rover.ekf_position_ok() ? 1U << 2 : 0U;
    snapshot.flags |= output_limited_for_sample() ? 1U << 3 : 0U;
    snapshot.flags |= (_stage == Stage::SteerIdentify ||
                       _stage == Stage::SpeedIdentify ||
                       _stage == Stage::PositionBaseline) ? 1U << 4 : 0U;
    snapshot.flags |= (_margin_x < 0.5f || _margin_y < 0.5f) ? 1U << 5 : 0U;
    snapshot.flags |= ahrs.has_recent_extnav_velocity(500U) ? 1U << 6 : 0U;
    snapshot.run_id = _run_id;
    snapshot.target = _last_target;
    snapshot.actual = _last_actual;
    snapshot.output = _last_output;
    snapshot.position_x = _field_x;
    snapshot.position_y = _field_y;
    snapshot.speed = _speed;
    snapshot.yaw_rate = _yaw_rate;
    snapshot.margin_x = _margin_x;
    snapshot.margin_y = _margin_y;
    snapshot.position_uncertainty = _position_uncertainty;
    rover.Log_Write_AutoTune_Sample(snapshot);
#endif
}

bool ModeAutoTune::_enter()
{
    FailureReason reason;
    if (!static_checks(reason)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AutoTune: invalid %s",
                        failure_text(reason));
        return false;
    }
    if (_persistence_state != PersistenceState::Idle) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "AutoTune: parameter recovery busy");
        return false;
    }

    _effective_actuator_max = MIN(float(_actuator_max),
                                  g2.motors.get_throttle_max());
    _action_actuator_max = _effective_actuator_max;
    g2.motors.set_actuator_output_limit_pct(_effective_actuator_max);
    if (hal.util->get_soft_armed()) {
        command_stop();
    } else {
        command_zero();
    }
    _quicktune_enable_param = AP_Param::find("RTUN_ENABLE",
                                             &_quicktune_enable_type);
    _stage = Stage::Standby;
    _failure_reason = FailureReason::None;
    _phase = 0U;
    _retry_count = 0U;
    _ready_start_ms = 0U;
    _stopped_start_ms = 0U;
    _last_sample_ms = 0U;
    _last_wait_message_ms = 0U;
    _last_wait_reason = FailureReason::None;
    _transaction_active = false;
    _transaction_committed = false;
    _candidate_persistence_started = false;
    _steer_candidate_ready = false;
    _manual_exit = false;
    _position_uncertainty = 0.0f;
    _run_area_length = 0.0f;
    _run_area_width = 0.0f;
    _run_clearance = 0.0f;
    _run_actuator_max = 0.0f;
    _saturation_start_ms = 0U;
    _path_reference_valid = false;
    _experiment_failure_reason = FailureReason::None;
    gcs().send_text(MAV_SEVERITY_NOTICE,
                    hal.util->get_soft_armed() ?
                    "AutoTune: waiting for checks" :
                    "AutoTune: standby, arm to start");
    return true;
}

void ModeAutoTune::_exit()
{
    command_zero();
    g2.motors.clear_actuator_output_limit();

    if (_transaction_active && !_transaction_committed) {
        _manual_exit = true;
        restore_ram_baseline();
        if (_candidate_persistence_started) {
            start_persistence(true);
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "AutoTune: operator exit, restoring baseline");
        } else {
            gcs().send_text(MAV_SEVERITY_NOTICE,
                            "AutoTune: operator exit, baseline restored");
        }
    }
    _transaction_active = false;
    _stage = Stage::Standby;
}

void ModeAutoTune::set_stage(Stage stage, const char *message)
{
    _stage = stage;
    _phase = 0U;
    _stage_start_ms = AP_HAL::millis();
    reset_stop_confirmation();
    _angle_settled_ms = 0U;
    if (message != nullptr) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "AutoTune: %s", message);
    }
#if HAL_LOGGING_ENABLED
    LogModel snapshot{};
    snapshot.stage = uint8_t(stage);
    snapshot.event = 1U;
    snapshot.reason = uint8_t(_failure_reason);
    snapshot.run_id = _run_id;
    rover.Log_Write_AutoTune_Model(snapshot, true);
#endif
}

void ModeAutoTune::begin_run()
{
    _run_id++;
    if (!ahrs.get_relative_position_NE_origin(_center_ne) ||
        !ahrs.get_location(_center_location)) {
        abort_run(FailureReason::Estimator, false);
        return;
    }
    _field_yaw_rad = ahrs.get_yaw();
    _run_area_length = float(_area_length);
    _run_area_width = float(_area_width);
    _run_clearance = float(_clearance);
    _run_actuator_max = float(_actuator_max);
    _position_uncertainty = 0.0f;
    if (!update_navigation_state()) {
        abort_run(FailureReason::Estimator, false);
        return;
    }
    if (_x_limit <= minimum_x_limit || _y_limit <= minimum_y_limit) {
        abort_run(FailureReason::Configuration, true);
        return;
    }

    _effective_actuator_max = MIN(_run_actuator_max,
                                  g2.motors.get_throttle_max());
    if (!is_positive(_effective_actuator_max)) {
        abort_run(FailureReason::ControlLimits, true);
        return;
    }
    _action_actuator_max = 0.10f * _effective_actuator_max;
    _steer_action_max = 0.0f;
    _crawl_speed = MIN(initial_crawl_speed, space_limited_speed());
    if (is_positive(g2.speed_max)) {
        _crawl_speed = MIN(_crawl_speed, float(g2.speed_max));
    }
    if (_crawl_speed < minimum_crawl_speed) {
        abort_run(FailureReason::Configuration, true);
        return;
    }
    _baseline_speed_target = _crawl_speed;
    _selected_speed = _crawl_speed;
    _rate_target = MIN(radians(15.0f),
                       radians(0.25f * attitude_control.get_steer_rate_max()));
    if (_rate_target < radians(3.0f)) {
        abort_run(FailureReason::ControlLimits, true);
        return;
    }

    if (!initialise_managed_params()) {
        abort_run(FailureReason::Internal, true);
        return;
    }

    _steer_model.reset();
    _speed_model.reset();
    _position_model.reset();
    _baseline_rate_perf.reset(_rate_target);
    _candidate_rate_perf.reset(_rate_target);
    _baseline_speed_perf.reset(_baseline_speed_target);
    _candidate_speed_perf.reset(_baseline_speed_target);
    _baseline_position_perf.reset(1.0f);
    _candidate_position_perf.reset(1.0f);
    _final_perf.reset(1.0f);
    _decel_mean = 0.0f;
    _decel_m2 = 0.0f;
    _decel_samples = 0U;
    _steady_speed_sum = 0.0f;
    _steady_throttle_sum = 0.0f;
    _steady_samples = 0U;
    _angle_overshoot = 0.0f;
    _saturation_start_ms = 0U;
    _path_reference_valid = false;
    _experiment_failure_reason = FailureReason::None;
    _failure_reason = FailureReason::None;
    _transaction_active = true;
    _transaction_committed = false;
    _candidate_persistence_started = false;
    _steer_candidate_ready = false;
    _manual_exit = false;
    _run_start_ms = AP_HAL::millis();
    _last_sample_ms = _run_start_ms;
    g2.motors.set_actuator_output_limit_pct(_effective_actuator_max);
    set_stage(Stage::BaselineRate, "baseline rate test");
}

void ModeAutoTune::abort_run(FailureReason reason, bool speed_feedback_reliable)
{
    if (_stage == Stage::Stopping || _stage == Stage::Rollback ||
        _stage == Stage::Commit || _stage == Stage::Complete) {
        return;
    }
    _failure_reason = reason == FailureReason::None ?
                      FailureReason::Internal : reason;
    _speed_feedback_reliable = speed_feedback_reliable;
    _action_actuator_max = _effective_actuator_max;
    gcs().send_text(MAV_SEVERITY_WARNING, "AutoTune: abort %s",
                    failure_text(_failure_reason));
    set_stage(Stage::Stopping, "controlled stop");
    if (!speed_feedback_reliable) {
        command_zero();
    }
}

void ModeAutoTune::finish_failure()
{
    _transaction_active = false;
    _transaction_committed = false;
    g2.motors.clear_actuator_output_limit();
    if (_failure_reason == FailureReason::Persistence &&
        _persistence_state == PersistenceState::Failed) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,
                        "AutoTune: EEPROM rollback incomplete");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "AutoTune: rolled back (%s)",
                        failure_text(_failure_reason));
    }
    set_stage(Stage::Complete, nullptr);
    rover.set_mode(rover.mode_hold, ModeReason::UNKNOWN);
}

void ModeAutoTune::finish_success()
{
    _transaction_committed = true;
    _transaction_active = false;
    _persistence_state = PersistenceState::Idle;
    _candidate_persistence_started = false;
    g2.motors.clear_actuator_output_limit();
    gcs().send_text(MAV_SEVERITY_NOTICE, "AutoTune: complete, parameters saved");
    set_stage(Stage::Complete, nullptr);
    rover.set_mode(rover.mode_hold, ModeReason::UNKNOWN);
}

void ModeAutoTune::update_baseline_rate()
{
    const uint32_t now = AP_HAL::millis();
    if (_phase == 0U) {
        command_stop();
        if (vehicle_stopped()) {
            _baseline_rate_perf.reset(_rate_target);
            _stage_start_ms = now;
            _phase = 1U;
            reset_stop_confirmation();
        } else if (now - _stage_start_ms > stopping_timeout_ms) {
            abort_run(FailureReason::Timeout, _speed_feedback_reliable);
        }
        return;
    }
    if (_phase == 1U) {
        const uint8_t segment = uint8_t((now - _stage_start_ms) /
                                        rate_test_segment_ms);
        const float target = (segment & 1U) == 0U ? _rate_target : -_rate_target;
        command_rate_speed(target, 0.0f);
        if (_sample_due) {
            _baseline_rate_perf.update(target, _yaw_rate,
                                       output_limited_for_sample());
        }
        if (segment >= 4U) {
            _phase = 2U;
            _stage_start_ms = now;
            reset_stop_confirmation();
        }
        return;
    }

    command_stop();
    if (vehicle_stopped()) {
        if (_steer_candidate_ready) {
            apply_candidate_range(ParamIndex::SteerFF,
                                  ParamIndex::SteerPDMax);
            _candidate_rate_perf.reset(_rate_target);
            _steer_candidate_ready = false;
            _retry_count = 0U;
            set_stage(Stage::SteerValidate, "steering validation");
        } else {
            _retry_count = 0U;
            _steer_model.reset();
            set_stage(Stage::SteerIdentify, "steering identification");
        }
    } else if (now - _stage_start_ms > stopping_timeout_ms) {
        abort_run(FailureReason::Timeout, _speed_feedback_reliable);
    }
}

void ModeAutoTune::update_steer_identify()
{
    const uint32_t now = AP_HAL::millis();
    if (_phase == 0U) {
        command_stop();
        if (vehicle_stopped()) {
            _steer_model.reset();
            if (_retry_count == 0U) {
                _raw_amplitude_pct = 0.10f * _effective_actuator_max;
            }
            _stage_start_ms = now;
            _phase = 1U;
            reset_stop_confirmation();
        } else if (now - _stage_start_ms > stopping_timeout_ms) {
            abort_run(FailureReason::Timeout, _speed_feedback_reliable);
        }
        return;
    }
    if (_phase == 1U) {
        const uint8_t segment = uint8_t(((now - _stage_start_ms) / 750U) & 1U);
        const float target_rate = segment == 0U ? _rate_target : -_rate_target;
        const float steering_request = attitude_control.get_steering_out_rate(
            limit_turn_rate_target(target_rate),
            g2.motors.limit.steer_left,
            g2.motors.limit.steer_right,
            rover.G_Dt);
        const float steering_cap = _raw_amplitude_pct * 0.01f;
        const float steering = constrain_float(steering_request,
                                               -steering_cap,
                                               steering_cap);
        if (!is_equal(steering, steering_request)) {
            attitude_control.get_steering_rate_pid().reset_I();
        }
        command_mixed(0.0f, steering);
        _last_target = target_rate;
        _last_actual = _yaw_rate;
        if (_sample_due) {
            const float actual_steering = g2.motors.get_steering() / 4500.0f;
            if (!_steer_model.update(actual_steering, _yaw_rate,
                                     output_limited_for_sample())) {
                abort_run(FailureReason::ModelUnstable, true);
                return;
            }
            if ((_steer_model.samples % 150U) == 0U &&
                _steer_model.samples <= 900U &&
                (_steer_model.output_max - _steer_model.output_min) < radians(5.0f)) {
                _raw_amplitude_pct = MIN(_effective_actuator_max,
                                         1.75f * _raw_amplitude_pct);
                _action_actuator_max = MAX(_action_actuator_max,
                                           _raw_amplitude_pct);
            }
        }

        FailureReason model_reason;
        const float observed_rate = MAX(fabsf(_steer_model.output_min),
                                        fabsf(_steer_model.output_max));
        const bool qualified = _steer_model.qualified(model_reason) &&
                               (0.70f * observed_rate) >= radians(3.0f);
        const uint32_t elapsed = now - _stage_start_ms;
        if ((elapsed >= rate_identify_min_ms && qualified) ||
            elapsed >= rate_identify_max_ms) {
            _phase = 2U;
            _stage_start_ms = now;
            reset_stop_confirmation();
        }
        return;
    }

    command_stop();
    if (!vehicle_stopped()) {
        if (now - _stage_start_ms > stopping_timeout_ms) {
            abort_run(FailureReason::Timeout, _speed_feedback_reliable);
        }
        return;
    }
    FailureReason reason;
    const bool model_qualified = _steer_model.qualified(reason);
    const float observed_rate = MAX(fabsf(_steer_model.output_min),
                                    fabsf(_steer_model.output_max));
    const float selected_rate = MIN(_rate_target, 0.70f * observed_rate);
    const bool candidate_derived = model_qualified &&
                                   selected_rate >= radians(3.0f) &&
                                   derive_steering_candidate();
    if (!candidate_derived) {
        if (model_qualified) {
            reason = FailureReason::ModelData;
        }
        write_model_log(_steer_model, 3U, reason, 0.0f, 0.0f, 0.0f);
        if (_retry_count == 0U) {
            _retry_count = 1U;
            _raw_amplitude_pct *= 0.70f;
            _action_actuator_max = _raw_amplitude_pct;
            _phase = 0U;
            gcs().send_text(MAV_SEVERITY_NOTICE,
                            "AutoTune: steering retry at lower output");
            return;
        }
        abort_run(reason, true);
        return;
    }

    _rate_target = selected_rate;
    _steer_action_max = _action_actuator_max;
    _steer_candidate_ready = true;
    _retry_count = 0U;
    set_stage(Stage::BaselineRate, "baseline rate recheck");
}

void ModeAutoTune::update_steer_validate()
{
    const uint32_t now = AP_HAL::millis();
    if (_phase == 0U) {
        command_stop();
        if (vehicle_stopped()) {
            _candidate_rate_perf.reset(_rate_target);
            _stage_start_ms = now;
            _phase = 1U;
            reset_stop_confirmation();
        } else if (now - _stage_start_ms > stopping_timeout_ms) {
            abort_run(FailureReason::Timeout, _speed_feedback_reliable);
        }
        return;
    }
    if (_phase == 1U) {
        const uint8_t segment = uint8_t((now - _stage_start_ms) /
                                        rate_test_segment_ms);
        const float target = (segment & 1U) == 0U ? _rate_target : -_rate_target;
        command_rate_speed(target, 0.0f);
        if (_sample_due) {
            _candidate_rate_perf.update(target, _yaw_rate,
                                        output_limited_for_sample());
        }
        if (segment >= 4U) {
            _phase = 2U;
            _stage_start_ms = now;
            reset_stop_confirmation();
        }
        return;
    }

    command_stop();
    if (!vehicle_stopped()) {
        if (now - _stage_start_ms > stopping_timeout_ms) {
            abort_run(FailureReason::Timeout, _speed_feedback_reliable);
        }
        return;
    }
    const bool acceptable =
        _candidate_rate_perf.maximum_overshoot <= maximum_overshoot &&
        _candidate_rate_perf.saturation_fraction() <= maximum_saturation_fraction &&
        _candidate_rate_perf.cost() <= _baseline_rate_perf.cost();
    if (!acceptable) {
        if (_retry_count == 0U) {
            _retry_count = 1U;
            set_candidate(ParamIndex::SteerP,
                          0.75f * _managed[uint8_t(ParamIndex::SteerP)].candidate,
                          ParamReason::ConservativePI);
            set_candidate(ParamIndex::SteerI,
                          0.75f * _managed[uint8_t(ParamIndex::SteerI)].candidate,
                          ParamReason::ConservativePI);
            apply_candidate_range(ParamIndex::SteerP, ParamIndex::SteerI);
            _phase = 0U;
            gcs().send_text(MAV_SEVERITY_NOTICE,
                            "AutoTune: steering gains reduced for retry");
            return;
        }
        abort_run(_candidate_rate_perf.maximum_overshoot > maximum_overshoot ?
                  FailureReason::Overshoot : FailureReason::CandidateWorse,
                  true);
        return;
    }

    apply_candidate_range(ParamIndex::SteerAngleP, ParamIndex::SteerAngleP);
    _retry_count = 0U;
    set_stage(Stage::AngleValidate, "heading validation");
}

void ModeAutoTune::update_angle_validate()
{
    const uint32_t now = AP_HAL::millis();
    constexpr float angle_step = radians(30.0f);

    if (_phase == 0U) {
        command_stop();
        if (vehicle_stopped()) {
            _angle_base_yaw = ahrs.get_yaw();
            _angle_step_start_yaw = _angle_base_yaw;
            _angle_target_yaw = wrap_PI(_angle_base_yaw + angle_step);
            _angle_peak = 0.0f;
            _angle_overshoot = 0.0f;
            _stage_start_ms = now;
            _phase = 1U;
            _angle_settled_ms = 0U;
        } else if (now - _stage_start_ms > stopping_timeout_ms) {
            abort_run(FailureReason::Timeout, _speed_feedback_reliable);
        }
        return;
    }

    command_heading_speed(_angle_target_yaw, 0.0f);
    const float signed_step = wrap_PI(_angle_target_yaw - _angle_step_start_yaw);
    const float step_size = fabsf(signed_step);
    const float direction = is_negative(signed_step) ? -1.0f : 1.0f;
    _last_target = signed_step;
    _last_actual = wrap_PI(ahrs.get_yaw() - _angle_step_start_yaw);

    const float travelled = direction * _last_actual;
    _angle_peak = MAX(_angle_peak, travelled);
    if (step_size > 1.0e-3f) {
        _angle_overshoot = MAX(_angle_overshoot,
                              MAX(_angle_peak - step_size, 0.0f) / step_size);
    }

    if (heading_reached(_angle_target_yaw)) {
        if (_phase == 1U) {
            _phase = 2U;
            _angle_step_start_yaw = ahrs.get_yaw();
            _angle_target_yaw = wrap_PI(_angle_base_yaw - angle_step);
            _angle_peak = 0.0f;
            _stage_start_ms = now;
            _angle_settled_ms = 0U;
            return;
        }
        if (_phase == 2U) {
            _phase = 3U;
            _angle_step_start_yaw = ahrs.get_yaw();
            _angle_target_yaw = _angle_base_yaw;
            _stage_start_ms = now;
            _angle_settled_ms = 0U;
            return;
        }

        if (_angle_overshoot > maximum_overshoot) {
            if (_retry_count == 0U) {
                _retry_count = 1U;
                set_candidate(ParamIndex::SteerAngleP,
                              0.75f * _managed[uint8_t(ParamIndex::SteerAngleP)].candidate,
                              ParamReason::ConservativePI);
                apply_candidate_range(ParamIndex::SteerAngleP,
                                      ParamIndex::SteerAngleP);
                _phase = 0U;
                gcs().send_text(MAV_SEVERITY_NOTICE,
                                "AutoTune: heading gain reduced for retry");
                return;
            }
            abort_run(FailureReason::Overshoot, true);
            return;
        }

        _retry_count = 0U;
        set_stage(Stage::SpeedBaseline, "baseline speed test");
        return;
    }

    if (now - _stage_start_ms > angle_step_timeout_ms) {
        abort_run(FailureReason::Timeout, true);
    }
}

void ModeAutoTune::update_speed_baseline()
{
    if (_phase == 0U) {
        _baseline_speed_perf.reset(_baseline_speed_target);
        start_straight_experiment(StraightKind::Baseline,
                                  _baseline_speed_target);
        _phase = 1U;
    }
    const ExperimentResult result = update_straight_experiment();
    if (result == ExperimentResult::Failed) {
        abort_run(_experiment_failure_reason, _speed_feedback_reliable);
        return;
    }
    if (result == ExperimentResult::Complete) {
        _retry_count = 0U;
        _speed_model.reset();
        _steady_speed_sum = 0.0f;
        _steady_throttle_sum = 0.0f;
        _steady_samples = 0U;
        set_stage(Stage::SpeedIdentify, "speed identification");
    }
}

void ModeAutoTune::update_speed_identify()
{
    if (_phase == 0U) {
        _speed_model.reset();
        _steady_speed_sum = 0.0f;
        _steady_throttle_sum = 0.0f;
        _steady_samples = 0U;
        start_straight_experiment(StraightKind::Identify, _crawl_speed);
        if (_retry_count != 0U) {
            _raw_amplitude_pct = 0.07f * _effective_actuator_max;
        }
        _action_actuator_max = MAX(_steer_action_max,
                                   _raw_amplitude_pct);
        _phase = 1U;
    }
    const ExperimentResult result = update_straight_experiment();
    if (result == ExperimentResult::Failed) {
        abort_run(_experiment_failure_reason, _speed_feedback_reliable);
        return;
    }
    if (result != ExperimentResult::Complete) {
        return;
    }

    if (_phase == 1U) {
        FailureReason reason;
        const bool model_qualified = _speed_model.qualified(reason);
        const bool candidate_derived = model_qualified && derive_speed_candidate();
        if (!candidate_derived) {
            if (model_qualified) {
                reason = FailureReason::ModelData;
            }
            write_model_log(_speed_model, 3U, reason, 0.0f, 0.0f, 0.0f);
            if (_retry_count == 0U) {
                _retry_count = 1U;
                _phase = 0U;
                gcs().send_text(MAV_SEVERITY_NOTICE,
                                "AutoTune: speed retry at lower output");
                return;
            }
            abort_run(reason, true);
            return;
        }

        // Re-measure the existing speed controller at the automatically
        // selected cruise speed.  The candidate is then tested with exactly
        // the same target profile and field geometry.
        _baseline_speed_perf.reset(_selected_speed);
        start_straight_experiment(StraightKind::Baseline, _selected_speed);
        _phase = 2U;
        return;
    }

    apply_candidate_range(ParamIndex::CruiseSpeed, ParamIndex::CruiseThrottle);
    apply_candidate_range(ParamIndex::SpeedFF, ParamIndex::SpeedPDMax);
    _candidate_speed_perf.reset(_selected_speed);
    _retry_count = 0U;
    set_stage(Stage::SpeedValidate, "speed validation");
}

void ModeAutoTune::update_speed_validate()
{
    if (_phase == 0U) {
        _candidate_speed_perf.reset(_selected_speed);
        start_straight_experiment(StraightKind::Validate,
                                  _selected_speed);
        _phase = 1U;
    }
    const ExperimentResult result = update_straight_experiment();
    if (result == ExperimentResult::Failed) {
        abort_run(_experiment_failure_reason, _speed_feedback_reliable);
        return;
    }
    if (result != ExperimentResult::Complete) {
        return;
    }

    const bool acceptable =
        _candidate_speed_perf.maximum_overshoot <= maximum_overshoot &&
        _candidate_speed_perf.saturation_fraction() <= maximum_saturation_fraction &&
        _candidate_speed_perf.cost() <= _baseline_speed_perf.cost();
    if (!acceptable) {
        if (_retry_count == 0U) {
            _retry_count = 1U;
            set_candidate(ParamIndex::SpeedP,
                          0.75f * _managed[uint8_t(ParamIndex::SpeedP)].candidate,
                          ParamReason::ConservativePI);
            set_candidate(ParamIndex::SpeedI,
                          0.75f * _managed[uint8_t(ParamIndex::SpeedI)].candidate,
                          ParamReason::ConservativePI);
            apply_candidate_range(ParamIndex::SpeedP, ParamIndex::SpeedI);
            _phase = 0U;
            gcs().send_text(MAV_SEVERITY_NOTICE,
                            "AutoTune: speed gains reduced for retry");
            return;
        }
        abort_run(_candidate_speed_perf.maximum_overshoot > maximum_overshoot ?
                  FailureReason::Overshoot : FailureReason::CandidateWorse,
                  true);
        return;
    }

    _retry_count = 0U;
    _position_model.reset();
    set_stage(Stage::PositionBaseline, "baseline position path");
}

void ModeAutoTune::update_position_baseline()
{
    if (_phase == 0U) {
        _position_model.reset();
        start_path_experiment(false, true);
        if (_path_state == PathState::Idle) {
            abort_run(_experiment_failure_reason, true);
            return;
        }
        _baseline_position_perf.reset(_path_amplitude);
        _phase = 1U;
    }
    const ExperimentResult result = update_path_experiment(_baseline_position_perf);
    if (result == ExperimentResult::Failed) {
        const bool retryable =
            _experiment_failure_reason == FailureReason::ModelData ||
            _experiment_failure_reason == FailureReason::ModelUnstable ||
            _experiment_failure_reason == FailureReason::ModelFit;
        if (_retry_count == 0U && retryable) {
            _retry_count = 1U;
            _selected_speed = MAX(0.8f * _selected_speed, minimum_crawl_speed);
            _phase = 0U;
            gcs().send_text(MAV_SEVERITY_NOTICE,
                            "AutoTune: path retry at lower speed");
            return;
        }
        abort_run(_experiment_failure_reason, _speed_feedback_reliable);
        return;
    }
    if (result != ExperimentResult::Complete) {
        return;
    }

    FailureReason reason;
    const bool model_qualified = _position_model.qualified(reason);
    const bool candidate_derived = model_qualified && derive_position_candidate();
    if (!candidate_derived) {
        if (model_qualified) {
            reason = FailureReason::ModelData;
        }
        write_model_log(_position_model, 3U, reason, 0.0f, 0.0f, 0.0f);
        if (_retry_count == 0U) {
            _retry_count = 1U;
            _selected_speed = MAX(0.8f * _selected_speed, minimum_crawl_speed);
            _phase = 0U;
            gcs().send_text(MAV_SEVERITY_NOTICE,
                            "AutoTune: path model retry at lower speed");
            return;
        }
        abort_run(reason, true);
        return;
    }

    apply_candidate_range(ParamIndex::PosP, ParamIndex::VelFiltD);
    _retry_count = 0U;
    set_stage(Stage::PositionValidate, "position validation");
}

void ModeAutoTune::update_position_validate()
{
    if (_phase == 0U) {
        start_path_experiment(false, false);
        if (_path_state == PathState::Idle) {
            abort_run(_experiment_failure_reason, true);
            return;
        }
        _candidate_position_perf.reset(_path_amplitude);
        _phase = 1U;
    }
    const ExperimentResult result = update_path_experiment(_candidate_position_perf);
    if (result == ExperimentResult::Failed) {
        abort_run(_experiment_failure_reason, _speed_feedback_reliable);
        return;
    }
    if (result != ExperimentResult::Complete) {
        return;
    }

    const bool acceptable =
        _candidate_position_perf.cost() <= _baseline_position_perf.cost() &&
        _candidate_position_perf.cost() <= maximum_model_nrmse &&
        _candidate_position_perf.saturation_fraction() <= maximum_saturation_fraction;
    if (!acceptable) {
        if (_retry_count == 0U) {
            _retry_count = 1U;
            set_candidate(ParamIndex::PosP,
                          0.85f * _managed[uint8_t(ParamIndex::PosP)].candidate,
                          ParamReason::PathFit);
            set_candidate(ParamIndex::VelP,
                          0.85f * _managed[uint8_t(ParamIndex::VelP)].candidate,
                          ParamReason::PathFit);
            set_candidate(ParamIndex::VelI,
                          0.75f * _managed[uint8_t(ParamIndex::VelI)].candidate,
                          ParamReason::PathFit);
            apply_candidate_range(ParamIndex::PosP, ParamIndex::VelI);
            _phase = 0U;
            gcs().send_text(MAV_SEVERITY_NOTICE,
                            "AutoTune: position gains reduced for retry");
            return;
        }
        abort_run(FailureReason::CandidateWorse, true);
        return;
    }

    _retry_count = 0U;
    set_stage(Stage::FinalVerify, "combined figure-eight validation");
}

void ModeAutoTune::update_final_verify()
{
    if (_phase == 0U) {
        start_path_experiment(true, false);
        if (_path_state == PathState::Idle) {
            abort_run(_experiment_failure_reason, true);
            return;
        }
        _final_perf.reset(MAX(_path_amplitude, 0.20f));
        _phase = 1U;
    }
    const ExperimentResult result = update_path_experiment(_final_perf);
    if (result == ExperimentResult::Failed) {
        abort_run(_experiment_failure_reason, _speed_feedback_reliable);
        return;
    }
    if (result != ExperimentResult::Complete) {
        return;
    }

    if (!evaluate_candidate()) {
        abort_run(FailureReason::CandidateWorse, true);
        return;
    }

    _failure_reason = FailureReason::None;
    _action_actuator_max = _effective_actuator_max;
    set_stage(Stage::Stopping, "final stop");
}

void ModeAutoTune::update_stopping()
{
    if (_speed_feedback_reliable) {
        command_stop();
        if (!vehicle_stopped()) {
            if (AP_HAL::millis() - _stage_start_ms < stopping_timeout_ms) {
                return;
            }
            command_zero();
            _speed_feedback_reliable = false;
            if (_failure_reason == FailureReason::None) {
                _failure_reason = FailureReason::Timeout;
            }
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "AutoTune: stop timeout, output zero");
        }
    } else {
        command_zero();
    }

    if (_failure_reason == FailureReason::None) {
        set_stage(Stage::Commit, "saving parameters");
        start_persistence(false);
        return;
    }

    if (_transaction_active) {
        restore_ram_baseline();
    }
    if (_candidate_persistence_started) {
        start_persistence(true);
        set_stage(Stage::Rollback, "restoring saved baseline");
    } else {
        finish_failure();
    }
}

void ModeAutoTune::update_commit()
{
    command_zero();
    if (_sample_due) {
        FailureReason reason;
        if (!runtime_safety_check(reason)) {
            _failure_reason = reason == FailureReason::None ?
                              FailureReason::Internal : reason;
            restore_ram_baseline();
            start_persistence(true);
            set_stage(Stage::Rollback, "commit interrupted, restoring baseline");
            return;
        }
    }
    if (_persistence_state == PersistenceState::CandidateDone) {
        if (_persistence_had_error) {
            _failure_reason = FailureReason::Persistence;
            set_stage(Stage::Rollback, "verifying baseline recovery");
        } else {
            finish_success();
        }
        return;
    }
    if (_persistence_state == PersistenceState::BaselineDone) {
        _failure_reason = FailureReason::Persistence;
        finish_failure();
        return;
    }
    if (_persistence_state == PersistenceState::Failed) {
        _failure_reason = FailureReason::Persistence;
        finish_failure();
    }
}

void ModeAutoTune::update_rollback()
{
    command_zero();
    if (_persistence_state == PersistenceState::BaselineDone) {
        finish_failure();
        return;
    }
    if (_persistence_state == PersistenceState::Failed) {
        _failure_reason = FailureReason::Persistence;
        finish_failure();
    }
}

void ModeAutoTune::update()
{
    const uint32_t now = AP_HAL::millis();
    _sample_due = now - _last_sample_ms >= sample_period_ms;
    if (_sample_due) {
        _last_sample_ms = now;
    }

    if (_stage == Stage::Standby) {
        if (!hal.util->get_soft_armed()) {
            command_zero();
            _ready_start_ms = 0U;
            return;
        }

        FailureReason reason;
        const bool static_ok = static_checks(reason);
        if (static_ok) {
            _effective_actuator_max = MIN(float(_actuator_max),
                                          g2.motors.get_throttle_max());
            _action_actuator_max = _effective_actuator_max;
            g2.motors.set_actuator_output_limit_pct(_effective_actuator_max);
        }
        if (!static_ok || !dynamic_checks(reason, true)) {
            if (reason == FailureReason::ControlLimits &&
                _speed_feedback_reliable) {
                command_stop();
            } else {
                command_zero();
            }
            _ready_start_ms = 0U;
            if (reason != _last_wait_reason ||
                now - _last_wait_message_ms >= 5000U) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "AutoTune: waiting %s",
                                failure_text(reason));
                _last_wait_reason = reason;
                _last_wait_message_ms = now;
            }
            return;
        }

        command_zero();

        if (_ready_start_ms == 0U) {
            _ready_start_ms = MAX(now, 1U);
            return;
        }
        if (now - _ready_start_ms >= readiness_time_ms) {
            begin_run();
        }
        return;
    }

    const bool safety_monitored =
        _stage != Stage::Stopping && _stage != Stage::Commit &&
        _stage != Stage::Rollback && _stage != Stage::Complete;
    if (safety_monitored) {
        FailureReason reason;
        if (!hal.util->get_soft_armed()) {
            abort_run(FailureReason::Disarmed, false);
        } else if (ahrs.has_recent_extnav_velocity(500U)) {
            abort_run(FailureReason::ExternalVelocity,
                      _speed_feedback_reliable);
        } else if (standard_failsafe_active()) {
            abort_run(FailureReason::Failsafe, _speed_feedback_reliable);
        } else if (_sample_due && !runtime_safety_check(reason)) {
            abort_run(reason, _speed_feedback_reliable);
        }
    } else if (_sample_due && _stage == Stage::Stopping) {
        if (_failure_reason == FailureReason::None) {
            FailureReason reason;
            if (!runtime_safety_check(reason)) {
                _failure_reason = reason == FailureReason::None ?
                                  FailureReason::Internal : reason;
                gcs().send_text(MAV_SEVERITY_WARNING,
                                "AutoTune: abort %s",
                                failure_text(_failure_reason));
            }
        } else {
            // Keep the latest speed estimate available for a controlled stop,
            // but never turn a failed estimate into a second abort path.
            if (!update_navigation_state()) {
                _speed_feedback_reliable = false;
            }
        }
    }

    switch (_stage) {
    case Stage::Standby:
        break;
    case Stage::BaselineRate:
        update_baseline_rate();
        break;
    case Stage::SteerIdentify:
        update_steer_identify();
        break;
    case Stage::SteerValidate:
        update_steer_validate();
        break;
    case Stage::AngleValidate:
        update_angle_validate();
        break;
    case Stage::SpeedBaseline:
        update_speed_baseline();
        break;
    case Stage::SpeedIdentify:
        update_speed_identify();
        break;
    case Stage::SpeedValidate:
        update_speed_validate();
        break;
    case Stage::PositionBaseline:
        update_position_baseline();
        break;
    case Stage::PositionValidate:
        update_position_validate();
        break;
    case Stage::FinalVerify:
        update_final_verify();
        break;
    case Stage::Stopping:
        update_stopping();
        break;
    case Stage::Commit:
        update_commit();
        break;
    case Stage::Rollback:
        update_rollback();
        break;
    case Stage::Complete:
        command_zero();
        break;
    }

    if (_sample_due && _stage != Stage::Standby) {
        write_sample_log();
    }
}
