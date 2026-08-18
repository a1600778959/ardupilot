#pragma once

#include "mode.h"

class ModeAutoTune : public Mode
{
public:
    ModeAutoTune();

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name4() const override { return "ATUN"; }
    void update() override;
    bool is_autopilot_mode() const override { return true; }
    bool allows_arming_from_transmitter() override { return true; }

    // Parameter persistence is deliberately serviced outside the mode update.
    // This lets an operator take over immediately while a partially-started
    // candidate EEPROM transaction is being restored to its baseline.
    void background_save_update();

    static const struct AP_Param::GroupInfo var_info[];

    enum class Stage : uint8_t {
        Standby = 0,
        BaselineRate,
        SteerIdentify,
        SteerValidate,
        AngleValidate,
        SpeedBaseline,
        SpeedIdentify,
        SpeedValidate,
        PositionBaseline,
        PositionValidate,
        FinalVerify,
        Stopping,
        Commit,
        Rollback,
        Complete,
    };

    enum class FailureReason : uint8_t {
        None = 0,
        Configuration,
        VehicleType,
        ControlLimits,
        Estimator,
        GPS,
        ExternalVelocity,
        Failsafe,
        Boundary,
        Saturation,
        ModelData,
        ModelUnstable,
        ModelFit,
        Overshoot,
        CandidateWorse,
        Persistence,
        Disarmed,
        Timeout,
        Internal,
        Operator,
    };

    struct LogSample {
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

    struct LogModel {
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

    struct LogParam {
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

protected:
    bool _enter() override;
    void _exit() override;

private:
    static constexpr uint8_t managed_param_count = 33;
    static constexpr float model_sample_dt = 0.02f;

    enum class ParamIndex : uint8_t {
        CruiseSpeed = 0,
        CruiseThrottle,
        SteerFF,
        SteerP,
        SteerI,
        SteerD,
        SteerDFF,
        SteerIMax,
        SteerFiltT,
        SteerFiltE,
        SteerFiltD,
        SteerSMax,
        SteerPDMax,
        SpeedFF,
        SpeedP,
        SpeedI,
        SpeedD,
        SpeedDFF,
        SpeedIMax,
        SpeedFiltT,
        SpeedFiltE,
        SpeedFiltD,
        SpeedSMax,
        SpeedPDMax,
        SteerAngleP,
        PosP,
        VelP,
        VelI,
        VelD,
        VelFF,
        VelIMax,
        VelFiltE,
        VelFiltD,
    };

    enum class ParamReason : uint8_t {
        Baseline = 0,
        Identified,
        ConservativePI,
        CruiseMap,
        PathFit,
        ModelUnsupported,
        Retained,
        Rollback,
    };

    enum class ParamAction : uint8_t {
        Baseline = 0,
        Candidate,
        Save,
        Verify,
        Rollback,
    };

    struct ManagedParam {
        AP_Param *param{nullptr};
        ap_var_type type{AP_PARAM_NONE};
        float baseline{0.0f};
        float candidate{0.0f};
        ParamReason reason{ParamReason::Baseline};
        bool candidate_applied{false};
    };

    struct RLSModel {
        float theta[4]{};
        float covariance[4][4]{};
        float y_hist[2]{};
        float u_hist[2]{};
        float squared_error{0.0f};
        float output_min{0.0f};
        float output_max{0.0f};
        uint16_t samples{0};
        uint16_t saturated_samples{0};
        bool history_valid{false};

        void reset();
        bool update(float input, float output, bool saturated);
        float nrmse() const;
        float saturation_fraction() const;
        bool stable(float &largest_pole) const;
        bool plant(float &gain, float &time_constant) const;
        bool qualified(FailureReason &reason) const;
    };

    struct Performance {
        float squared_error{0.0f};
        float scale{1.0f};
        float peak{0.0f};
        float step_target{0.0f};
        float step_start_actual{0.0f};
        float maximum_overshoot{0.0f};
        uint16_t samples{0};
        uint16_t saturated_samples{0};

        void reset(float normalising_scale);
        void update(float target, float actual, bool saturated);
        float cost() const;
        float saturation_fraction() const;
    };

    enum class StraightKind : uint8_t {
        Baseline,
        Identify,
        Validate,
    };

    enum class StraightState : uint8_t {
        Idle,
        Align,
        Run,
        StopForTurn,
        Turn,
        FinishStop,
    };

    enum class ExperimentResult : uint8_t {
        Running,
        Complete,
        Failed,
    };

    enum class PathState : uint8_t {
        Idle,
        MoveToStart,
        Run,
        FinishStop,
    };

    enum class PersistenceState : uint8_t {
        Idle,
        SaveCandidate,
        VerifyCandidate,
        SaveBaseline,
        VerifyBaseline,
        CandidateDone,
        BaselineDone,
        Failed,
    };

    // RTA_ parameters
    AP_Int8 _enable;
    AP_Float _area_length;
    AP_Float _area_width;
    AP_Float _clearance;
    AP_Float _actuator_max;

    ManagedParam _managed[managed_param_count]{};
    RLSModel _steer_model;
    RLSModel _speed_model;
    RLSModel _position_model;
    Performance _baseline_rate_perf;
    Performance _candidate_rate_perf;
    Performance _baseline_speed_perf;
    Performance _candidate_speed_perf;
    Performance _baseline_position_perf;
    Performance _candidate_position_perf;
    Performance _final_perf;

    Stage _stage{Stage::Standby};
    FailureReason _failure_reason{FailureReason::None};
    uint8_t _phase{0};
    uint8_t _retry_count{0};
    uint16_t _run_id{0};
    uint32_t _stage_start_ms{0};
    uint32_t _run_start_ms{0};
    uint32_t _ready_start_ms{0};
    uint32_t _stopped_start_ms{0};
    uint32_t _last_sample_ms{0};
    uint32_t _last_wait_message_ms{0};
    FailureReason _last_wait_reason{FailureReason::None};

    bool _transaction_active{false};
    bool _transaction_committed{false};
    bool _candidate_persistence_started{false};
    bool _steer_candidate_ready{false};
    bool _manual_exit{false};
    bool _speed_feedback_reliable{false};
    bool _command_saturated{false};
    bool _action_limited{false};
    bool _sample_due{false};
    uint32_t _saturation_start_ms{0};

    Location _center_location;
    Vector2f _center_ne;
    float _field_yaw_rad{0.0f};
    float _position_uncertainty{0.0f};
    float _x_limit{0.0f};
    float _y_limit{0.0f};
    float _field_x{0.0f};
    float _field_y{0.0f};
    float _field_vx{0.0f};
    float _field_vy{0.0f};
    float _margin_x{0.0f};
    float _margin_y{0.0f};
    float _speed{0.0f}; // controller-equivalent forward speed
    float _ground_speed{0.0f};
    float _yaw_rate{0.0f};
    float _run_area_length{0.0f};
    float _run_area_width{0.0f};
    float _run_clearance{0.0f};
    float _run_actuator_max{0.0f};
    float _effective_actuator_max{0.0f};
    float _action_actuator_max{0.0f};
    float _steer_action_max{0.0f};
    float _crawl_speed{0.0f};
    float _selected_speed{0.0f};
    float _baseline_speed_target{0.0f};
    float _rate_target{0.0f};
    float _last_target{0.0f};
    float _last_actual{0.0f};
    float _last_output{0.0f};

    StraightState _straight_state{StraightState::Idle};
    StraightKind _straight_kind{StraightKind::Baseline};
    int8_t _travel_direction{1};
    uint8_t _straight_folds{0};
    uint16_t _straight_samples{0};
    uint32_t _straight_start_ms{0};
    uint32_t _straight_state_start_ms{0};
    uint32_t _straight_run_ms{0};
    float _straight_target{0.0f};
    float _identify_speed_limit{0.0f};
    float _raw_amplitude_pct{0.0f};
    float _raw_command_pct{0.0f};
    float _steady_speed_sum{0.0f};
    float _steady_throttle_sum{0.0f};
    uint16_t _steady_samples{0};
    float _previous_speed{0.0f};
    float _decel_mean{0.0f};
    float _decel_m2{0.0f};
    uint16_t _decel_samples{0};

    PathState _path_state{PathState::Idle};
    bool _path_figure_eight{false};
    bool _path_collect_model{false};
    uint32_t _path_start_ms{0};
    uint32_t _path_state_start_ms{0};
    uint32_t _path_duration_ms{0};
    float _path_half_length{0.0f};
    float _path_amplitude{0.0f};
    float _path_speed{0.0f};
    float _path_squared_error{0.0f};
    uint16_t _path_samples{0};
    uint16_t _path_saturated_samples{0};
    Vector2f _path_target_field;
    bool _path_reference_valid{false};
    float _path_reference_half_length{0.0f};
    float _path_reference_amplitude{0.0f};
    float _path_reference_speed{0.0f};

    float _angle_base_yaw{0.0f};
    float _angle_step_start_yaw{0.0f};
    float _angle_target_yaw{0.0f};
    float _angle_peak{0.0f};
    float _angle_overshoot{0.0f};
    uint32_t _angle_settled_ms{0};

    PersistenceState _persistence_state{PersistenceState::Idle};
    uint8_t _persistence_index{0};
    bool _persistence_had_error{false};
    bool _persistence_failure_reported{false};
    FailureReason _experiment_failure_reason{FailureReason::None};

    AP_Param *_quicktune_enable_param{nullptr};
    ap_var_type _quicktune_enable_type{AP_PARAM_NONE};

    bool static_checks(FailureReason &reason) const;
    bool dynamic_checks(FailureReason &reason, bool require_stopped);
    bool update_navigation_state();
    bool update_position_uncertainty(float &uncertainty) const;
    bool runtime_safety_check(FailureReason &reason);
    bool output_limited_for_sample() const {
        return _command_saturated || _action_limited;
    }
    bool quicktune_active() const;
    bool standard_failsafe_active() const;
    bool vehicle_stopped();
    bool heading_reached(float target_yaw);
    void reset_stop_confirmation();

    bool initialise_managed_params();
    void set_candidate(ParamIndex index, float value, ParamReason reason);
    void apply_candidate_range(ParamIndex first, ParamIndex last);
    void restore_ram_baseline();
    void reset_controllers();
    void start_persistence(bool baseline);
    void log_param(uint8_t index, ParamAction action, ParamReason reason,
                   uint8_t result, float readback) const;

    void begin_run();
    void set_stage(Stage stage, const char *message);
    void abort_run(FailureReason reason, bool speed_feedback_reliable);
    void finish_failure();
    void finish_success();

    void update_baseline_rate();
    void update_steer_identify();
    void update_steer_validate();
    void update_angle_validate();
    void update_speed_baseline();
    void update_speed_identify();
    void update_speed_validate();
    void update_position_baseline();
    void update_position_validate();
    void update_final_verify();
    void update_stopping();
    void update_commit();
    void update_rollback();

    bool derive_steering_candidate();
    bool derive_speed_candidate();
    bool derive_position_candidate();
    bool evaluate_candidate() const;

    void start_straight_experiment(StraightKind kind, float target);
    ExperimentResult update_straight_experiment();
    void start_path_experiment(bool figure_eight, bool collect_model);
    ExperimentResult update_path_experiment(Performance &performance);

    void command_mixed(float throttle_pct, float steering_normalised);
    float limit_turn_rate_target(float target_rate_rads) const;
    void command_rate_speed(float turn_rate_rads, float target_speed);
    void command_heading_speed(float heading_rad, float target_speed);
    void command_position_target(const Vector2p &position,
                                 const Vector2f &velocity,
                                 const Vector2f &acceleration);
    bool command_stop();
    void command_zero();

    Vector2f field_to_ne(float x, float y) const;
    Vector2f ne_to_field(const Vector2f &ne) const;
    float heading_for_direction(int8_t direction) const;
    float configured_deceleration() const;
    float conservative_deceleration() const;
    float stopping_distance(float speed) const;
    float straight_remaining_distance(int8_t direction) const;
    bool straight_needs_turn(int8_t direction) const;
    float space_limited_speed() const;
    void update_deceleration_estimate(float command_pct);

    void write_sample_log() const;
    void write_model_log(const RLSModel &model, uint8_t event,
                         FailureReason reason, float overshoot,
                         float gain, float time_constant) const;
    static const char *failure_text(FailureReason reason);
};
