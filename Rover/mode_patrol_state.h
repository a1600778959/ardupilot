#pragma once

#include <stdint.h>

class ModePatrolRoute
{
public:
    enum class State : uint8_t {
        WaitingForPoints,
        Navigating,
        Fault,
    };

    enum class Phase : uint8_t {
        LineStart,
        LineEnd,
    };

    struct Target {
        uint16_t line_index;
        Phase phase;
        bool starts_new_line;
    };

    static Target start_target()
    {
        return {1, Phase::LineStart, true};
    }

    bool resume_target(Target &target) const
    {
        if (!navigating() || (_line_index == 0)) {
            return false;
        }

        target = {_line_index, _phase, false};
        return true;
    }

    bool next_target(Target &target) const
    {
        if (!navigating() || (_line_index == 0)) {
            return false;
        }

        if (_phase == Phase::LineStart) {
            target = {_line_index, Phase::LineEnd, false};
            return true;
        }

        if (_line_index == UINT16_MAX) {
            return false;
        }

        target = {static_cast<uint16_t>(_line_index + 1U), Phase::LineStart, true};
        return true;
    }

    void commit(const Target &target)
    {
        _line_index = target.line_index;
        _phase = target.phase;
        _state = State::Navigating;
    }

    void reset()
    {
        _state = State::WaitingForPoints;
        _phase = Phase::LineStart;
        _line_index = 0;
    }

    bool set_fault()
    {
        const bool new_fault = _state != State::Fault;
        _state = State::Fault;
        return new_fault;
    }

    bool waiting_for_points() const { return _state == State::WaitingForPoints; }
    bool navigating() const { return _state == State::Navigating; }
    bool in_fault() const { return _state == State::Fault; }
    Phase phase() const { return _phase; }
    uint16_t line_index() const { return _line_index; }

private:
    State _state{State::WaitingForPoints};
    Phase _phase{Phase::LineStart};
    uint16_t _line_index{0};
};

class ModePatrolPivotWatchdog
{
public:
    bool update(bool monitor, float heading_error_deg, uint32_t now_ms, uint32_t timeout_ms)
    {
        if (!monitor || (timeout_ms == 0)) {
            reset();
            return false;
        }

        if (!_monitoring) {
            _monitoring = true;
            _best_heading_error_deg = heading_error_deg;
            _last_progress_ms = now_ms;
            return false;
        }

        if (heading_error_deg < (_best_heading_error_deg - progress_threshold_deg)) {
            _best_heading_error_deg = heading_error_deg;
            _last_progress_ms = now_ms;
            return false;
        }

        return (now_ms - _last_progress_ms) >= timeout_ms;
    }

    void reset()
    {
        _monitoring = false;
        _best_heading_error_deg = 0.0f;
        _last_progress_ms = 0;
    }

private:
    static constexpr float progress_threshold_deg = 0.5f;

    bool _monitoring{false};
    float _best_heading_error_deg{0.0f};
    uint32_t _last_progress_ms{0};
};
