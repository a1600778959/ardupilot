#pragma once

#include <math.h>
#include <stdint.h>

class AR_PivotTurnCompletion
{
public:
    bool update(float heading_error_deg,
                float yaw_rate_deg_s,
                uint32_t now_ms,
                uint32_t delay_ms)
    {
        if (!isfinite(heading_error_deg) ||
            !isfinite(yaw_rate_deg_s) ||
            (fabsf(heading_error_deg) >= heading_accuracy_deg) ||
            (fabsf(yaw_rate_deg_s) >= yaw_rate_accuracy_deg_s)) {
            reset();
            return false;
        }

        if (!_timer_running) {
            _timer_running = true;
            _delay_start_ms = now_ms;
        }

        return (now_ms - _delay_start_ms) >= delay_ms;
    }

    void reset()
    {
        _timer_running = false;
        _delay_start_ms = 0U;
    }

private:
    static constexpr float heading_accuracy_deg = 5.0f;
    static constexpr float yaw_rate_accuracy_deg_s = 5.0f;

    bool _timer_running{false};
    uint32_t _delay_start_ms{0U};
};
