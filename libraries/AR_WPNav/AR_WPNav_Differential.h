#pragma once

#include <math.h>
#include <stdint.h>

// Pure transition rules for differential-drive waypoint navigation.  Keeping
// these decisions free of AHRS and motor dependencies makes the MOVE-SPIN-MOVE
// invariants explicit and unit-testable.
class AR_WPNavDifferential
{
public:
    struct EndpointInput {
        bool path_terminal;
        float planned_speed_mps;
        float realtime_distance_m;
        float planned_distance_m;
        float capture_radius_m;
        float capture_speed_mps;
    };

    static bool endpoint_captured(const EndpointInput &input)
    {
        constexpr float planned_speed_epsilon_mps = 1.0e-4f;
        const bool low_speed = (input.capture_speed_mps > 0.0f) ?
            (fabsf(input.planned_speed_mps) <= input.capture_speed_mps) :
            (input.path_terminal && (fabsf(input.planned_speed_mps) <= planned_speed_epsilon_mps));
        return isfinite(input.planned_speed_mps) &&
               isfinite(input.realtime_distance_m) &&
               isfinite(input.planned_distance_m) &&
               isfinite(input.capture_radius_m) &&
               isfinite(input.capture_speed_mps) &&
               low_speed &&
               (input.realtime_distance_m >= 0.0f) &&
               (input.planned_distance_m >= 0.0f) &&
               (input.capture_radius_m > 0.0f) &&
               (input.realtime_distance_m <= input.capture_radius_m) &&
               (input.planned_distance_m <= input.capture_radius_m);
    }

    enum class MoveAction : uint8_t {
        Continue,
        BeginSpin,
        Fault,
    };

    static MoveAction resolve_move(bool endpoint_captured, bool pivot_activated)
    {
        if (!endpoint_captured) {
            return MoveAction::Continue;
        }
        return pivot_activated ? MoveAction::BeginSpin : MoveAction::Fault;
    }

    enum class SpinAction : uint8_t {
        Continue,
        Complete,
        Fault,
    };

    struct SpinOutput {
        SpinAction action;
        float turn_rate_rads;
    };

    static SpinOutput resolve_spin(bool controller_active_before_update,
                                   bool completion_edge,
                                   float requested_turn_rate_rads)
    {
        // Completion deactivates AR_PivotTurn, so its edge must take priority
        // over the inactive-controller fault rule.
        if (completion_edge) {
            return {SpinAction::Complete, 0.0f};
        }
        if (!controller_active_before_update || !isfinite(requested_turn_rate_rads)) {
            return {SpinAction::Fault, 0.0f};
        }
        return {SpinAction::Continue, requested_turn_rate_rads};
    }
};
