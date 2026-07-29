#pragma once

#include <stdint.h>

class ModePatrolRoute
{
public:
    enum class LegType : uint8_t {
        None,
        InitialTransit,
        WorkLine,
        Transition,
    };

    struct Target {
        Target() = default;
        Target(uint16_t line_index_in, LegType leg_type_in) :
            line_index(line_index_in),
            leg_type(leg_type_in)
        {}

        bool starts_new_line() const
        {
            return (leg_type == LegType::InitialTransit) ||
                   (leg_type == LegType::Transition);
        }

        uint16_t line_index{0U};
        LegType leg_type{LegType::None};
    };

    static Target start_target()
    {
        return {1, LegType::InitialTransit};
    }

    static bool target_after(const Target &current, Target &next)
    {
        if (current.line_index == 0) {
            return false;
        }

        if ((current.leg_type == LegType::InitialTransit) ||
            (current.leg_type == LegType::Transition)) {
            next = {current.line_index, LegType::WorkLine};
            return true;
        }

        if ((current.leg_type != LegType::WorkLine) ||
            (current.line_index == UINT16_MAX)) {
            return false;
        }

        next = {static_cast<uint16_t>(current.line_index + 1U),
                LegType::Transition};
        return true;
    }

};
