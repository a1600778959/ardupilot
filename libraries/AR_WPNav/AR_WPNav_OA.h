#pragma once

#include "AR_WPNav.h"

class AR_WPNav_OA : public AR_WPNav {
public:

    // re-use parent's constructor
    using AR_WPNav::AR_WPNav;

    // update navigation
    void update(float dt) override;

    // set desired location and (optionally) next_destination
    // next_destination should be provided if known to allow smooth cornering
    bool set_desired_location(const Location &destination, Location next_destination = Location()) override WARN_IF_UNUSED;

    bool set_desired_location_stopping_from_origin(
        const Location &origin,
        const Location &destination,
        float speed_max_mps = 0.0f) override WARN_IF_UNUSED;
    void cancel_stopping_line() override;

    // true if vehicle has reached desired location. defaults to true because this is normally used by missions and we do not want the mission to become stuck
    bool reached_destination() const override;

    // get object avoidance adjusted origin. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    const Location &get_oa_origin() const override;

    // get object avoidance adjusted destination. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    const Location &get_oa_destination() const override;

private:

    // object avoidance variables
    bool _oa_active;                // true if we should use alternative destination to avoid obstacles
    bool _stopping_line_oa_bypass{false};
    Location _origin_oabak;         // backup of _origin so it can be restored when oa completes
    Location _destination_oabak;    // backup of _desitnation so it can be restored when oa completes
    Location _next_destination_oabak; // backup of _next_destination so it can be restored when oa completes
    Location _oa_origin;            // intermediate origin during avoidance
    Location _oa_destination;       // intermediate destination during avoidance
};
