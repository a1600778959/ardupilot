#pragma once

#include <AP_Common/AP_Common.h>

// Global parameter class.
//
class Parameters {
public:
    enum {
        k_param_dummy,
        k_param_reserved_1,
        k_param_ins,
        k_param_ahrs,
        k_param_reserved_4,
        k_param_compass,
        k_param_logger,
        k_param_NavEKF3,
        k_param_gps,
    };
    AP_Int8 dummy;
};

extern const AP_Param::Info var_info[];
