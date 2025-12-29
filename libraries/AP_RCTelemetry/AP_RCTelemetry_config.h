#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Frsky_Telem/AP_Frsky_config.h>
#include <AP_RCProtocol/AP_RCProtocol_config.h>

#ifndef HAL_CRSF_TELEM_ENABLED
#define HAL_CRSF_TELEM_ENABLED AP_FRSKY_SPORT_PASSTHROUGH_ENABLED
#endif

#ifndef HAL_SPEKTRUM_TELEM_ENABLED
#define HAL_SPEKTRUM_TELEM_ENABLED 1
#endif

#ifndef AP_GHST_TELEM_ENABLED
#define AP_GHST_TELEM_ENABLED AP_RCPROTOCOL_GHST_ENABLED
#endif
