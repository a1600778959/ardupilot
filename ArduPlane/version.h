#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

<<<<<<< HEAD
#define THISFIRMWARE "ArduPlane V4.5.7"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,5,7,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 4
#define FW_MINOR 5
#define FW_PATCH 7
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL
=======
#define THISFIRMWARE "ArduPlane V4.4.0-beta1"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA

#define FW_MAJOR 4
#define FW_MINOR 4
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_BETA
>>>>>>> 7fd85dd860 (Plane: prepare for 4.4.0beta1)

#include <AP_Common/AP_FWVersionDefine.h>
