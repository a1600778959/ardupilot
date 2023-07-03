#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

<<<<<<< HEAD
<<<<<<< HEAD
#define THISFIRMWARE "ArduRover V4.5.7"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,5,7,FIRMWARE_VERSION_TYPE_OFFICIAL
=======
#define THISFIRMWARE "ArduRover V4.4.0-beta2"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+2
>>>>>>> a8f2a6d14b (Rover: version to 4.4.0-beta2)
=======
#define THISFIRMWARE "ArduRover V4.4.0-beta3"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+3
>>>>>>> 0c06a383b9 (Rover: version to 4.4.0-beta3)

#define FW_MAJOR 4
#define FW_MINOR 5
#define FW_PATCH 7
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL

#include <AP_Common/AP_FWVersionDefine.h>
