#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
#define THISFIRMWARE "ArduRover V4.4.0-beta4"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+4
>>>>>>> 7d6895a0f4 (Rover: version to 4.4.0-beta4)
=======
#define THISFIRMWARE "ArduRover V4.4.0-beta5"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+5
>>>>>>> 4802091ae0 (Rover: version to 4.4.0-beta5)
=======
#define THISFIRMWARE "ArduRover V4.4.0-beta6"
=======
#define THISFIRMWARE "ArduRover V4.4.0-beta7"
>>>>>>> 107102e952 (Rover: version to 4.4.0-beta7)

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+6
>>>>>>> 3683e92af2 (Rover: version to 4.4.0-beta6)
=======
#define THISFIRMWARE "ArduRover V4.4.0-beta8"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+7
>>>>>>> cb547f02fc (Rover: version to 4.4.0-beta8)
=======
#define THISFIRMWARE "ArduRover V4.4.0-beta9"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+8
>>>>>>> 48cdde264c (Rover: version to 4.4.0-beta9)
=======
#define THISFIRMWARE "ArduRover V4.4.0-beta10"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+9
>>>>>>> 17a04a81ea (Rover: version to 4.4.0-beta10)
=======
#define THISFIRMWARE "ArduRover V4.4.0-beta11"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+10
>>>>>>> 5b388f5687 (Rover: version to 4.4.0-beta11)

#define FW_MAJOR 4
#define FW_MINOR 5
#define FW_PATCH 7
=======
#define THISFIRMWARE "ArduRover V4.4.0"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 4
#define FW_MINOR 4
#define FW_PATCH 0
>>>>>>> f5e7cc4e86 (Rover: version to 4.4.0)
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL

#include <AP_Common/AP_FWVersionDefine.h>
