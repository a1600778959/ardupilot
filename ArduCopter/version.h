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
#define THISFIRMWARE "ArduCopter V4.5.7"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,5,7,FIRMWARE_VERSION_TYPE_OFFICIAL
=======
#define THISFIRMWARE "ArduCopter V4.4.0-beta2"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+2
>>>>>>> d8a35e5dec (Copter: version to 4.4.0-beta2)
=======
#define THISFIRMWARE "ArduCopter V4.4.0-beta3"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+3
>>>>>>> 6fbb052c41 (Copter: version to 4.4.0-beta3)
=======
#define THISFIRMWARE "ArduCopter V4.4.0-beta4"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+4
>>>>>>> 9835ebb782 (Copter: version to 4.4.0-beta4)
=======
#define THISFIRMWARE "ArduCopter V4.4.0-beta5"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_BETA+5
>>>>>>> 1c5c94f9ed (Copter: version to 4.4.0-beta5)

#define FW_MAJOR 4
#define FW_MINOR 5
#define FW_PATCH 7
=======
#define THISFIRMWARE "ArduCopter V4.4.0"
=======
#define THISFIRMWARE "ArduCopter V4.4.1-beta1"
>>>>>>> fb8864e24f (Copter: version to 4.4.1-beta1)
=======
#define THISFIRMWARE "ArduCopter V4.4.1-beta2"
>>>>>>> f3d3a226e4 (Copter: version to 4.4.1-beta2)
=======
#define THISFIRMWARE "ArduCopter V4.4.1"
>>>>>>> 17070023cb (Copter: version to 4.4.1)

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,1,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 4
#define FW_MINOR 4
<<<<<<< HEAD
#define FW_PATCH 0
>>>>>>> d79487d5c8 (Copter: version to 4.4.0)
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL
=======
#define FW_PATCH 1
<<<<<<< HEAD
#define FW_TYPE FIRMWARE_VERSION_TYPE_BETA
>>>>>>> fb8864e24f (Copter: version to 4.4.1-beta1)
=======
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL
>>>>>>> 17070023cb (Copter: version to 4.4.1)

#include <AP_Common/AP_FWVersionDefine.h>
