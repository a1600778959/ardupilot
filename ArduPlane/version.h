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
#define THISFIRMWARE "ArduPlane V4.5.7"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,5,7,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 4
#define FW_MINOR 5
#define FW_PATCH 7
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL
=======
#define THISFIRMWARE "ArduPlane V4.4.0-beta1"
=======
#define THISFIRMWARE "ArduPlane V4.4.0-beta2"
>>>>>>> abb3bcfb1d (Plane: prepare for 4.4.0beta2)
=======
#define THISFIRMWARE "ArduPlane V4.4.0-beta3"
>>>>>>> 58d66d7a1e (Plane: prepare for 4.4.0beta3)
=======
#define THISFIRMWARE "ArduPlane V4.4.0-beta4"
>>>>>>> 0a3c50ab4b (Plane: prepare for 4.4.0-beta4)
=======
#define THISFIRMWARE "ArduPlane V4.4.0-beta5"
>>>>>>> bd0a91e2df (Plane: prepare for 4.4.0-beta5)
=======
#define THISFIRMWARE "ArduPlane V4.4.0"
>>>>>>> dc41723823 (Plane: prepare for 4.4.0)
=======
#define THISFIRMWARE "ArduPlane V4.4.1-beta1"
>>>>>>> d6973b2e4a (Plane: prepare for 4.4.1-beta1)

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,1,FIRMWARE_VERSION_TYPE_BETA

#define FW_MAJOR 4
#define FW_MINOR 4
<<<<<<< HEAD
#define FW_PATCH 0
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
#define FW_TYPE FIRMWARE_VERSION_TYPE_BETA
>>>>>>> 7fd85dd860 (Plane: prepare for 4.4.0beta1)
=======
#define FW_TYPE FIRMWARE_VERSION_TYPE_BETA+1
>>>>>>> abb3bcfb1d (Plane: prepare for 4.4.0beta2)
=======
#define FW_TYPE FIRMWARE_VERSION_TYPE_BETA+2
>>>>>>> 58d66d7a1e (Plane: prepare for 4.4.0beta3)
=======
#define FW_TYPE FIRMWARE_VERSION_TYPE_BETA+3
>>>>>>> 0a3c50ab4b (Plane: prepare for 4.4.0-beta4)
=======
#define FW_TYPE FIRMWARE_VERSION_TYPE_BETA+4
>>>>>>> bd0a91e2df (Plane: prepare for 4.4.0-beta5)
=======
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL
>>>>>>> dc41723823 (Plane: prepare for 4.4.0)
=======
#define FW_PATCH 1
#define FW_TYPE FIRMWARE_VERSION_TYPE_BETA
>>>>>>> d6973b2e4a (Plane: prepare for 4.4.1-beta1)

#include <AP_Common/AP_FWVersionDefine.h>
