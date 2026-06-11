#include "LR_MsgHandler.h"
#include "LogReader.h"
#include "Replay.h"

#include <AP_DAL/AP_DAL.h>

#include <cinttypes>

extern const AP_HAL::HAL& hal;

#define MSG_CREATE(sname,msgbytes) log_ ##sname msg; memcpy((void*)&msg, (msgbytes)+3, sizeof(msg));

LR_MsgHandler::LR_MsgHandler(struct log_Format &_f) :
    MsgHandler(_f) {
}

void LR_MsgHandler_RFRH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RFRH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RFRF::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RFRF, msgbytes);
    AP::dal().handle_message(msg, ekf3);
}

void LR_MsgHandler_RFRN::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RFRN, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_REV3::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REV3, msgbytes);

    switch ((AP_DAL::Event)msg.event) {

    case AP_DAL::Event::resetGyroBias:
        ekf3.resetGyroBias();
        break;
    case AP_DAL::Event::resetHeightDatum:
        ekf3.resetHeightDatum();
        break;
    case AP_DAL::Event::setTerrainHgtStable:
        ekf3.setTerrainHgtStable(true);
        break;
    case AP_DAL::Event::unsetTerrainHgtStable:
        ekf3.setTerrainHgtStable(false);
        break;
    case AP_DAL::Event::requestYawReset:
        ekf3.requestYawReset();
        break;
    case AP_DAL::Event::checkLaneSwitch:
        ekf3.checkLaneSwitch();
        break;
    case AP_DAL::Event::setSourceSet0 ... AP_DAL::Event::setSourceSet2:
        ekf3.setPosVelYawSourceSet(uint8_t(msg.event)-uint8_t(AP_DAL::Event::setSourceSet0));
        break;
    }

}

void LR_MsgHandler_RSO3::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RSO3, msgbytes);
    Location loc;
    loc.lat = msg.lat;
    loc.lng = msg.lng;
    loc.alt = msg.alt;
    ekf3.setOriginLLH(loc);
}

void LR_MsgHandler_RWA3::process_message(uint8_t *msgbytes)
{
    (void)msgbytes;
}

void LR_MsgHandler_REY3::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REY3, msgbytes);
    ekf3.writeEulerYawAngle(msg.yawangle, msg.yawangleerr, msg.timestamp_ms, msg.type);
}

void LR_MsgHandler_RISH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RISH, msgbytes);
    AP::dal().handle_message(msg);
}
void LR_MsgHandler_RISI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RISI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RASH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RASH, msgbytes);
    AP::dal().handle_message(msg);
}
void LR_MsgHandler_RASI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RASI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RRNH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RRNH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RRNI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RRNI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RGPH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RGPH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RGPI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RGPI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RGPJ::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RGPJ, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RMGH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RMGH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RMGI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RMGI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RBCH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBCH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RBCI::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBCI, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_RVOH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RVOH, msgbytes);
    AP::dal().handle_message(msg);
}

void LR_MsgHandler_ROFH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(ROFH, msgbytes);
    AP::dal().handle_message(msg, ekf3);
}

void LR_MsgHandler_RWOH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RWOH, msgbytes);
    AP::dal().handle_message(msg, ekf3);
}

void LR_MsgHandler_RBOH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RBOH, msgbytes);
    AP::dal().handle_message(msg, ekf3);
}

void LR_MsgHandler_REPH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REPH, msgbytes);
    AP::dal().handle_message(msg, ekf3);
}

void LR_MsgHandler_RSLL::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(RSLL, msgbytes);
    AP::dal().handle_message(msg, ekf3);
}

void LR_MsgHandler_REVH::process_message(uint8_t *msgbytes)
{
    MSG_CREATE(REVH, msgbytes);
    AP::dal().handle_message(msg, ekf3);
}

#include <AP_AHRS/AP_AHRS.h>
#include "VehicleType.h"

bool LR_MsgHandler_PARM::set_parameter(const char *name, const float value)
{
    const char *ignore_parms[] = {
        "LOG_FILE_BUFSIZE",
        "LOG_DISARMED"
    };
    for (uint8_t i=0; i < ARRAY_SIZE(ignore_parms); i++) {
        if (strncmp(name, ignore_parms[i], AP_MAX_NAME_SIZE) == 0) {
            ::printf("Ignoring set of %s to %f\n", name, value);
            return true;
        }
    }

    return LogReader::set_parameter(name, value);
}

void LR_MsgHandler_PARM::process_message(uint8_t *msg)
{
    const uint8_t parameter_name_len = AP_MAX_NAME_SIZE + 1; // null-term
    char parameter_name[parameter_name_len];

    require_field(msg, "Name", parameter_name, parameter_name_len);

    float value = require_field_float(msg, "Value");
    set_parameter(parameter_name, value);
}
