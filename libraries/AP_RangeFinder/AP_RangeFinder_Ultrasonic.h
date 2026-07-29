#pragma once

#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_RangeFinder_Backend_Serial.h"

#define SENSOR_COUNT 2         // 最大传感器数量
#define PROCESSED_REG 0x0001   // 处理值寄存器地址
#define MODBUS_RESPONSE_SIZE 7 // 响应帧长度
#define MODBUS_REQUEST_SIZE 8  // 发送帧长度

struct SensorData {
    uint8_t address;      // Modbus设备地址
    float distance;       // 当前距离值(m)
    bool valid;           // 数据有效性标志
};

class AP_RangeFinder_Ultrasonic : public AP_RangeFinder_Backend_Serial {
public:
    static AP_RangeFinder_Backend_Serial* create(
        RangeFinder::RangeFinder_State& _state,
        AP_RangeFinder_Params& _params) {
        return NEW_NOTHROW AP_RangeFinder_Ultrasonic(_state, _params);
    }

protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    AP_RangeFinder_Ultrasonic(RangeFinder::RangeFinder_State& _state, AP_RangeFinder_Params& _params);
    uint16_t read_timeout_ms() const override {
        return 500;
    }

    bool get_reading(float& reading_m) override;
    void send_request();
    void read_data(uint8_t* response);
    float get_min_distance();

    uint8_t response[MODBUS_RESPONSE_SIZE] = {0};
    SensorData sensors[SENSOR_COUNT];
    int current_sensor_idx;
};
