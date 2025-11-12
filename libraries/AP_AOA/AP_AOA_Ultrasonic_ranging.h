// File: libraries/AP_Modbus/AP_MultiDistanceSensor.h
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>

#define MODBUS_UART_NUM 4      // serial端口号(如serial4)
#define SENSOR_COUNT 2         // 最大传感器数量
#define PROCESSED_REG 0x0001   // 处理值寄存器地址
#define MODBUS_TIMEOUT_MS 100  // 单次通信超时
#define MODBUS_RESPONSE_SIZE 7 // 响应帧长度
#define MODBUS_REQUEST_SIZE 8  // 发送帧长度

class AP_MultiDistanceSensor
{
public:
    CLASS_NO_COPY(AP_MultiDistanceSensor);
    AP_MultiDistanceSensor();
    void init();
    void update();
    bool get_distance(uint8_t sensor_idx, float &dist) const;
    // void update_sensor_task();
    int get_min_distance();
    void send_request();
    void read_data(uint8_t *response);
    // static *get_singleton();
    static AP_MultiDistanceSensor *get_singleton() { return _singleton; }

private:
    struct SensorData
    {
        uint8_t address;      // Modbus设备地址
        float distance;       // 当前距离值(m)
        uint32_t last_update; // 最后更新时间
        bool valid;           // 数据有效性标志
    };
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver *_uart;
    // AP_MultiDistanceSensor(); // 单例模式构造函数
    bool is_init = false;
    uint8_t response[MODBUS_RESPONSE_SIZE] = {0};
    SensorData sensors[SENSOR_COUNT];
    int current_sensor_idx;
    static AP_MultiDistanceSensor *_singleton;
};

namespace AP {
    AP_MultiDistanceSensor *aoa();
};