#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>

#define MODBUS_UART_NUM 4      // serial端口号(如serial4)
#define SENSOR_COUNT 2         // 最大传感器数量
#define PROCESSED_REG 0x0001   // 处理值寄存器地址
#define MODBUS_RESPONSE_SIZE 7 // 响应帧长度
#define MODBUS_REQUEST_SIZE 8  // 发送帧长度

struct SensorData
{
    uint8_t address;      // Modbus设备地址
    float distance;       // 当前距离值(m)
    bool valid;           // 数据有效性标志
};
class AP_MultiDistanceSensor
{
public:
    AP_MultiDistanceSensor();
    void init();
    void update();
    bool get_distance(uint8_t sensor_idx, float &dist) const;
    SensorData *get_min_distance();
    void send_request();
    void read_data(uint8_t *response);
    static AP_MultiDistanceSensor *get_singleton() {
        return _singleton;
    }
private:
    static AP_MultiDistanceSensor *_singleton;
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();
    AP_HAL::UARTDriver *_uart;
    bool is_init = false;
    uint8_t response[MODBUS_RESPONSE_SIZE] = {0};
    SensorData sensors[SENSOR_COUNT];
    int current_sensor_idx;
};
 namespace AP {
    AP_MultiDistanceSensor *distance_sensor();
};
