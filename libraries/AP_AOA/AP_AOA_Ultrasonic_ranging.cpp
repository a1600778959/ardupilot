// File: libraries/AP_Modbus/AP_MultiDistanceSensor.cpp
#include "AP_AOA_Ultrasonic_ranging.h"

// singleton instance
AP_MultiDistanceSensor *AP_MultiDistanceSensor::_singleton;

AP_MultiDistanceSensor::AP_MultiDistanceSensor()
{
    current_sensor_idx = -1;
    if (_singleton != nullptr) {
        AP_HAL::panic("AOA must be singleton");
        return;
    }
    _singleton = this;
}

void AP_MultiDistanceSensor::init()
{
    // 初始化传感器地址(0x01~0x06)
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        sensors[i].address = 0x01 + i;
        sensors[i].distance = 6666.0f;
        sensors[i].last_update = 0;
        sensors[i].valid = false;
    }

    // 配置UART参数
    _uart = hal.serial(MODBUS_UART_NUM);
    if (_uart == nullptr)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "485 uart is not config");
        return;
    }

    _uart->set_stop_bits(1);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    // hal.serial(MODBUS_UART_NUM)->set_unbuffered_writes(true);
    //_uart->begin(115200);
    hal.scheduler->delay(100); // 等待初始化串口
    // hal.serial(MODBUS_UART_NUM)->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    // hal.serial(MODBUS_UART_NUM)->set_stop_bits(1);
}

void AP_MultiDistanceSensor::send_request()
{
    // 构造请求帧
    if (current_sensor_idx < SENSOR_COUNT - 1)
    {
        current_sensor_idx++;
    }
    else
    {
        current_sensor_idx = 0;
    }
    uint8_t request[MODBUS_REQUEST_SIZE] = {
        sensors[current_sensor_idx].address,
        0x03,
        (PROCESSED_REG >> 8),
        (PROCESSED_REG & 0xFF),
        0x00, 0x01,
        0x00, 0x00};
    // 添加CRC
    uint16_t crc = calc_crc_modbus(request, sizeof(request) - 2);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "start collect:%d", current_sensor_idx);
    // 发送请求
    _uart->write(request, 8);

}

void AP_MultiDistanceSensor::read_data(uint8_t *rep)
{
    for (int i = 0; i < MODBUS_RESPONSE_SIZE; i++)
    {
        rep[i] = _uart->read();
    }
    if (rep[1] == 0x03 && rep[2] == 0x02)
    {
        uint32_t sensor_idx = rep[0] - 1;
        if (calc_crc_modbus(rep, 5) == ((rep[6] << 8) | rep[5]))
        {
            uint16_t raw_val = (rep[3] << 8) | rep[4];
            sensors[sensor_idx].distance = raw_val;
            sensors[sensor_idx].valid = true;
            sensors[sensor_idx].last_update = AP_HAL::millis();
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "S%d: %.2fm OK",
            //               rep[0], sensors[sensor_idx].distance * 0.001f);
        }
        else
        {
            sensors[sensor_idx].valid = false;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "S%d CRC Error",
                          rep[0]);
        }
    }
}

void AP_MultiDistanceSensor::update()
{
    // 获取传感器的处理值时, 需要大于等于100ms的间隔
    // 这个间隔时间太大, 对于cpu来讲，不可能在这里一直等待返回
    // 因此每次轮询应该为先读取上一次发送请求的数据, 在发送读的命令
    if (!is_init)
    {
        is_init = true;
        init();
        send_request();
        return;
    }
    uint8_t rec_num = _uart->available();
    if (rec_num > 0)
    {
        read_data(response);
    }
    else
    {
        sensors[current_sensor_idx].valid = false;
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AOA data is invalid");
    }
    // 发送请求
    send_request();
    gcs().send_named_float("aoa_dist", get_min_distance()->distance);
}

bool AP_MultiDistanceSensor::get_distance(uint8_t sensor_idx, float &dist) const
{
    if (sensor_idx >= SENSOR_COUNT)
        return false;
    dist = sensors[sensor_idx].distance;
    return sensors[sensor_idx].valid;
}

// 返回有效的超声波检测的距离最小值
SensorData *AP_MultiDistanceSensor::get_min_distance(){
    SensorData *min = nullptr;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (min == nullptr)
        {
            min = &sensors[i];
            continue;
        }
        if (sensors[i].distance < min->distance && sensors[i].valid)
        {
            min = &sensors[i];
        }
    }
    return min;
}

namespace AP {
    AP_MultiDistanceSensor *distance_sensor()
    {
        return AP_MultiDistanceSensor::get_singleton();
    }
}

// 任务调度注册
// static void update_sensor_task()
// {
//     AP_MultiDistanceSensor::instance().update();
// }

// AP_SCHEDULER_TASK(update_sensor_task, 50, 100);