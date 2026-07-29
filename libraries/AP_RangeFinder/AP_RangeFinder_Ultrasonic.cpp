#include "AP_RangeFinder_Ultrasonic.h"
#include <cmath>

AP_RangeFinder_Ultrasonic::AP_RangeFinder_Ultrasonic(
    RangeFinder::RangeFinder_State& _state, AP_RangeFinder_Params& _params)
    : AP_RangeFinder_Backend_Serial(_state, _params) {
    params.scaling.set_default(0);
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        sensors[i].address = 0x01 + i;
        sensors[i].distance = 6666.0f;
        sensors[i].valid = false;
    }
}

void AP_RangeFinder_Ultrasonic::send_request() {
    // 构造请求帧
    if (current_sensor_idx < SENSOR_COUNT - 1) {
        current_sensor_idx++;
    } else {
        current_sensor_idx = 0;
    }
    uint8_t request[MODBUS_REQUEST_SIZE] = {sensors[current_sensor_idx].address,
                                            0x03,
                                            (PROCESSED_REG >> 8),
                                            (PROCESSED_REG & 0xFF),
                                            0x00,
                                            0x01,
                                            0x00,
                                            0x00};
    // 添加CRC
    uint16_t crc = calc_crc_modbus(request, sizeof(request) - 2);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    // 发送请求
    uart->write(request, 8);
}

void AP_RangeFinder_Ultrasonic::read_data(uint8_t* rep) {
    for (int i = 0; i < MODBUS_RESPONSE_SIZE; i++) {
        rep[i] = uart->read();
    }
    if (rep[1] == 0x03 && rep[2] == 0x02) {
        uint32_t sensor_idx = rep[0] - 1;
        if (calc_crc_modbus(rep, 5) == ((rep[6] << 8) | rep[5])) {
            uint16_t raw_val = (rep[3] << 8) | rep[4];
            sensors[sensor_idx].distance = raw_val * 0.001f;
            sensors[sensor_idx].valid = true;
        } else {
            sensors[sensor_idx].valid = false;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "S%d CRC Error", rep[0]);
        }
    }
}

bool AP_RangeFinder_Ultrasonic::get_reading(float& reading_m) {
    bool isRead = false;
    if (uart == nullptr) {
        return false;
    }
    uint8_t rec_num = uart->available();
    if (rec_num > 0) {
        read_data(response);
        isRead = true;
    } else {
        sensors[current_sensor_idx].valid = false;
    }
    // 发送请求
    send_request();
    reading_m = get_min_distance();

    return isRead;
}

// 返回有效的超声波检测的距离最小值
float AP_RangeFinder_Ultrasonic::get_min_distance() {
    float min = MAXFLOAT;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sensors[i].distance < min && sensors[i].valid) {
            min = sensors[i].distance;
        }
    }
    return min;
}
