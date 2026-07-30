#include "AP_AOA_ALX.h"
#include <GCS_MAVLink/GCS.h> //地面站

AP_AOA_ALX::AP_AOA_ALX() : _uart(nullptr),
                           _payload_len(0),
                           _payload_cnt(0),
                           _xor_sum(0),
                           _parse_state(WAIT_HEADER1)
{
    memset(&_current, 0, sizeof(_current));
}

void AP_AOA_ALX::init(uint8_t serial_num)
{
    _uart = AP_HAL::get_HAL().serial(serial_num);
    if (_uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AOA serial %u unavailable", (unsigned)serial_num);
        return;
    }
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_stop_bits(1);
}

void AP_AOA_ALX::update()
{
    if (_uart == nullptr) {
        return;
    }
    uint16_t rec_num = _uart->available();

    while (rec_num > 0)
    {
        rec_num--;
        uint8_t byte = _uart->read();
        switch (_parse_state)
        {
        case WAIT_HEADER1:
            if (byte == 0x59)  //帧头0✖59 B1
            {
                _xor_sum = 0;
                _rx_buffer[0] = byte;
                _xor_sum += byte;
                _parse_state = WAIT_HEADER2;
            }
            break;
        case WAIT_HEADER2:
            if (byte == 0x4D)  //帧头0✖40 B2
            {
                _rx_buffer[1] = byte;
                _xor_sum += byte;
                _parse_state = WAIT_HEADER3;
            }
            else
            {
                _reset_parser();
            }
            break;
        case WAIT_HEADER3:
            if (byte == 0x17)  //PDOA跟随模式0X17 B3
            {
                _rx_buffer[2] = byte;
                _xor_sum += byte;
                _parse_state = SEQ_B4;
            }
            else
            {
                _reset_parser();
            }
            break;
        case SEQ_B4: // 帧序列号
            _rx_buffer[3] = byte;
            _xor_sum += byte;
            _parse_state = SEQ_B5;
            break;
        case SEQ_B5: // 帧序列号
            _rx_buffer[4] = byte;
            _xor_sum += byte;
            _parse_state = PARSE_LENGTH_L;
            break;
        case PARSE_LENGTH_L:
            _rx_buffer[5] = byte;
            _payload_len = byte;
            _xor_sum += byte;
            _parse_state = PARSE_LENGTH_H;
            break;
        case PARSE_LENGTH_H:
            _rx_buffer[6] = byte;
            _payload_len |= (byte << 8);
            _xor_sum += byte;
            if (_payload_len != AOA_MAX_PAYLOAD)  //数据长度不等于37则返回
            {
                _reset_parser();
                break;
            }
            _parse_state = PARSE_PAYLOAD;
            _payload_cnt = 7; //前7个字节已存储

            break;
        case PARSE_PAYLOAD:
            if (_payload_cnt >= sizeof(_rx_buffer)) {
                _reset_parser();
                break;
            }
            _rx_buffer[_payload_cnt++] = byte;
            _xor_sum += byte;
            if (_payload_cnt >= _payload_len + 7)
            {
                _parse_state = CHECK_SUM;
            }
            break;
        case CHECK_SUM:
        {
            uint8_t calc_xor = _xor_sum;
            if (byte == calc_xor)
            {
                _process_packet();
            }
            else
            {
                gcs().send_text(MAV_SEVERITY_INFO, "AOA XOR Err:%02x vs %02x\n", byte, calc_xor);
            }
            _reset_parser();
            break;
        }
        default:
            _reset_parser();
            break;
        }
    }
}

void AP_AOA_ALX::_reset_parser()
{
    _payload_len = 0;
    _payload_cnt = 0;
    _xor_sum = 0;
    _parse_state = WAIT_HEADER1;
}

void AP_AOA_ALX::_process_packet()
{
    // 解析协议0x17数据包
    if (_rx_buffer[2] == 0x17)
    {
        // 距离解析 (cm转m)
        uint32_t dist_cm = (uint32_t)_rx_buffer[15] << 24 |
                           (uint32_t)_rx_buffer[14] << 16 |
                           (uint32_t)_rx_buffer[13] << 8 |
                           _rx_buffer[12];
        _current.distance_m = dist_cm * 0.01f;

        // 方位角解析 (int16_t)
        int16_t azimuth = (int16_t)(_rx_buffer[18] << 8 | _rx_buffer[17]);
        _current.azimuth_deg = azimuth;

        _current.timestamp_ms = AP_HAL::millis();
        _current.data_confirmed = _rx_buffer[19];
    }
}

bool AP_AOA_ALX::get_raw_data(float &dist_m, float &azimuth_deg)
{
    if ((_current.data_confirmed > 90) && (AP_HAL::millis() - _current.timestamp_ms < 1000))
    {
        dist_m = _current.distance_m;
        azimuth_deg = _current.azimuth_deg;
        _current.data_confirmed = 0;
        return true;
    }
    return false;
}
