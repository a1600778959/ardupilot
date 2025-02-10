// libraries/AP_AOA/AP_AOA_ALX.cpp
#include "AP_AOA_ALX.h"

// const AP_Param::GroupInfo AP_AOA_ALX::var_info[] = {
//     AP_GROUPINFO("UART_NUM", 1, AP_AOA_ALX, _uart_num, 3),
//     AP_GROUPEND};

AP_AOA_ALX::AP_AOA_ALX() : _payload_len(0),
                           _payload_cnt(0),
                           _xor_sum(0),
                           _parse_state(WAIT_HEADER1)
{
    memset(&_current, 0, sizeof(_current));
}

void AP_AOA_ALX::init()
{
    _uart = hal.serial(6);
    _uart->begin(230400, 256, 256);
}

void AP_AOA_ALX::update()
{
    while (_uart->available() > 0)
    {
        uint8_t byte = _uart->read();
        switch (_parse_state)
        {
        case WAIT_HEADER1:
            if (byte == 0xFF)
            {
                _parse_state = WAIT_HEADER2;
                _xor_sum = byte;
            }
            break;
        case WAIT_HEADER2:
            if (byte == 0xFF)
            {
                _parse_state = WAIT_HEADER3;
                _xor_sum ^= byte;
            }
            else
            {
                _reset_parser();
            }
            break;
        case WAIT_HEADER3:
            if (byte == 0xFF)
            {
                _parse_state = WAIT_HEADER4;
                _xor_sum ^= byte;
            }
            else
            {
                _reset_parser();
            }
            break;
        case WAIT_HEADER4:
            if (byte == 0xFF)
            {
                _parse_state = PARSE_LENGTH_L;
                _xor_sum ^= byte;
            }
            else
            {
                _reset_parser();
            }
            break;
        case PARSE_LENGTH_L:
            _payload_len = byte;
            _parse_state = PARSE_LENGTH_H;
            _xor_sum ^= byte;
            break;
        case PARSE_LENGTH_H:
            _payload_len |= (byte << 8);
            if (_payload_len > AOA_MAX_PAYLOAD)
            {
                _reset_parser();
                return;
            }
            _parse_state = PARSE_PAYLOAD;
            _payload_cnt = 0;
            _xor_sum ^= byte;
            break;
        case PARSE_PAYLOAD:
            _rx_buffer[_payload_cnt++] = byte;
            _xor_sum ^= byte;
            if (_payload_cnt >= _payload_len)
            {
                _parse_state = CHECK_XOR;
            }
            break;
        case CHECK_XOR:
        {
            uint8_t calc_xor = _xor_sum;
            if (byte == calc_xor)
            {
                _process_packet();
            }
            else
            {
                hal.console->printf("AOA XOR Err:%02x vs %02x\n", byte, calc_xor);
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

void AP_AOA_ALX::_process_packet()
{
    // 解析协议0x2001数据包
    if (_rx_buffer[0] == 0x20 && _rx_buffer[1] == 0x01)
    {
        // 距离解析 (cm转m)
        uint32_t dist_cm = (uint32_t)_rx_buffer[4] << 24 |
                           (uint32_t)_rx_buffer[5] << 16 |
                           (uint32_t)_rx_buffer[6] << 8 |
                           _rx_buffer[7];
        _current.distance_m = dist_cm * 0.01f;

        // 方位角解析 (int16_t)
        int16_t azimuth = (int16_t)(_rx_buffer[8] << 8 | _rx_buffer[9]);
        _current.azimuth_deg = azimuth * 0.1f;

        _current.timestamp_ms = AP_HAL::millis();
        _current.data_valid = true;
    }
}

bool AP_AOA_ALX::get_raw_data(float &dist_m, float &azimuth_deg)
{
    if (_current.data_valid && (AP_HAL::millis() - _current.timestamp_ms < 200))
    {
        dist_m = _current.distance_m;
        azimuth_deg = _current.azimuth_deg;
        _current.data_valid = false;
        return true;
    }
    return false;
}
