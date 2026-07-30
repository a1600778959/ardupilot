#pragma once
#include <AP_HAL/AP_HAL.h>
#define AOA_MAX_PAYLOAD 118
class AP_AOA_ALX
{
public:
    AP_AOA_ALX();
    void init(uint8_t sernum);
    void update();
    bool get_raw_data(float &dist_m, float &azimuth_deg);

private:
    enum ParseState
    {
        WAIT_HEADER1,
        WAIT_HEADER2,
        WAIT_HEADER3,
        SEQ_B4,
        SEQ_B5,
        PARSE_LENGTH_L,
        PARSE_LENGTH_H,
        PARSE_PAYLOAD,
        CHECK_SUM
    };
    AP_HAL::UARTDriver *_uart;
    uint8_t _rx_buffer[AOA_MAX_PAYLOAD+10];
    uint16_t _payload_len;
    uint16_t _payload_cnt;
    uint8_t _xor_sum;
    ParseState _parse_state;

    typedef struct Follow_Data_t
    {
        uint32_t timestamp_ms;
        float distance_m;    //距离，单位米
        float azimuth_deg;   //方位角，单位度
        uint8_t data_confirmed; //数据可信度

    } Follow_Data;
    Follow_Data _current;
    void _process_packet();
    void _reset_parser();
};
