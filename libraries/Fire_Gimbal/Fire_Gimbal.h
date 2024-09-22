#pragma once
#ifndef _Fire_Gimbal_h_
#define _Fore_Gimbal_h_
#include <FireFight/FireFightCRC.h>
#include <Fire_RC/Fire_RC.h>
// #include <RC_Channel/RC_Channel.h>
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))
struct Gimbal_str
{
    int16_t pitch;
    int16_t roll;
    /* data */
};

extern const AP_HAL::HAL &hal;
extern Gimbal_str Gimbal;

class Fire_Gimbal
{
private:
    FireFightCRC CRC;
    void Data_Receive_Anl_Task(uint8_t *data_buf, uint16_t num);
    void inc_multiple();           // 变倍+
    void red_multiple();           // 变倍-
    void inc_zoom();               // 变焦+
    void red_zoom();               // 变焦-
    void inc_aperture();           // 光圈+
    void red_aperture();           // 光圈-
    void open_LED();               // 灯光开
    void close_LED();              // 灯光关
    void open_windscreen_wiper();  // 雨刷开
    void close_windscreen_wiper(); // 雨刷关
    void turn_to_save_loc();       // 转到保存位置001
    void save_loc();               // 设置预置位置001
    void up_right(uint8_t pitch,uint8_t roll);//上右边控制
    void up_left(uint8_t pitch, uint8_t roll); // 上左边控制
    void down_right(uint8_t pitch, uint8_t roll); // 下右边控制
    void down_left(uint8_t pitch, uint8_t roll); // 下左边控制


    /* data */
public:

    // void read_pitch();
    // void read_roll();
    void Data_Receive_Prepare();
    void control_by_RC();
    void stop(); // 停止
    void control_pitch(int16_t pitch);
    void control_roll(int16_t roll);
};



#endif