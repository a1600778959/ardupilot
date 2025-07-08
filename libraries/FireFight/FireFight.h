#pragma once
#ifndef _FireFight_H_
#define _FireFight_H_
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "FireFightCRC.h"
#include <RC_Channel/RC_Channel.h>
// #include <AP_Param/AP_Param.h>
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

extern const AP_HAL::HAL &hal;

class FireFight
{
private:
    uint8_t linebuf[10];
    uint8_t linebuf_len = 0;
    FireFightCRC CRC;

    // void write_two(uint8_t address_ID,uint16_t start_reg_adress,uint16_t val_1,uint16_t val_2);

    /* data */
public:
    void uart_init(void);
    void read(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num);
    void write_one(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num);
    void write_six(uint8_t address_ID, uint16_t start_reg_adress, int16_t val_1, int16_t val_2, int16_t val_3, int16_t val_4, int16_t val_5, int16_t val_6); // 连续写六个参数
    // uint8_t check_send_one(uint16_t val);
    uint8_t check_send_one(uint8_t addressID);
    void function_fire_fight(uint8_t DT_ms); // 1号消防炮板子，具体定义见文档
    void parm_change();
    void write_two(uint8_t address_ID, uint16_t start_reg_adress, int16_t val_1, int16_t val_2);
    void up_button(uint16_t val);
    void down_button(uint16_t val);
    void left_button(uint16_t val);
    void right_button(uint16_t val);
    void zhu_button(uint16_t val);
    void wu_button(uint16_t val);
    void upanddown_zero();
    void leftandright_zero();
    void wuzhu_zero();
    void zhu_zero();
    void wu_zero();
    void valve_button(uint16_t val);    // 阀门按钮
    void pump_button(uint16_t val);     // 泵按钮
    void Record_button(uint16_t val);   // 录制按钮
    void playback_button(uint16_t val); // 回放按钮
    void FireFight_ID2(uint8_t DT_ms);  // 2号消防炮板子，声光报警1；推杆风扇电机2；自喷淋3
    void FireFight_ID3(uint8_t DT_ms);  // 3号消防炮控制板，拖带离合器1；风扇2；GPIO灯控3、4
    int16_t address_1, address_2;
    uint16_t Set_Left_motor, Read_Left_motor;
    uint16_t Set_Right_motor, Read_Right_motor;
    static const struct AP_Param::GroupInfo var_info[];
// protected:
    AP_Int16 LR; //堵转电流左右
    AP_Int16 UD; //堵转电流上下
    AP_Int16 BUARD; //与消防炮波特率数值
};

#endif