#pragma once
#ifndef _Explosion_gases_h_
#include <FireFight/FireFightCRC.h>
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))
#define Thermal_ID 0x01   //热成像ID
#define Gases_ID 0x02     //气体传感器ID
#define Bms_ID 0XDD       //Bms传感器ID
#define Broa_ID 0x03       // 压力传感器ID
#define Temp_ID 0x04      //温湿度传感器
extern const AP_HAL::HAL &hal;

struct Ex_Gases
{
    int16_t Co2;    //外部一氧化碳含量
    int16_t NH3;
    int16_t temp; //设备温度
    int16_t humidity; //外部湿度
    int16_t waterpress; //水压
    int16_t T_temp;  //外部温湿度传感器温度
    int16_t T_humidity; // 外部温湿度传感器湿度

    /* data */
};

struct Thermal_imaging
{
   uint16_t x;
   uint16_t y;
   int16_t temp;
    /* data */
};

struct Bms_info_str
{
    uint16_t All_Vol;   // 电池总电压 单位 10mV，高字节在前
    int16_t I_cur;     //当前电流数值 单位 10mA
    uint8_t RSOC;       // 电池剩余电量%
    uint16_t temp;      //电池温度

};

class Explosion_gases
{
private:
    FireFightCRC CRC;
    
    void Data_Receive_Anl_Task(uint8_t *data_buf, uint16_t num);//数据处理函数
    
    void read(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num); // 只需要填写寄存器ID和寄存器个数
    void read_Bms(uint16_t comand_ID);                  // 只需要填写寄存器ID和寄存器个数
        /* data */
public : 
    Ex_Gases gases;
    Thermal_imaging Ther_ima;
    Bms_info_str Bms_info;
    void get_Thermal_imaging();    //读取热成像数据
    void read_Explosion_gasese();  //读取气体传感器数据
    void get_Bms_Info();     //读取BMS信息
    void get_Broa_info();     //读取水压
    void get_Temp();           //读取温湿度
    void Data_Receive_Prepare(); // 数据接收准备
};  



#endif