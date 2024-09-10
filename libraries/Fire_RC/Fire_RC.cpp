#include "Fire_RC.h"

#include <GCS_MAVLink/GCS.h> //地面站
int16_t Rc_In[25];
uint8_t Fire_RC::Data_Receive_Anl_Task(uint8_t *data_buf, uint16_t num)
{
    uint8_t send_buff[255];
    uint8_t cnt = 0;
    // int16_t zero = 1234;
    // uint32_t GPS = 12345678;
    uint16_t crc = 0;
    volatile int16_t temp_16;
    volatile int32_t temp_32;

    // uint16_t register_add = 0;
    // uint16_t register_num = 0;
    // int16_t write_data = 0;
    // int16_t write_data_buff[27]; // 最多同时写入27个数据
    switch (data_buf[1])
    {
    case 0x03: 

        E_g.Data_Receive_Prepare();

        send_buff[cnt++] = 0x4D; //遥控器ID
        send_buff[cnt++] = 0x03; // 返回功能位
        send_buff[cnt++] = 224; // 返回字节数
        temp_16 = E_g.Bms_info.temp;
        send_buff[cnt++] = BYTE1(temp_16);   // 右电机温度H
        send_buff[cnt++] = BYTE0(temp_16);   // 右电机温度L
        temp_16 = E_g.gases.T_temp/10;
        send_buff[cnt++] = BYTE1(temp_16); // 左电机温度H
        send_buff[cnt++] = BYTE0(temp_16); // 左电机温度L
        temp_16 = E_g.gases.temp;
        send_buff[cnt++] = BYTE1(temp_16);        // 壳体温度H
        send_buff[cnt++] = BYTE0(temp_16);        // 壳体温度L
        temp_16 = E_g.gases.humidity;
        send_buff[cnt++] = BYTE1(temp_16);            // 壳体湿度H
        send_buff[cnt++] = BYTE0(temp_16);            // 壳体湿度L
        temp_16 = Gimbal.pitch;
        send_buff[cnt++] = BYTE1(temp_16);         // 氧气H
        send_buff[cnt++] = BYTE0(temp_16);      // 氧气L
        temp_16 = Gimbal.roll;
        send_buff[cnt++] = BYTE1(temp_16);      // 一氧化碳H
        send_buff[cnt++] = BYTE0(temp_16);      // 一氧化碳L
        temp_16 = E_g.gases.Co2;
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "E_g.gases.Co2:%d", E_g.gases.Co2);
        send_buff[cnt++] = BYTE1(temp_16);        // 二氧化碳H
        send_buff[cnt++] = BYTE0(temp_16);        // 二氧化碳L
        
        send_buff[cnt++] = BYTE1(temp_16);                  // 硫化氢H
        send_buff[cnt++] = BYTE0(temp_16);                  // 硫化氢L
        
        send_buff[cnt++] = BYTE1(temp_16);                  // 氢气H
        send_buff[cnt++] = BYTE0(temp_16);                  // 氢气L
        
        send_buff[cnt++] = BYTE1(temp_16);                  // 甲烷H
        send_buff[cnt++] = BYTE0(temp_16);                  // 甲烷L
        
        send_buff[cnt++] = BYTE1(temp_16);                  // 二氧化氮H
        send_buff[cnt++] = BYTE0(temp_16);                  // 二氧化氮L
        
        send_buff[cnt++] = BYTE1(temp_16);                  // 二氧化硫H
        send_buff[cnt++] = BYTE0(temp_16);                  // 二氧化硫L
        
        send_buff[cnt++] = BYTE1(temp_16);                  // 甲苯H
        send_buff[cnt++] = BYTE0(temp_16);                  // 甲苯L
        
        send_buff[cnt++] = BYTE1(temp_16);                  // 氰化氢H
        send_buff[cnt++] = BYTE0(temp_16);                  // 氰化氢L
        
        send_buff[cnt++] = BYTE1(temp_16);                  // none1
        send_buff[cnt++] = BYTE0(temp_16);                  // none1
        
        send_buff[cnt++] = BYTE1(temp_16);                  // none2
        send_buff[cnt++] = BYTE0(temp_16);                  // none2
        
        send_buff[cnt++] = BYTE1(temp_16);                  // none3
        send_buff[cnt++] = BYTE0(temp_16);                  // none3
        
        send_buff[cnt++] = BYTE1(temp_16);                  // none4
        send_buff[cnt++] = BYTE0(temp_16);                  // none4
        
        send_buff[cnt++] = BYTE1(temp_16);                  // none5
        send_buff[cnt++] = BYTE0(temp_16);                  // none5
        
        send_buff[cnt++] = BYTE1(temp_16);                  // none6
        send_buff[cnt++] = BYTE0(temp_16);                  // none6
        
        send_buff[cnt++] = BYTE1(temp_16);                  // none7
        send_buff[cnt++] = BYTE0(temp_16);                  // none7
        
        send_buff[cnt++] = BYTE1(temp_16);                  // none8
        send_buff[cnt++] = BYTE0(temp_16);                  // none8
        
        send_buff[cnt++] = BYTE1(temp_16);                  // none9
        send_buff[cnt++] = BYTE0(temp_16);                  // none9
        
        send_buff[cnt++] = BYTE1(temp_16);                  // none10
        send_buff[cnt++] = BYTE0(temp_16);                  // none10
        
        send_buff[cnt++] = BYTE1(temp_16);                  // none11
        send_buff[cnt++] = BYTE0(temp_16);                  // none11
        
        send_buff[cnt++] = BYTE1(temp_16);                  // 可燃气体H
        send_buff[cnt++] = BYTE0(temp_16);                  // 可燃气体L
        temp_16 = E_g.gases.NH3;
        send_buff[cnt++] = BYTE1(temp_16);                        // 氨气H
        send_buff[cnt++] = BYTE0(temp_16);                        // 氨气L
        temp_16 = 1234;
        send_buff[cnt++] = BYTE1(temp_16);                     // none12
        send_buff[cnt++] = BYTE0(temp_16);                     // none12
        temp_32 = 12345678;
        send_buff[cnt++] = BYTE3(temp_32 );                     // temp_32经度
        send_buff[cnt++] = BYTE2(temp_32 );                     // temp_32经度
        send_buff[cnt++] = BYTE1(temp_32 );                     // temp_32经度
        send_buff[cnt++] = BYTE0(temp_32 );                     // temp_32经度
        send_buff[cnt++] = BYTE3(temp_32 );                     // temp_32维度
        send_buff[cnt++] = BYTE2(temp_32 );                     // temp_32维度
        send_buff[cnt++] = BYTE1(temp_32 );                     // temp_32维度
        send_buff[cnt++] = BYTE0(temp_32 );                     // temp_32维度
        send_buff[cnt++] = BYTE3(temp_32 );                     // temp_32海拔
        send_buff[cnt++] = BYTE2(temp_32 );                     // temp_32海拔
        send_buff[cnt++] = BYTE1(temp_32 );                     // temp_32海拔
        send_buff[cnt++] = BYTE0(temp_32 );                     // temp_32海拔
        send_buff[cnt++] = BYTE1(temp_16);                         // temp_32航向
        send_buff[cnt++] = BYTE0(temp_16);                         // temp_32航向
        temp_16 = 1;                                               // arming.is_armed();
        send_buff[cnt++] = BYTE1(temp_16);                         // temp_32状态
        send_buff[cnt++] = BYTE0(temp_16);                         // temp_32状态
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 导航状态
        send_buff[cnt++] = BYTE0(temp_16);                         // 导航状态
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 工况状态
        send_buff[cnt++] = BYTE0(temp_16);                         // 工况状态
       
        RC.F8 > 0 ? temp_16 = 0: temp_16 = 1;
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "RC.F8:%d", RC.F8);
        send_buff[cnt++] = BYTE1(temp_16);                         // 雷达开关
        send_buff[cnt++] = BYTE0(temp_16);    
        temp_16 = 1234;                     // 雷达开关
        send_buff[cnt++] = BYTE1(temp_16);                         // none13
        send_buff[cnt++] = BYTE0(temp_16);                         // none13
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 车体pitch
        send_buff[cnt++] = BYTE0(temp_16);                         // 车体pitch
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 车体roll
        send_buff[cnt++] = BYTE0(temp_16);                         // 车体roll
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 车体yaw
        send_buff[cnt++] = BYTE0(temp_16);                         // 车体yaw
        
        send_buff[cnt++] = BYTE1(temp_16);                         // none14
        send_buff[cnt++] = BYTE0(temp_16);                         // none14
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 水炮模式
        send_buff[cnt++] = BYTE0(temp_16);                         // 水炮模式
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 供水水压
        send_buff[cnt++] = BYTE0(temp_16);                         // 供水水压
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 供水流量
        send_buff[cnt++] = BYTE0(temp_16);                         // 供水流量
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 气体探测高度
        send_buff[cnt++] = BYTE0(temp_16);                         // 气体探测高度
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 报警音频
        send_buff[cnt++] = BYTE0(temp_16);                         // 报警音频
        
        send_buff[cnt++] = BYTE1(temp_16);                         // none15
        send_buff[cnt++] = BYTE0(temp_16);                         // none15
        
        send_buff[cnt++] = BYTE1(temp_16);                         // none16
        send_buff[cnt++] = BYTE0(temp_16);                         // none16
        
        send_buff[cnt++] = BYTE1(temp_16);                         // none17
        send_buff[cnt++] = BYTE0(temp_16);                         // none17
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 视觉云台水平角度
        send_buff[cnt++] = BYTE0(temp_16);                         // 视觉云台水平角度
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 视觉云台pitch数值
        send_buff[cnt++] = BYTE0(temp_16);                         // 视觉云台pitch数值
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 水炮水平角度
        send_buff[cnt++] = BYTE0(temp_16);                         // 水炮水平角度
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 水炮pitch
        send_buff[cnt++] = BYTE0(temp_16);                         // 水炮pitch
        
        send_buff[cnt++] = BYTE1(temp_16);                         // none18
        send_buff[cnt++] = BYTE0(temp_16);                         // none18
        
        send_buff[cnt++] = BYTE1(temp_16);                         // none19
        send_buff[cnt++] = BYTE0(temp_16);                         // none19
        
        send_buff[cnt++] = BYTE1(temp_16);                         // none20
        send_buff[cnt++] = BYTE0(temp_16);                         // none20
        
        send_buff[cnt++] = BYTE1(temp_16);                         // none21
        send_buff[cnt++] = BYTE0(temp_16);                         // none21
        temp_16 = 1;
        send_buff[cnt++] = BYTE1(temp_16);                         // 车体ID
        send_buff[cnt++] = BYTE0(temp_16);                         // 车体ID
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 车体状态
        send_buff[cnt++] = BYTE0(temp_16);                         // 车体状态
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 车体报警码
        send_buff[cnt++] = BYTE0(temp_16);                         // 车体报警码
        temp_16 = E_g.Bms_info.RSOC;
        send_buff[cnt++] = BYTE1(temp_16);                         // 车体电量
        send_buff[cnt++] = BYTE0(temp_16);                         // 车体电量
        temp_16 = (hal.rcout->read(2));
        send_buff[cnt++] = BYTE1(temp_16);                         // 左电机转速
        send_buff[cnt++] = BYTE0(temp_16);                         // 左电机转速
        temp_16 = (hal.rcout->read(1));
        send_buff[cnt++] = BYTE1(temp_16);                         // 右电机转速
        send_buff[cnt++] = BYTE0(temp_16);                         // 右电机转速
        temp_16 = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight) / 1000.0f);
        send_buff[cnt++] = BYTE1(temp_16);                         // 车体放电电流
        send_buff[cnt++] = BYTE0(temp_16);                         // 车体放电电流
        temp_16 = E_g.Bms_info.I_cur;
        send_buff[cnt++] = BYTE1(temp_16);                         // 车体充电电流
        send_buff[cnt++] = BYTE0(temp_16);                         // 车体充电电流
        temp_16 = E_g.Bms_info.All_Vol;
        send_buff[cnt++] = BYTE1(temp_16);                         // 车体电池电压
        send_buff[cnt++] = BYTE0(temp_16);                         // 车体电池电压
        temp_16 = 1234;
        send_buff[cnt++] = BYTE1(temp_16);                         // 回零设置
        send_buff[cnt++] = BYTE0(temp_16);                         // 回零设置
        
        send_buff[cnt++] = BYTE1(temp_16);                         // none22
        send_buff[cnt++] = BYTE0(temp_16);                         // none22
        
        send_buff[cnt++] = BYTE1(temp_16);                         // none23
        send_buff[cnt++] = BYTE0(temp_16);                         // none23
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 左前电流*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 左前电流*0.1
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 左前电压*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 左前电压*0.1
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 左前转速
        send_buff[cnt++] = BYTE0(temp_16);                         // 左前转速
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 左前温度
        send_buff[cnt++] = BYTE0(temp_16);                         // 左前温度
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 右前电流*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 右前电流*0.1
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 右前电压*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 右前电压*0.1
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 右前转速
        send_buff[cnt++] = BYTE0(temp_16);                         // 右前转速
        
        send_buff[cnt++] = BYTE1(temp_16);                         // 右前温度
        send_buff[cnt++] = BYTE0(temp_16);                         // 右前温度
        send_buff[cnt++] = BYTE1(temp_16);                         // 左后电流*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 左后电流*0.1
        send_buff[cnt++] = BYTE1(temp_16);                         // 左后电压*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 左后电压*0.1
        send_buff[cnt++] = BYTE1(temp_16);                         // 左后转速
        send_buff[cnt++] = BYTE0(temp_16);                         // 左后转速
        send_buff[cnt++] = BYTE1(temp_16);                         // 左后温度
        send_buff[cnt++] = BYTE0(temp_16);                         // 左后温度
        send_buff[cnt++] = BYTE1(temp_16);                         // 右后电流*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 右后电流*0.1
        send_buff[cnt++] = BYTE1(temp_16);                         // 右后电压*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 右后电压*0.1
        send_buff[cnt++] = BYTE1(temp_16);                         // 右后转速
        send_buff[cnt++] = BYTE0(temp_16);                         // 右后转速
        send_buff[cnt++] = BYTE1(temp_16);                         // 右后温度
        send_buff[cnt++] = BYTE0(temp_16);                         // 右后温度
        send_buff[cnt++] = BYTE1(temp_16);                         // 左行走电流*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 左行走电流*0.1
        send_buff[cnt++] = BYTE1(temp_16);                         // 左行走电压*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 左行走电压*0.1
        send_buff[cnt++] = BYTE1(temp_16);                         // 左行走转速
        send_buff[cnt++] = BYTE0(temp_16);                         // 左行走转速
        send_buff[cnt++] = BYTE1(temp_16);                         // 左行走温度
        send_buff[cnt++] = BYTE0(temp_16);                         // 左行走温度
        send_buff[cnt++] = BYTE1(temp_16);                         // 右行走电流*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 右行走电流*0.1
        send_buff[cnt++] = BYTE1(temp_16);                         // 右行走电压*0.1
        send_buff[cnt++] = BYTE0(temp_16);                         // 右行走电压*0.1
        send_buff[cnt++] = BYTE1(temp_16);                         // 右行走转速
        send_buff[cnt++] = BYTE0(temp_16);                         // 右行走转速
        send_buff[cnt++] = BYTE1(temp_16);                         // 右行走温度
        send_buff[cnt++] = BYTE0(temp_16);                         // 右行走温度
        send_buff[cnt++] = BYTE3(temp_32);                          // 左前姿态电机报警码S96-97
        send_buff[cnt++] = BYTE2(temp_32);                          // 左前姿态电机报警码S96-97
        send_buff[cnt++] = BYTE1(temp_32);                          // 左前姿态电机报警码S96-97
        send_buff[cnt++] = BYTE0(temp_32);                          // 左前姿态电机报警码S96-97
        send_buff[cnt++] = BYTE3(temp_32);                          // 右前姿态电机报警码S98-99
        send_buff[cnt++] = BYTE2(temp_32);                          // 右前姿态电机报警码S98-99
        send_buff[cnt++] = BYTE1(temp_32);                          // 右前姿态电机报警码S98-99
        send_buff[cnt++] = BYTE0(temp_32);                          // 右前姿态电机报警码S98-99
        send_buff[cnt++] = BYTE3(temp_32);                          // 左后姿态电机报警码S100-101
        send_buff[cnt++] = BYTE2(temp_32);                          // 左后姿态电机报警码S100-101
        send_buff[cnt++] = BYTE1(temp_32);                          // 左后姿态电机报警码S100-101
        send_buff[cnt++] = BYTE0(temp_32);                          // 左后姿态电机报警码S100-101
        send_buff[cnt++] = BYTE3(temp_32);                          // 右后姿态电机报警码S102-103
        send_buff[cnt++] = BYTE2(temp_32);                          // 右后姿态电机报警码S102-103
        send_buff[cnt++] = BYTE1(temp_32);                          // 右后姿态电机报警码S102-103
        send_buff[cnt++] = BYTE0(temp_32);                          // 右后姿态电机报警码S102-103
        send_buff[cnt++] = BYTE3(temp_32);                          // 左行走电机报警码S104-105
        send_buff[cnt++] = BYTE2(temp_32);                          // 左行走电机报警码S104-105
        send_buff[cnt++] = BYTE1(temp_32);                          // 左行走电机报警码S104-105
        send_buff[cnt++] = BYTE0(temp_32);                          // 左行走电机报警码S104-105
        send_buff[cnt++] = BYTE3(temp_32);                          // 右行走电机报警码S106-107
        send_buff[cnt++] = BYTE2(temp_32);                          // 右行走电机报警码S106-107
        send_buff[cnt++] = BYTE1(temp_32);                          // 右行走电机报警码S106-107
        send_buff[cnt++] = BYTE0(temp_32);                          // 右行走电机报警码S106-107

        send_buff[cnt++] = BYTE1(temp_16); // none24
        send_buff[cnt++] = BYTE0(temp_16); // none24

        send_buff[cnt++] = BYTE1(temp_16); // none25
        send_buff[cnt++] = BYTE0(temp_16); // none25

        send_buff[cnt++] = BYTE1(temp_16); // none26
        send_buff[cnt++] = BYTE0(temp_16); // none26

        send_buff[cnt++] = BYTE1(temp_16); // none27
        send_buff[cnt++] = BYTE0(temp_16); // none27

        crc = CRC.Funct_CRC16(send_buff, cnt);                        // 官方给的CRC校验
        send_buff[cnt++] = BYTE0(crc);
        send_buff[cnt++] = BYTE1(crc);
        hal.serial(6)->write(send_buff, cnt);
        return 1; //表示气体传感器更新完毕
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "cnt:%d", cnt);
        // 如果是读寄存器则执行以下函数
        // register_add = data_buf[2] << 8 | data_buf[3];
        // register_num = data_buf[4] << 8 | data_buf[5];
        // read_register_command(register_add, register_num);
        /*需要后期增加读取函数编写*/
        break;
    case 0x10:
        if ( data_buf[3] == 0x10)
        {
            RC.T0 = (data_buf[7] << 8) + data_buf[8];   //云台
            // Rc_In[12] = RC.T0 * 500 + 1000;
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Rc_In[12]:%d", Rc_In[12]);
            RC.T2 = (data_buf[11] << 8) + data_buf[12];   //喷雾/聚焦
            if (RC.T2  <= 1 )
            {
                Rc_In[13] = -RC.T2 * 500 + 1500;
            }
            else if (RC.T2 == 2)
            {
                Rc_In[13] = 2000;
            }

            RC.T4 = (data_buf[15] << 8) + data_buf[16]; // 车灯开关
            Rc_In[14] = RC.T4 * 1000 + 1000;
            RC.T7 = (data_buf[21] << 8) + data_buf[22];   //预留
            if (RC.T7 <= 1)
            {
                Rc_In[15] = -RC.T7 * 500 + 1500;
            }
            else if (RC.T7 == 2)
            {
                Rc_In[15] = 2000;
            }
            RC.T8 = (data_buf[23] << 8) + data_buf[24];   //预留
            if (RC.T8 <= 1)
            {
                Rc_In[16] = -RC.T8 * 500 + 1500;
            }
            else if (RC.T8 == 2)
            {
                Rc_In[16] = 2000;
            }
            RC.T9 = (data_buf[25] << 8) + data_buf[26];   //跟随遥控
            Rc_In[17] = RC.T9 * 1000 + 1000;
            RC.T10 = (data_buf[27] << 8) + data_buf[28];  //模式切换
            Rc_In[18] = RC.T10 * 142 + 858;
            (Rc_In[18] > 2000) ? Rc_In[18] = 2000 : Rc_In[18] = Rc_In[18];
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Rc_In[18]:%d", Rc_In[18]);
            /* code */
            return 2; // 表示遥控器更新完毕
        }
        else if(data_buf[3] == 0x50)
        {
            RC.P0_LR = data_buf[7];
            Rc_In[3] = data_buf[7] * 3.9063f + 1000;
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Rc_In[0]:%d", Rc_In[0]);
            RC.P0_UD = data_buf[8];
            Rc_In[2] = data_buf[8] * 3.9063f + 1000;
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Rc_In[1]:%d", Rc_In[1]);
            RC.P1_LR = data_buf[9];
            Rc_In[0] = data_buf[9] * 3.9063f + 1000;
            RC.P1_UD = data_buf[10];
            Rc_In[1] = data_buf[10] * 3.9063f + 1000;
            RC.P2_LR = data_buf[11];
            Rc_In[4] = data_buf[11] * 3.9063f + 1000;
            RC.P2_UD = data_buf[12];
            Rc_In[5] = data_buf[12] * 3.9063f + 1000;
            RC.P3_LR = data_buf[13];
            Rc_In[6] = data_buf[13] * 3.9063f + 1000;
            RC.P3_UD = data_buf[14];
            Rc_In[7] = data_buf[14] * 3.9063f + 1000;
            RC.P4_LR = data_buf[15];
            Rc_In[8] = data_buf[15] * 3.9063f + 1000;
            RC.P4_UD = data_buf[16];
            Rc_In[9] = data_buf[16] * 3.9063f + 1000;
            RC.P5_LR = data_buf[17];
            Rc_In[10] = data_buf[17] * 3.9063f + 1000;
            RC.P5_UD = data_buf[18];
            Rc_In[11] = data_buf[18] * 3.9063f + 1000;
        }
        return 2; // 表示遥控器更新完毕
        break;
    case 0x0F:
        RC.F0 = (data_buf[7] & 0x01);
        Rc_In[12] = -RC.F0 * 1000 + 2000;
        RC.F5 = (uint8_t)((data_buf[7] & 0x20) >> 5);
        Rc_In[19] = -RC.F5 * 1000 + 2000;
        RC.F8 = (uint8_t)((data_buf[8] & 0x01) >> 0);
        Rc_In[20] = -RC.F8 * 1000 + 2000;
        RC.F18 = (uint8_t)((data_buf[9] & 0x02) >> 1);
        RC.F17 = (uint8_t)((data_buf[9] & 0x04) >> 2);
        RC.F21 = (uint8_t)((data_buf[9] & 0x20) >> 5);
        Rc_In[21] = RC.F21 * 1000 + 1000;
        RC.F22 = (uint8_t)((data_buf[9] & 0x40) >> 6);
        Rc_In[22] = RC.F22 * 1000 + 1000;
        return 2; // 表示遥控器更新完毕
        break;
    }
    return 0; // 表示遥错误
}

int16_t Fire_RC::get_RC(uint8_t num)
{
    if (num <= 18)
    {
        return Rc_In[num]; /* code */
    }
    return -1;
}
uint8_t Fire_RC::Data_Receive_Prepare()
{
    static uint8_t frist_rcin = 0;
    uint8_t num = hal.serial(6)->available(); // 读取串口有多少个数据
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "num:%d", num);
    uint8_t c;
    static uint8_t stat = 0, rece_len = 0, data_len = 0;
    static uint8_t data_buff[255];
    while(num > 0)
    {
        c = hal.serial(6)->read();
        // copter_rc.MAVlink_Data_Receive_Prepare(c);
        num--;
        if (stat == 0)
        {
            if (c == 0x4D)  //如果等于发射器发送的帧头0x4D,则记录
            {
                data_buff[rece_len++] = c;
                stat++;
            }
        }
        else if (stat == 1) // 接收功能码
        {
            if (c == 0x03 || c == 0x10 || c == 0x0F)
            {
                data_buff[rece_len++] = c;
                stat++;
                data_len = 4;
                /* code */
            }
            else
            {
                rece_len = 0;
                stat = 0;
            }
        }
        else if (stat == 2 && data_len > 0) // 记录寄存器地址
        {
            data_len--;
            data_buff[rece_len++] = c;  //直接接收起始地址与寄存器数量
            if (data_len == 0)
            {
                stat++;
            } 
        }
        else if (stat == 3) // 根据上面的功能码进行接下来的数据接收
        {
            data_buff[rece_len++] = c;  //这里已经接收完一个，故下面的需要减掉1
            stat++;
            switch (data_buff[1])
            {
            case 0x03:        // 如果是读取寄存器
                data_len = 1; // 如果功能码是0x03仅需接收2个CRC即可完成接受
                break;
            case 0x0F:        // 写单个寄存器
                data_len = 10; // 如果功能码是0x03仅需接收2个CRC+8个数据位+1个字节数即可完成接受
                break;
            case 0x10:
                data_len = ((data_buff[4] << 8) + data_buff[5])*2 + 2;
                break;
            default:
                rece_len = 0;
                stat = 0; // 当不是上面任何的数据的话，不进行后面的接收
                break;
            }
        }
        else if (stat == 4 && data_len > 0)
        {
            data_len--;
            data_buff[rece_len++] = c;
            if (data_len == 0) // 代表接收完毕
            {
                uint16_t crc = CRC.Funct_CRC16(data_buff, rece_len - 2);
                if (crc == ((data_buff[rece_len - 2]) | data_buff[rece_len - 1] << 8)) // 表示校验通过可以进行解析
                {
                    uint8_t res = Data_Receive_Anl_Task(data_buff, rece_len);
                    // gcs().send_text(MAV_SEVERITY_CRITICAL, "res:%d", res);
                    rece_len = 0;
                    stat = 0;
                    if (frist_rcin == 0 )
                    {
                        if (res == 2 && Rc_In[3] > 1400)
                        {
                            frist_rcin = 1;
                            return res;
                        }
                        else
                        {
                            return 3;
                        }
                        /* code */
                    }
                    return res;
                    
                     
                }
                rece_len = 0;
                stat = 0;
            }
        }
        else
        {
            rece_len = 0;
            stat = 0;
        }
    }
    if (frist_rcin == 0)
    {
        return 3;
    }
    else
    {
        return 0;
    }
        
}