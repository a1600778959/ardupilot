#include "FireFight.h"
#include <stdio.h>
#include <GCS_MAVLink/GCS.h> //地面站
#include <array>             // 使用标准库中的array代替C风格数组
// #include "RC_Channel.h"         //加入遥控读取通道
// #include "rover/Rover.h"

#define FRAME_LENGTH 9  // 帧长

// class Action
// {
// public:
//     int16_t record_Left_Right_pulse, record_Up_Down_pulse;
//     uint16_t record_delay;
//     Action() : record_Left_Right_pulse(0), record_Up_Down_pulse(0), record_delay(0) {} // 添加默认构建函数
//     Action(int16_t Action_record_Left_Right_pulse, int16_t Action_record_Up_Down_pulse, uint16_t Action_record_delay) : record_Left_Right_pulse(Action_record_Left_Right_pulse), record_Up_Down_pulse(Action_record_Up_Down_pulse), record_delay(Action_record_delay) {}
// };

// const volatile Action actions[MAX_ACTIONS]; // 动作数组
// std::array<Action, MAX_ACTIONS> actions;
// volatile uint8_t num_actions = 0;  // 记录的动作数量
// volatile uint8_t action_index = 0; // 执行动作

void FireFight::uart_init()
{

    // hal.serial(1)->begin(256000); // 初始化串口程序
    hal.serial(1)->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    hal.serial(1)->set_unbuffered_writes(true);
    hal.scheduler->delay(1); // 等待初始化串口
    read(1,2,2);             // 读取设备堵转电流
    hal.scheduler->delay(2); // 等待串口回复
    check_send_one(1);
    AP_Param::set_object_value(this, var_info, "LR", address_1);
    AP_Param::set_object_value(this, var_info, "UD", address_2);
    read(1, 2, 23);           // 读取设备波特率设置
    hal.scheduler->delay(2); // 等待串口回复
    check_send_one(1);
    AP_Param::set_object_value(this, var_info, "BUARD", address_1);
    // LR = (AP_Int16)address_1;
    // UD = (AP_Int16)address_2;
    // write_one(0x01, 0x0002, 10);
    // hal.scheduler->delay(100); // 上下电机堵转电流
    // write_one(0x01, 0x0003, 200);
    // hal.scheduler->delay(100); // 雾柱电机堵转电流
    // write_one(0x01, 0x0004, 1);
    // hal.scheduler->delay(100); // 堵转时间
    // hal.serial(3)->begin(115200);
    AP_Param::setup_object_defaults(this, var_info);  //调用初始化
    gcs()
        .send_text(MAV_SEVERITY_CRITICAL, // 地面站消息发送
                   "uart set ok");
}

void FireFight::read(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num) // 只需要填写寄存器ID和寄存器个数
{
    uint8_t data_to_send[10];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    data_to_send[cnt++] = address_ID; // 设备地址为01
    data_to_send[cnt++] = 0x03;       // 读取的功能码为03
    data_to_send[cnt++] = BYTE1(reg_adress);
    data_to_send[cnt++] = BYTE0(reg_adress);
    data_to_send[cnt++] = BYTE1(reg_num);
    data_to_send[cnt++] = BYTE0(reg_num);
    crc = CRC.Funct_CRC16(data_to_send, cnt); // 官方给的CRC校验
    data_to_send[cnt++] = BYTE0(crc);
    data_to_send[cnt++] = BYTE1(crc);
    hal.serial(1)->write(data_to_send, cnt);
}

void FireFight::write_one(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num) // 只需要填写寄存器ID和寄存器个数
{
    uint8_t data_to_send[10];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    data_to_send[cnt++] = address_ID; // 设备地址为01
    data_to_send[cnt++] = 0x06;       // 写入的功能码为06
    data_to_send[cnt++] = BYTE1(reg_adress);
    data_to_send[cnt++] = BYTE0(reg_adress);
    data_to_send[cnt++] = BYTE1(reg_num);
    data_to_send[cnt++] = BYTE0(reg_num);
    crc = CRC.Funct_CRC16(data_to_send, cnt); // 官方给的CRC校验
    data_to_send[cnt++] = BYTE0(crc);
    data_to_send[cnt++] = BYTE1(crc);
    hal.serial(1)->write(data_to_send, cnt);
}

void FireFight::write_six(uint8_t address_ID, uint16_t start_reg_adress, int16_t val_1, int16_t val_2, int16_t val_3, int16_t val_4, int16_t val_5, int16_t val_6) // 写两个寄存器，用于归零
{
    uint8_t data_to_send[30];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    uint16_t write_num = 0x0006;      // 怀疑写入地址错误，未验证，若验证清删除
    data_to_send[cnt++] = address_ID; // 设备地址为01
    data_to_send[cnt++] = 0x10;       // 写入的功能码为0x10
    data_to_send[cnt++] = BYTE1(start_reg_adress);
    data_to_send[cnt++] = BYTE0(start_reg_adress);
    data_to_send[cnt++] = BYTE1(write_num); // 写入数量
    data_to_send[cnt++] = BYTE0(write_num);
    data_to_send[cnt++] = 12; // 写入字节数
    data_to_send[cnt++] = BYTE1(val_1);
    data_to_send[cnt++] = BYTE0(val_1);
    data_to_send[cnt++] = BYTE1(val_2);
    data_to_send[cnt++] = BYTE0(val_2);
    data_to_send[cnt++] = BYTE1(val_3);
    data_to_send[cnt++] = BYTE0(val_3);
    data_to_send[cnt++] = BYTE1(val_4);
    data_to_send[cnt++] = BYTE0(val_4);
    data_to_send[cnt++] = BYTE1(val_5);
    data_to_send[cnt++] = BYTE0(val_5);
    data_to_send[cnt++] = BYTE1(val_6);
    data_to_send[cnt++] = BYTE0(val_6);
    crc = CRC.Funct_CRC16(data_to_send, cnt); // 官方给的CRC校验
    data_to_send[cnt++] = BYTE0(crc);
    data_to_send[cnt++] = BYTE1(crc);
    hal.serial(1)->write(data_to_send, cnt);
}

void FireFight::write_two(uint8_t address_ID, uint16_t start_reg_adress, int16_t val_1, int16_t val_2) // 写两个寄存器，用于归零
{
    uint8_t data_to_send[15];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    uint16_t write_num = 0x0002;      // 怀疑写入地址错误，未验证，若验证清删除
    data_to_send[cnt++] = address_ID; // 设备地址为01
    data_to_send[cnt++] = 0x10;       // 写入的功能码为0x10
    data_to_send[cnt++] = BYTE1(start_reg_adress);
    data_to_send[cnt++] = BYTE0(start_reg_adress);
    data_to_send[cnt++] = BYTE1(write_num); // 写入数量
    data_to_send[cnt++] = BYTE0(write_num);
    data_to_send[cnt++] = 0x04; // 写入字节数
    data_to_send[cnt++] = BYTE1(val_1);
    data_to_send[cnt++] = BYTE0(val_1);
    data_to_send[cnt++] = BYTE1(val_2);
    data_to_send[cnt++] = BYTE0(val_2);
    crc = CRC.Funct_CRC16(data_to_send, cnt); // 官方给的CRC校验
    data_to_send[cnt++] = BYTE0(crc);
    data_to_send[cnt++] = BYTE1(crc);
    hal.serial(1)->write(data_to_send, cnt);
}

uint8_t FireFight::check_send_one(uint8_t addressID)
{
    // uint16_t reg_adress,reg_num;
    uint8_t num = hal.serial(1)->available(); // 读取串口有多少个数据
    uint8_t c;
    static uint8_t stat = 0, len_date = 0;
    if (num > 0)
    {
        // hal.console->printf("当前有%d个数据",num);
        for (; num > 0; num--)
        {
            c = hal.serial(1)->read();
            if (linebuf_len == 0)
            {
                if (c == 1) // ID正确
                {
                    linebuf[linebuf_len++] = c;
                }
            }
            else if (linebuf_len == 1)
            {
                if (c == 3) // 确定是读出的指令
                {
                    linebuf[linebuf_len++] = c;
                }
            }
            else if (linebuf_len == 2)
            {
                if (c == 4) // 读出4个字符
                {
                    linebuf[linebuf_len++] = c;
                    stat = 1; // 开始接受剩下的数据
                    len_date = 6;
                }
            }
            else if (stat == 1 && len_date > 0)
            {
                len_date--;
                linebuf[linebuf_len++] = c;
                if (len_date == 0)
                {
                    stat = 2;
                }
            }
            else if (stat == 2)
            {
                if (linebuf_len == FRAME_LENGTH)
                {
                    uint16_t crc = CRC.Funct_CRC16(linebuf, FRAME_LENGTH - 2);
                    if (crc == ((linebuf[FRAME_LENGTH - 2]) | linebuf[FRAME_LENGTH - 1] << 8))
                    {
                        if (linebuf[0] == 1) // 如果是消防炮消息
                        {
                            address_1 = ((linebuf[3] << 8) | linebuf[4]) / 10;
                            address_2 = ((linebuf[5] << 8) | linebuf[6]) / 10;
                            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Left_Right_pulse:%d", Left_Right_pulse);
                            /* code */
                        }
                        // else if (linebuf[0] == 100) // 读取左边电机转速设定值和当前转速
                        // {
                        //     Set_Left_motor = ((linebuf[3] << 8) | linebuf[4]);
                        //     Read_Left_motor = ((linebuf[5] << 8) | linebuf[6]);
                        //     // gcs().send_text(MAV_SEVERITY_CRITICAL, "Set_Left_motor:%d", (int16_t)Set_Left_motor);
                        //     // gcs().send_text(MAV_SEVERITY_CRITICAL, "Read_Left_motor:%d", (int16_t)Read_Left_motor);
                        // }
                        // else if (linebuf[0] == 101) // 读取右边电机转速设定值和当前转速
                        // {
                        //     Set_Right_motor = ((linebuf[3] << 8) | linebuf[4]);
                        //     Read_Right_motor = ((linebuf[5] << 8) | linebuf[6]);
                        //     // gcs().send_text(MAV_SEVERITY_CRITICAL, "Read_Right_motor:%d", (int16_t)Read_Right_motor);
                        // }

                        // gcs().send_text(MAV_SEVERITY_CRITICAL, "上下的脉冲值为:%d", Up_Down_pulse);
                    }
                    linebuf_len = 0;
                    stat = 0;
                }
            }
            else
            {
                linebuf_len = 0;
                stat = 0;
            }
            // hal.serial(1)->write(c);
        }
    }

    return 0;
}

void FireFight::up_button(uint16_t val)
{
    write_one(0x01, 0x000C, val); // 发送内存地址12,指令1，按键上功能
}

void FireFight::down_button(uint16_t val)
{
    write_one(0x01, 0x000D, val); // 发送内存地址13,指令1，按键上功能
}

void FireFight::left_button(uint16_t val)
{
    write_one(0x01, 0x000E, val); // 发送内存地址14,指令1，按键上功能
}

void FireFight::right_button(uint16_t val)
{
    write_one(0x01, 0x000F, val); // 发送内存地址15,指令1，按键上功能
}

void FireFight::zhu_button(uint16_t val)
{
    write_one(0x01, 0x0010, val); // 发送内存地址16,指令1，按键上功能
}

void FireFight::wu_button(uint16_t val)
{
    write_one(0x01, 0x0011, val); // 发送内存地址17,指令1，按键上功能
}

void FireFight::upanddown_zero() // 上下行程归零
{
    write_two(0x01, 0x000C, 0, 0); // 发送内存地址12,指令1，按键上功能
}

void FireFight::leftandright_zero() // 左右行程归零
{
    write_two(0x01, 0x000E, 0, 0); // 发送内存地址12,指令1，按键上功能
}

void FireFight::wuzhu_zero()
{
    write_two(0x01, 0x0010, 0, 0); // 发送内存地址12,指令1，按键上功能
}

void FireFight::zhu_zero() // 柱归零
{
    write_one(0x01, 0x0010, 0); // 发送内存地址16,指令1，按键上功能
}

void FireFight::wu_zero() // 雾归零
{
    write_one(0x01, 0x0011, 0); // 发送内存地址17,指令1，按键上功能
}

void FireFight::valve_button(uint16_t val) // 具体定义请参考.h文件

{
    write_one(0x01, 0x0013, val);
}

void FireFight::pump_button(uint16_t val)
{
    write_one(0x01, 0x0014, val);
}

void FireFight::Record_button(uint16_t val)
{
    write_one(0x01, 0x0015, val);
}

void FireFight::playback_button(uint16_t val)
{
    write_one(0x01, 0x0016, val);
}


void FireFight::function_fire_fight(uint8_t DT_ms) // 执行周期，传入DT很重要
{
    static uint16_t time_count_ms = 0;
    static uint8_t ignition_lock = 0;
    int16_t action_stop_1 = 0,action_stop_2 = 0;                    // pitch轴标志位
    int16_t action_ignition_1 = 0, action_ignition_2 = 0;           //
    // static uint8_t flag = 0;
    uint16_t under_offset = 1800;
    uint16_t low_offset = 1200;
    // int8_t exp_offset_Up_Down = 0, exp_offset_Left_Right = 0;
    uint16_t stop_ctrl = (hal.rcin->read(4));  //刹车控制
    uint16_t ignition_ctrl_frist = (hal.rcin->read(5));  //一级软件点火控制
    uint16_t ignition_ctrl_second = (hal.rcin->read(11));  // 二级软件点火控制
    // uint16_t LED_ctrl = (hal.rcin->read(6));        //灯控制
    // 起爆逻辑如下：ignition_ctrl_frist、ignition_ctrl_second杆必须在规定时间执行以下操作：
    // 1：ignition_ctrl_frist杆先推到下，再推到上，保持不动，开启一级解保护
    // 2：ignition_ctrl_second 先旋转至最小，再旋转至最大，重复两次，开启二级解保护，启动引爆继电器
    if (abs(ignition_ctrl_frist - 1500) > 100)
    {

        if (time_count_ms > 5000)
        {
            ignition_lock = 0;
            time_count_ms = 0;  //启动失败，请重新进入起爆流程
                // lock_flag = 1; // Push_rod_fan_ID_2 ^ 0x0001  Self_spraying_ID3 ^ 0x0001
            // ((T_8 - 1500) > 0) ? (Push_rod_fan_ID_2 = 1) : (Push_rod_fan_ID_2 = 0);
        }
        else if (time_count_ms <= 5000) //在5秒内进行判断
        {
            if (ignition_ctrl_frist < 1500 && ignition_lock == 0)
            {
                ignition_lock = 1; // 必须初始数值向下
            }
            else if(ignition_ctrl_frist > 1500 && ignition_lock == 1) 
            {
                ignition_lock = 2;
            }
            else if (ignition_ctrl_frist > 1500 && ignition_ctrl_second < low_offset && ignition_lock == 2)
            {
                ignition_lock = 3;
            }
            else if (ignition_ctrl_frist > 1500 && ignition_ctrl_second > under_offset && ignition_lock == 3)
            {
                ignition_lock = 4;
            }
            else if (ignition_ctrl_frist > 1500 && ignition_ctrl_second < low_offset && ignition_lock == 4)
            {
                ignition_lock = 5;
            }
            else if (ignition_ctrl_frist > 1500 && ignition_ctrl_second > under_offset && ignition_lock == 5)
            {
                ignition_lock = 0;  //重置标志位，避免未起爆
                if (action_ignition_2 == 0 && action_ignition_1 == 0) // 如果第一次，则优先使用action1去尝试起爆
                {
                    action_ignition_1 = action_ignition_1 ^ 0x0001;
                }
                else  //启用第二套起爆
                {
                    action_ignition_1 = action_ignition_1 ^ 0x0001;
                    action_ignition_2 = action_ignition_2 ^ 0x0001;
                }
                
            }
            
        }
        if (ignition_lock > 0)
        {
            time_count_ms += DT_ms;
        }

        // {exp_offset_Up_Down = 1} : exp_offset_Up_Down = -1; // 等于1表示上，-1表示向下
    }

    
    if (abs(stop_ctrl - 1500) > 100)
    {
        ((stop_ctrl - 1500) > 0) ? (action_stop_1 = 1, action_stop_2 = 0) : (action_stop_1 = 0, action_stop_2 = 1);
        // {exp_offset_Up_Down = 1} : exp_offset_Up_Down = -1; // 等于1表示上，-1表示向下
    }
    else if (abs(stop_ctrl - 1500) < 100)
    {
        action_stop_1 = 0, action_stop_2 = 0;
    }

    write_six(1, 12, action_stop_1, action_stop_2, action_stop_1, action_stop_2, action_ignition_1, action_ignition_2);
    /* code */
  
}

void FireFight::FireFight_ID2(uint8_t DT_ms) // 执行周期，传入DT很重要
{
    static uint16_t time_count_ms = 0;
    static uint8_t lock_flag = 0;  //当持续拨动某个杠超过1s时候则置位
    static int16_t Push_rod_fan_ID_2 = 0, arm_LED_ID_1 = 0; // pitch轴标志位
    static int16_t Self_spraying_ID3 = 0, ID4 = 0;
    static int16_t ID5 = 0, ID6 = 0;

    uint16_t under_offset = 1700;
    uint16_t low_offset = 1300;
    // int8_t exp_offset_Up_Down = 0, exp_offset_Left_Right = 0;
    uint16_t T_8 = Rc_In[16];
    uint16_t F21 = Rc_In[21];
    uint16_t F5 = Rc_In[19];
    if (abs(T_8 - 1500) > 100 && lock_flag == 0)
    {
        
        if(time_count_ms > 1000)
        {
            lock_flag = 1; // Push_rod_fan_ID_2 ^ 0x0001  Self_spraying_ID3 ^ 0x0001
            ((T_8 - 1500) > 0) ? (Push_rod_fan_ID_2 = 1) : (Push_rod_fan_ID_2 = 0);
        }
        else
        {
            time_count_ms += DT_ms;
        }
        
            
        // {exp_offset_Up_Down = 1} : exp_offset_Up_Down = -1; // 等于1表示上，-1表示向下
    }
    else if (abs(T_8 - 1500) < 100)
    {
        lock_flag = 0;
        time_count_ms = 0;
    }
    // write_two(1,0,exp_offset_Up_Down,exp_offset_Left_Right);

    if (F21 > under_offset)
    {
        Self_spraying_ID3 = 0;
        // write_two(0x01,0x0010,1,0);
    }
    else if ((F21) < low_offset)
    {
        // write_two(0x01,0x0010,0,0);
        Self_spraying_ID3 = 1;
    }

    if (F5 > under_offset)
    {
        arm_LED_ID_1 = 0;
        // write_two(0x01,0x0010,1,0);
    }
    else if ((F5) < low_offset)
    {
        // write_two(0x01,0x0010,0,0);
        arm_LED_ID_1 = 1;
    }
   write_six(2, 12, arm_LED_ID_1, Push_rod_fan_ID_2, Self_spraying_ID3, ID4, ID5, ID6);
    //这里设置初始地址为12,因为方便几个板子间移植
}

void FireFight::FireFight_ID3(uint8_t DT_ms) // 执行周期，传入DT很重要
{
    static uint16_t time_count_ms_T7 = 0, lock_flag_T7 = 0;
    static uint16_t time_count_ms_T8 = 0, lock_flag_T8 = 0;
    static int16_t release_belt_ID_1 = 0, fan_ID_2 = 0; // pitch轴标志位
    static int16_t GPIO_LED_ID_3 = 0, GPIO_LED_ID_4 = 0;   //
    static int16_t ID5 = 0, ID6 = 0;     //

    uint16_t under_offset = 1700;
    uint16_t low_offset = 1300;
    // int8_t exp_offset_Up_Down = 0, exp_offset_Left_Right = 0;
    uint16_t T_7 = Rc_In[15];
    uint16_t T_8 = Rc_In[16];
    uint16_t T_4 = Rc_In[14];
    if (abs(T_7 - 1500) > 100 && lock_flag_T7 == 0)
    {
        if (time_count_ms_T7 > 3000)
        {
            lock_flag_T7 = 1;
            ((T_7 - 1500) > 0) ? (ID5 = 0) : (release_belt_ID_1 = release_belt_ID_1 ^ 0x0001);
        }
        else
        {
            time_count_ms_T7 += DT_ms;
        }
        // {exp_offset_Up_Down = 1} : exp_offset_Up_Down = -1; // 等于1表示上，-1表示向下
    }
    else if (abs(T_7 - 1500) < 100)
    {
        lock_flag_T7 = 0;
        time_count_ms_T7 = 0;
    }

    if (abs(T_8 - 1500) > 100 && lock_flag_T8 == 0)
    {

        if (time_count_ms_T8 > 1000)
        {
            lock_flag_T8 = 1;
            ((T_8 - 1500) > 0) ? (fan_ID_2 = 1) : ( fan_ID_2 = 0);
        }
        else
        {
            time_count_ms_T8 += DT_ms;
        }

    }
    else if (abs(T_8 - 1500) < 100)
    {
        lock_flag_T8 = 0;
        time_count_ms_T8 = 0;
    }

    if ((T_4) > under_offset)
    {
        GPIO_LED_ID_3 = 1, GPIO_LED_ID_4 = 1;
        // write_two(0x01,0x0010,1,0);
    }
    else if ((T_4) < low_offset)
    {
        // write_two(0x01,0x0010,0,0);
        GPIO_LED_ID_3 = 0, GPIO_LED_ID_4 = 0;
    }
    write_six(3, 12, release_belt_ID_1, fan_ID_2, GPIO_LED_ID_3, GPIO_LED_ID_4, ID5, ID6);
    // 这里设置初始地址为12,因为方便几个板子间移植
}
void FireFight::parm_change()
{
    static int16_t last_UD = 0,last_LR = 0,last_BUARD = 0;
    if (last_UD == 0)
        last_UD = UD;
    if (last_LR == 0)
        last_LR = LR;
    if (last_BUARD == 0)
        last_BUARD = BUARD;
    if (last_UD != UD)
    {
        last_UD = UD;
        write_one(1, 2, last_UD);  //写入参数
        write_one(1, 24, 1);  // 保存参数
    }
    if (last_LR != LR)
    {
        last_LR = LR;
        write_one(1, 3, last_LR); // 写入参数
        write_one(1, 24, 1);      // 保存参数
    }
    if (last_BUARD != BUARD)
    {
        last_BUARD = BUARD;
        write_one(1, 23, last_BUARD); // 写入参数
        write_one(1, 24, 1);      // 保存参数
    }
}

const AP_Param::GroupInfo FireFight::var_info[] =
{
        // @Param: LR
        // @DisplayName: STA_CURRENT_LR
        // @Description: 这个是左右电机堵转保护数值.
        // @Range:0 5000
        // @User: Standard
        AP_GROUPINFO("LR", 0, FireFight, LR, 3000),

        // @Param:UD
        // @DisplayName: STA_CURRENT_UD
        // @Description: 这个是左右电机堵转保护数值.
        // @Range:0 5000
        // @User:Standard
        // 注意这里是增加的名字
        AP_GROUPINFO("UD", 1, FireFight, UD, 3000),
        // @Param:BUARD
        // @DisplayName: STA_CURRENT_BUARD
        // @Description: 这个是消防炮波特率数值.
        // @Range:0 5000
        // @User:Standard
        // 注意这里是增加的名字
        AP_GROUPINFO("BUARD", 2, FireFight, BUARD, 2560),
        AP_GROUPEND
};
