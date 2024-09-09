#include "Explosion_gases.h"
#include <GCS_MAVLink/GCS.h> //地面站
void Explosion_gases::get_Thermal_imaging()  //热成像信息读取
{
    read(Thermal_ID, 0x0100, 8);
    Data_Receive_Prepare();
}

void Explosion_gases::get_Bms_Info() // BMS信息读取
{
    read_Bms(3);   //读取电池基本信息
    // Data_Receive_Prepare();
}

void Explosion_gases::get_Temp() // 防爆气体信息读取
{
    read(Temp_ID, 0, 2);
    // Data_Receive_Prepare();
}

void Explosion_gases::read_Explosion_gasese()  //防爆气体信息读取
{
    read(Gases_ID, 0, 7);
    // Data_Receive_Prepare();
}

void Explosion_gases::get_Broa_info() // 压力传感器信息
{
    read(Broa_ID, 4, 1);
    // Data_Receive_Prepare();
}

void Explosion_gases::read_Bms(uint16_t comand_ID) // 只需要填写寄存器ID和寄存器个数
{
    uint8_t data_to_send[10];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    data_to_send[cnt++] = 0XDD;       // Bms帧头信息DD A5
    data_to_send[cnt++] = 0XA5;       
    data_to_send[cnt++] = comand_ID;  // 读取的功能码为03为基本信息  04 单节电心电压
    data_to_send[cnt++] = 0;
    crc = (~(data_to_send[2] + data_to_send[3])) + 1; // ID+长度+数据取反之后加一
    data_to_send[cnt++] = BYTE1(crc);
    data_to_send[cnt++] = BYTE0(crc);
    data_to_send[cnt++] = 0x77;
    hal.serial(2)->write(data_to_send, cnt);
}

void Explosion_gases::read(uint8_t address_ID, uint16_t reg_adress, uint16_t reg_num) // 只需要填写寄存器ID和寄存器个数
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
    hal.serial(2)->write(data_to_send, cnt);
}


void Explosion_gases::Data_Receive_Anl_Task(uint8_t *data_buf, uint16_t num)
{
    if(data_buf[0]==Gases_ID)
    {
        gases.humidity = (data_buf[3] << 8) + data_buf[4]; // 00FE H(十六进制)=254=>湿度=25.4%RH  地址0
        gases.temp = (data_buf[5] << 8) + data_buf[6];     // 00AF H(十六进制)=175=>温度=17.5℃   地址1
        gases.Co2 = (data_buf[13] << 8) + data_buf[14];     // 00BD H(十六进制)=189=>CO=18.9ppm  地址5
        gases.NH3 = (data_buf[15] << 8) + data_buf[16];     // 00BD H(十六进制)=189=>CO=18.9ppm  地址6
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "gases.humidity:%d", gases.humidity);
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "gases.temp:%d", gases.temp);
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "gases.Co2:%d", gases.Co2);
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "gases.NH3:%d", gases.NH3);
    }
    else if (data_buf[0] == Thermal_ID)
    {
        Ther_ima.x = (data_buf[3] << 8) + data_buf[4];
        Ther_ima.y = (data_buf[5] << 8) + data_buf[6];
        Ther_ima.temp = (data_buf[9] << 8) + data_buf[10];
    }
    else if (data_buf[0] == Bms_ID)
    {
        Bms_info.All_Vol = ((data_buf[4] << 8) + data_buf[5])*0.1;
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "Bms_info.All_Vol :%d", Bms_info.All_Vol);
        Bms_info.I_cur = ((data_buf[6] << 8) + data_buf[7]);
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "Bms_info.I_cur :%d", Bms_info.I_cur);
        Bms_info.RSOC =  data_buf[23];
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "Bms_info.RSOC :%d", Bms_info.RSOC);
        Bms_info.temp = (((data_buf[27] << 8) + data_buf[28]) -2713);
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "Bms_info.temp :%d", Bms_info.temp);
    }
    else if (data_buf[0] == Broa_ID) //
    {
        gases.waterpress = ((data_buf[3] << 8) + data_buf[4]);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "gases.waterpress :%d", gases.waterpress);
    }
    else if(data_buf[0] == Temp_ID)
    {
        gases.T_temp = ((data_buf[3] << 8) + data_buf[4]);
        gases.T_humidity = ((data_buf[5] << 8) + data_buf[6]);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "gases.T_temp :%d", gases.T_temp);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "gases.T_humidity :%d", gases.T_humidity);
    }
}

void Explosion_gases::Data_Receive_Prepare()   //使用串口4用于传感器数据采集
{

    uint8_t num = hal.serial(2)->available(); // 读取串口有多少个数据

    // gcs().send_text(MAV_SEVERITY_CRITICAL, "num:%d", num);
    uint8_t c;
    static uint8_t stat = 0, rece_len = 0, data_len = 0;
    static uint8_t data_buff[255];
    while (num > 0)
    {
        
        c = hal.serial(2)->read();
        num--;
        hal.serial(7)->write(&c, 1); // 测试接受到的数据用的
        if (stat == 0)
        {
            if (c == 0XDD || c == Thermal_ID || c == Gases_ID || c == Broa_ID || c == Temp_ID) // 如果等于发射器发送的帧头0x01：表示气体传感器
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

                /* code */
            }
            else
            {
                rece_len = 0;
                stat = 0;
            }
        }
        else if (stat == 2) // 根据上面的功能码进行接下来的数据接收
        {
            data_buff[rece_len++] = c; // 这里已经接收完一个，故下面的需要减掉1
            if (data_buff[0] == 0XDD && data_buff[2] == 0)
            {
                stat = 4;
            }
            else
            {
                stat++;
                switch (data_buff[1])
                {
                case 0x03:            // 如果是读取寄存器
                    data_len = c + 2; // 如果功能码是0x03，表示后面还有有效字节+2CRC
                    break;
                case 0x0F: // 写单个寄存器
                    // data_len = 10; // 如果功能码是0x03仅需接收2个CRC+8个数据位+1个字节数即可完成接受
                    break;
                case 0x10:
                    // data_len = ((data_buff[4] << 8) + data_buff[5]) * 2 + 2;
                    break;
                default:
                    rece_len = 0;
                    stat = 0; // 当不是上面任何的数据的话，不进行后面的接收
                    break;
                }
            }
            
        }
        else if (stat == 3 && data_len > 0)
        {
            data_len--;
            data_buff[rece_len++] = c;
            if (data_len == 0) // 代表接收完毕
            {
                if (data_buff[0] == 0XDD && data_buff[rece_len - 1] == 0x77)
                {
                    uint16_t crc = 0;
                    for (uint8_t i = 2; i <= data_buff[3]+3;i++) // 这里进行和校验取反 +1
                    {
                        crc += data_buff[i];
                    }
                    
                    crc = (~crc) + 1;
                    // gcs().send_text(MAV_SEVERITY_CRITICAL, "crc:%d", crc);
                    if (crc == ((data_buff[rece_len - 3] << 8) | data_buff[rece_len - 2]))  //CRC倒数第二、第三位，高位在前低位后
                    {
                        // hal.serial(7)->write(data_buff,rece_len);
                        Data_Receive_Anl_Task(data_buff, rece_len);
                        /* code */
                    }
                }
                else
                {
                    uint16_t crc = CRC.Funct_CRC16(data_buff, rece_len - 2);
                    if (crc == ((data_buff[rece_len - 2]) | data_buff[rece_len - 1] << 8)) // 表示校验通过可以进行解析
                    {
                        Data_Receive_Anl_Task(data_buff, rece_len);
                    }

                }
                rece_len = 0;
                stat = 0;
            }
        }
        else if(stat == 4)
        {
            data_buff[rece_len++] = c;
            data_len = c + 3;
            stat = 3;
            // gcs().send_text(MAV_SEVERITY_CRITICAL, "data_len:%d", data_len);
        }
        else
        {
            rece_len = 0;
            stat = 0;
        }
    }
}