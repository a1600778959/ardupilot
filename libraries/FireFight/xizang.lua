--This is a script for reading gas sensors.
local DEBUG = true -- Set to true to enable debug messages
-- 协议配置
local MULTICAST_ADDR = "224.0.0.22"
local TELEMETRY_PORT = 7013
local FRAME_HEADER1 = 0xA5 
local FRAME_HEADER2 = 0x5A
local FRAME_TAIL = (0xAA)
local FRAME_CATEGORY = 0x20 -- 遥测帧
local EQUIPMENT_TYPE = 0x1401 -- 设备类型
local NODE_ID = 0x00 -- 节点编号
local FRAME_COUNTER = 0x00 -- 帧计数器
local EQUIPMENT_ID = 0x0000 -- 装备ID
local CRC16Tab = {
0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0
};
local port = serial:find_serial(0)
if not port then
    gcs:send_text(0, "No Scripting port Port")
    return
end

-- begin the port port
port:begin(9600)
port:set_flow_control(0)
-- Modbus读取命令
local modbus_cmd = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A,0x00,0x00}
local TIMEOUT_MS = 100;   
-- 计算CRC校验
local function calculate_crc(data)
    local crc = 0x0000
    for i = 1, #data do
        local byte = data[i]
        crc = (crc << 8) ~ CRC16Tab[(((crc >> 8) ~ byte)&0x00ff)+1]
    end
    return crc
end


-- 数据打包
local function pack_data()
    local data = {}
    local length_current = 1;
    local roll = math.floor(math.deg(ahrs:get_roll())*10)
    local pitch = math.floor(math.deg(ahrs:get_pitch())*10)
    local yaw = math.floor(math.deg(ahrs:get_yaw())*10)

    local gps_loc = ahrs:get_position()
    if gps_loc then
        --帧头 UINT16
        data[1] = FRAME_HEADER1
        data[2] = FRAME_HEADER2
        -- 发送用户ID UINT32
        data[3] = 0x00 -- 0x00固定值
        data[4] = 0x00 -- 0x00固定值
        data[5] = 0x00 -- 0x00固定值
        data[6] = 0x00 -- 0x00固定值
        -- 接收用户ID UINT32
        data[7] = 0x00 -- 0x00固定值
        data[8] = 0x00 -- 0x00固定值
        data[9] = 0x00 -- 0x00固定值
        data[10] = 0x00 -- 0x00固定值
        -- 协议版本号 UINT8
        data[11] = 0x41 -- 0x41固定值
        -- 帧类别 UINT8
        data[12] = FRAME_CATEGORY -- 0x20固定值
        -- 帧长度（动态计算） UINT8
        local length_pos = #data + 1
        data[13] = 0 -- 占位，稍后更新 
        -- 帧计数器 UINT8
        data[14] = FRAME_COUNTER -- 0x00固定值
        -- table.insert(data, string.char(FRAME_COUNTER))
        -- 节点编号 0 UINT8
        data[15] = NODE_ID -- 0x00固定值
        -- table.insert(data, string.char(NODE_ID))
        -- 装备类型 UINT16
        -- gcs:send_text(0,string.format("EQUIPMENT_TYPE:%d",EQUIPMENT_TYPE % 0x100))--当前AD值	
        data[16] = (EQUIPMENT_TYPE % 0x100)&0Xff
        data[17] = math.floor(EQUIPMENT_TYPE / 0x100)&0xff
        -- 装备ID UINT16
        data[18] = (EQUIPMENT_ID % 0x100)&0xff
        data[19] = (math.floor(EQUIPMENT_ID / 0x100))&0xff
        -- 消息类型 UINT8
        data[20] = 0x10 -- 0x10固定值
        -- 消息ID UINT16
        data[21] = 0x00-- 0x00固定值
        data[22] = 0x20 -- 0x20固定值
        -- 编码类型UINT8
        data[23] = 0x00 -- 0x00固定值
        -- table.insert(data, 0x00) -- 0x00固定值
        -- 经度UINT64
        local lon = gps_loc:lng()
        gcs:send_text(0,string.format("lon:%s",tostring(lon)))--当前AD值	
        for i=1,8 do
            data[24 + i - 1] = (lon % 0x100)&0xff
            lon = math.floor(lon / 0x100)
        end    
        -- 纬度UINT64
        local lon = gps_loc:lat()
        gcs:send_text(0,string.format("lat:%s",tostring(lon)))--当前AD值	
        for i=1,8 do
            data[32 + i - 1] = (lon % 0x100)&0xff
            lon = math.floor(lon / 0x100)
        end        
        -- 海拔高度UINT32
        local lon = gps_loc:alt()
        gcs:send_text(0,string.format("alt:%s",tostring(lon)))--当前AD值	
        for i=1,4 do
            data[40 + i - 1] = (lon % 0x100)&0xff
            lon = math.floor(lon / 0x100)
        end  
        -- 离地高度UINT32
        local lon = gps_loc:alt()
        for i=1,4 do
            data[44 + i - 1] = (lon % 0x100)&0xff
            lon = math.floor(lon / 0x100)
        end  
        -- 装备气压高度UINT32
        local lon = math.floor(baro:get_altitude()*100)
        gcs:send_text(0,string.format("altitude:%s",tostring(lon)))--当前AD值	
        for i=1,4 do
            data[48 + i - 1] = (lon % 0x100)&0xff
            lon = math.floor(lon / 0x100)
        end  
        -- 地理坐标系UINT8
        data[52] = 0x01 -- 0x01固定值 WGS-84
        -- 装备俯仰角UINT16
        -- gcs:send_text(0,string.format("roll:%02X",math.floor(roll / 0x100)))--当前AD值	        
        data[53] = ((roll % 0x100)&0xff)
        data[54] = ((roll >> 8)&0Xff)
        -- 装备横滚角UINT16
        data[55] = ((pitch % 0x100)&0xff)
        data[56] = ((pitch >> 8)&0xff)
        -- 装备偏航角UINT16
        data[57] = ((yaw % 0x100)&0xff)
        data[58] = ((yaw >> 8)&0xff)
        -- 装备指示空速UINT16
        
        local v_speed = math.floor(gps:ground_speed(0) * 100) -- 转换为厘米每秒
        data[59] = ((v_speed % 0x100)&0xff)
        data[60] = ((v_speed >> 8)&0xff)
        -- 装备地速UINT16
        data[61] = ((v_speed % 0x100)&0xff)
        data[62] = ((v_speed >> 8)&0xff)
        -- 风速UINT16
        data[63] = 0x00-- 0x00固定值
        data[64] = 0x00 -- 0x00固定值
        -- 风向UINT16
        data[65] = 0x00 -- 0x00固定值
        data[66] = 0x00 -- 0x00固定值
        -- 剩余油量 UINT16
        data[67] = 0x00 -- 0x00固定值
        data[68] = 0x00 -- 0x00固定值
        -- 装备剩余电量 UINT32
        local battery_re = math.floor(battery:pack_capacity_mah(0))       
        for i=1,4 do
            data[69 + i - 1] = (battery_re % 0x100)&0xff
            battery_re = math.floor(battery_re / 0x100)
        end          
     -- 电量百分比
        -- 装备状态 UINT8 0x00：在线正常状态；
        --0x01：在线故障状态；
        --0x02：在线警告状态；
        --0x03：离线状态。FRAME_COUNTER
        data[73] = 0x00 -- 0x00正常状态
        -- 标识符 装备属性 0x2001 光电载荷状态 0x2002  UINT16
        data[74] = 0x02 -- 0x2002固定
        data[75] = 0x20 -- 0x2002固定
        -- 状态 UINT8
        data[76] = 0x00 -- 0x00正常状态FRAME_COUNTER
        -- 光电载荷翻滚 INT16
        data[77] = 0x00 -- 0x00固定值
        data[78] = 0x00 -- 0x00固定值
        -- 光电载荷俯仰状态 INT16
        data[79] = 0x00 -- 0x00固定值
        data[80] = 0x00 -- 0x00固定值
        -- 光电载荷方位角 INT16
        data[81] = 0x00 -- 0x00固定值
        data[82] = 0x00 -- 0x00固定值
        -- 光电载荷水平视场 UINT16
        data[83] = 0x00 -- 光电载荷水平视场
        data[84] = 0x00
        -- 光电载荷垂直视场 UINT16
        data[85] = 0x00-- 光电载荷垂直视场\
        data[86] = 0x00
        -- 镜头最小焦距 UINT16
        data[87] = 0x00 -- 镜头最小焦距
        data[88] = 0x00 -- 镜头最小焦距
        -- 镜头最大焦距 UINT16
        data[89] = 0x00 -- 镜头最大焦距
        data[90] = 0x00 -- 镜头最大焦距
        --像元尺寸 UINT16
        data[91] = 0x00 -- 像元尺寸
        data[92] = 0x00 -- 像元尺寸
        -- 计算帧长度
        local length = #data - length_pos
        data[length_pos] = (length) -- 更新帧长度
        -- 计算CRC校验
        local crc = calculate_crc(data)
        data[93] = (crc % 0x100)&0xff -- CRC低字节
        data[94] = (crc>>8)&0xff -- CRC高字节
        -- 添加帧尾
        data[95] = FRAME_TAIL -- 帧尾
        -- gcs:send_text(0, string.format("date:%s",table.concat(data, " ")))
        -- 发送数据        
        for i = 1, #data do
            port:write((data[i]));
        end
        FRAME_COUNTER = (FRAME_COUNTER + 1) % 256 -- 更新帧计数器   
    else
        if DEBUG then
            gcs:send_text(0, "No GPS data")
        end
    end

end


-- 主循环
local address = 1;
local function update()
    
    -- 发送命令
    pack_data()
    -- send_modbus_command(address)
    -- address = address + 1;
    -- if address >= 5 then
    --     address = 1;
    -- end
    
    -- -- 等待并接收响应
    -- local buffer_rx = receive_modbus_response()
    

    
    -- 1秒后再次执行
    return update, 1000
end
-- 启动主循环
return update()