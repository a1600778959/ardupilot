--This is a script for reading gas sensors.
local DEBUG = false -- Set to true to enable debug messages
-- find the port first (0) scripting port port instance
local PARAM_TABLE_KEY = 2
local PARAM_TABLE_PREFIX = "RDGAS_"
local PARAM_TABLE_SIZE = 2

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format("RDGAS: could not find %s parameter", name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("RDGAS: could not add param %s", name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, PARAM_TABLE_SIZE), "RDGAS: could not add param table")

-- Load CAN driver with a buffer size of 20

local ID = bind_add_param('ID', 1, 32)
local CON_RC = bind_add_param('CON_RC', 2, 10)


local port = serial:find_serial(0)
if not port then
    gcs:send_text(0, "No Scripting port Port")
    return
end

-- begin the port port
port:begin(9600)
port:set_flow_control(0)
-- Modbus读取命令
local modbus_cmd = {0x20, 0x03, 0x00, 0x00, 0x00, 0x28,0x00,0x00}
local gas_cmd = {0x20, 0x06, 0x00, 0x2E, 0x00, 0x01,0x00,0x00}  --打开气泵

local TIMEOUT_MS = 100;  
local base ={
    temp = 0,  -- 温度
    humidity = 0,  -- 湿度
    PM2_5 = 0,  -- PM2.5
    PM10 = 0,  -- PM1
    wind_velocity = 0,  -- 风速
    wind_direction = 0,  -- 风向
    pressure = 0,  -- 气压
    nuclear_radiation = 0,  -- 核辐射
    CO = 0,  -- 一氧化碳
    CO2 = 0,  -- 二氧化碳
    N02 = 0,  -- 二氧化氮
    SO2 = 0,  -- 二氧化硫
    NH3 = 0,  -- 氨
    H2S = 0,  -- 硫化氢
    O2 = 0,  -- 氧气
    O3 = 0,  -- 臭氧
    HCN = 0,  -- 氰化氢
    CH4 = 0,  -- 甲烷
    combustible_gas = 0,  -- 可燃气体
    VOCs = 0,  -- 总挥发性有机物
} 
-- 计算CRC校验
local function calculate_crc(data)
    local crc = 0xFFFF
    for i = 1, #data-2 do
        crc = crc ~ data[i]
        for j = 1, 8 do
            if crc & 0x0001 ~= 0 then
                crc = crc>>1
                crc = crc~0xA001
            else
                crc = crc >> 1
            end
        end
    end
    return crc&0xFF, crc>> 8
end

-- 发送Modbus命令
local function send_gascmd_command(address,cmd)

    gas_cmd[1] = address;
    gas_cmd[6] = cmd;
    local crc_low, crc_high = calculate_crc(
        
    
    )
    gas_cmd[#gas_cmd] = crc_high
    gas_cmd[#gas_cmd-1] = crc_low
    
    -- 发送命令
    for i = 1, #gas_cmd do
        port:write(gas_cmd[i])
    end
    
    if DEBUG then
        gcs:send_text(0, "Gascmd: 命令已发送")
    end
end


-- 发送Modbus命令
local function send_modbus_command(address)

    modbus_cmd[1] = address;
    local crc_low, crc_high = calculate_crc(modbus_cmd)
    modbus_cmd[#modbus_cmd] = crc_high
    modbus_cmd[#modbus_cmd-1] = crc_low
    
    -- 发送命令
    for i = 1, #modbus_cmd do
        port:write(modbus_cmd[i])
    end
    
    if DEBUG then
        gcs:send_text(0, "Modbus: 命令已发送")
    end
end

-- 解析Modbus响应数据 s
local function parse_response(buffer_rx)
    local temp = 0;
    if not buffer_rx or #buffer_rx < 5 then
        return nil
    end                            --模组本地地址
    temp = ((buffer_rx[4]<<24) + (buffer_rx[5]<<16) + (buffer_rx[6]<<8) + buffer_rx[7])*0.1;  --0.1°C
    base.temp  = temp;
    temp = ((buffer_rx[8]<<24) + (buffer_rx[9]<<16) + (buffer_rx[10]<<8) + buffer_rx[11])*0.1; --0.1%RH
    base.humidity = temp;
    temp = ((buffer_rx[12]<<24) + (buffer_rx[13]<<16) + (buffer_rx[14]<<8) + buffer_rx[15]);  --1ug/m3
    base.PM2_5 = temp;
    temp = ((buffer_rx[16]<<24) + (buffer_rx[17]<<16) + (buffer_rx[18]<<8) + buffer_rx[19]);  --1ug/m3
    base.PM10 = temp;
    temp = ((buffer_rx[20]<<24) + (buffer_rx[21]<<16) + (buffer_rx[22]<<8) + buffer_rx[23])*0.1; --0.1m/s
    base.wind_velocity = temp;
    temp = ((buffer_rx[24]<<24) + (buffer_rx[25]<<16) + (buffer_rx[26]<<8) + buffer_rx[27]);  --1°
    base.wind_direction = temp;
    temp = ((buffer_rx[28]<<24) + (buffer_rx[29]<<16) + (buffer_rx[30]<<8) + buffer_rx[31])*0.001; --0.001KPa
    base.pressure = temp;
    temp = ((buffer_rx[32]<<24) + (buffer_rx[33]<<16) + (buffer_rx[34]<<8) + buffer_rx[35])*0.001; --0.001usv
    base.nuclear_radiation = temp;
    temp = ((buffer_rx[36]<<24) + (buffer_rx[37]<<16) + (buffer_rx[38]<<8) + buffer_rx[39]);   -- 1PPM
    base.CO = temp;
    temp = ((buffer_rx[40]<<24) + (buffer_rx[41]<<16) + (buffer_rx[42]<<8) + buffer_rx[43])*0.0001; --0.0001%VOL
    base.CO2 = temp;
    temp = ((buffer_rx[44]<<24) + (buffer_rx[45]<<16) + (buffer_rx[46]<<8) + buffer_rx[47])*0.1;  --0.1PPM
    base.N02 = temp;
    temp = ((buffer_rx[48]<<24) + (buffer_rx[49]<<16) + (buffer_rx[50]<<8) + buffer_rx[51])*0.1;  --0.1PPM
    base.SO2 = temp;
    temp = ((buffer_rx[52]<<24) + (buffer_rx[53]<<16) + (buffer_rx[54]<<8) + buffer_rx[55])*0.1;  --0.1PPM
    base.NH3 = temp;
    temp = ((buffer_rx[56]<<24) + (buffer_rx[57]<<16) + (buffer_rx[58]<<8) + buffer_rx[59])*0.1;  --0.1PPM
    base.H2S = temp;
    temp = ((buffer_rx[60]<<24) + (buffer_rx[61]<<16) + (buffer_rx[62]<<8) + buffer_rx[63])*0.01; --0.01%VOL
    base.O2 = temp;
    temp = ((buffer_rx[64]<<24) + (buffer_rx[65]<<16) + (buffer_rx[66]<<8) + buffer_rx[67])*0.1;  --0.1PPM
    base.O3 = temp;
    temp = ((buffer_rx[68]<<24) + (buffer_rx[69]<<16) + (buffer_rx[70]<<8) + buffer_rx[71])*0.01;  --0.01PPM
    base.HCN = temp;
    temp = ((buffer_rx[72]<<24) + (buffer_rx[73]<<16) + (buffer_rx[74]<<8) + buffer_rx[75])*0.01;  --0.01%LEL
    base.CH4 = temp;
    temp = ((buffer_rx[76]<<24) + (buffer_rx[77]<<16) + (buffer_rx[78]<<8) + buffer_rx[79])*0.01;  --0.01%LEL
    base.combustible_gas = temp;
    temp = ((buffer_rx[80]<<24) + (buffer_rx[81]<<16) + (buffer_rx[82]<<8) + buffer_rx[83])*0.001;  --0.001ppm
    base.VOCs = temp;
    if DEBUG then
        -- gcs:send_text(0,string.format("ID:%02X",DuiGasLocalAddrss))--当前AD值	
        -- gcs:send_text(0,string.format("气体类型:%s",DuiGasIform))   
        -- gcs:send_text(0,string.format("气体数值:%d",DucNowchroma))   
    end
    gcs:send_named_float("temperature",base.temp);  
    gcs:send_named_float("humidity",base.humidity);  
    gcs:send_named_float("PM2.5",base.PM2_5);  
    gcs:send_named_float("PM10",base.PM10);  
    gcs:send_named_float("wind_velocity",base.wind_velocity);  
    gcs:send_named_float("wind_direction",base.wind_direction);  
    gcs:send_named_float("pressure",base.pressure);  
    gcs:send_named_float("nuclear_radiation",base.nuclear_radiation);  
    gcs:send_named_float("CO",base.CO);  
    gcs:send_named_float("CO2",base.CO2);  
    gcs:send_named_float("N02",base.N02);  
    gcs:send_named_float("SO2",base.SO2);  
    gcs:send_named_float("NH3",base.NH3);  
    gcs:send_named_float("H2S",base.H2S);  
    gcs:send_named_float("O2",base.O2);  
    gcs:send_named_float("O3",base.O3);  
    gcs:send_named_float("HCN",base.HCN);  
    gcs:send_named_float("CH4",base.CH4);  
    gcs:send_named_float("combustible_gas",base.combustible_gas);  
    gcs:send_named_float("VOCs",base.VOCs);  
end


local function shallow_copy(t)
    return {table.unpack(t)}
end
-- 接收Modbus响应
local stat = 0;
local response = {};   --因为需要保存响应数据，所以使用一个全局表来存储
local data_length = 0;
local cur_count = 0;
local function receive_modbus_response()


    local n_bytes = port:available()
    while n_bytes > 0 do
        n_bytes = n_bytes - 1;

        local byte = port:read()
        -- gcs:send_text(0, string.format("n_bytes: %02x", byte))
        if stat == 0 and byte == 0x20 then -- 检查从站地址
            response[1] = byte
            cur_count = 1;
            stat = 1;
        elseif stat == 1 and byte == 0x03 then
            response[2] = byte
            cur_count = 2;
            stat = 2;
        elseif  stat == 2 and byte == 0x50 then
            response[3] = byte  --数据长度
            data_length = response[3] + 2;
            cur_count   = 3;
            stat = 3;
        elseif stat == 3 then
            if data_length > 1 then
                data_length = data_length - 1;
                cur_count = cur_count + 1;
                response[cur_count] = byte
            else
                cur_count = cur_count + 1;   --获取最后一个数字
                response[cur_count] = byte
                local copy = shallow_copy(response) -- 复制响应数据
                response = {} -- 清空响应数据
                stat = 0;
                cur_count = 0;
                data_length = 0;
                local crc_low, crc_high = calculate_crc({table.unpack(copy, 1, #copy)})
                -- gcs:send_text(0, string.format("Modbus: 收到响应 %d 字节", #copy))
                -- gcs:send_text(0, string.format("response: CRC校验值 = %02X %02X", copy[#copy-1], copy[#copy]))
                -- gcs:send_text(0, string.format("Modbus: CRC校验值 = %02X %02X", crc_low, crc_high))
                if crc_low == copy[#copy-1] and crc_high == copy[#copy] then
                    if DEBUG then
                        gcs:send_text(0, "Modbus: 收到有效响应")
                    end
                    parse_response(copy) -- 解析响应数据
                else

                    if DEBUG then
                        gcs:send_text(0, "Modbus: CRC校验失败")
                    end
                end
                    end
        else
            -- gcs:send_text(0, string.format("Modbus:校验帧头为:%02X %d", byte,stat))
            response = {} -- 清空响应数据
            stat = 0;
            cur_count = 0;
            data_length = 0;
        end
    end
    


    
    return nil
end
-- 主循环
local send_flag = 0;
local function update()
    local address = ID:get();
    local con_rc = rc:get_pwm(CON_RC:get());

    if send_flag == 0 then
        if con_rc and con_rc > 1650 then
            -- 发送命令
            send_gascmd_command(address,1)  -- 打开气泵
        else
            send_gascmd_command(address,0)  -- 关闭气泵
        end
        send_flag = 1;
    else
        -- 发送命令
        send_modbus_command(address)
        send_flag = 0;
    end

    -- address = address + 1;
    -- if address >= 5 then
    --     address = 1;
    -- end
    
    -- 等待并接收响应
    local buffer_rx = receive_modbus_response()
    

    
    -- 1秒后再次执行
    return update, 1000
end
-- 启动主循环
return update()