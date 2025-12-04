--This is a script for reading gas sensors.
local DEBUG = false -- Set to true to enable debug messages
local gas_type = {"NULL","AR","ASH3","B2H6","BR2",  --5
                 "CO","CO2","COCL2","CH20","CH202", --10
                "CH3BR","CH4","CH40","CH4S","CH5N", --15    
                "CH60","CIC","CL2","CLO2","C2CL4",  --20
                "C2HCL3","C2H2","C2H3CL","C2H","C2H40",--25
                "C2H60","C3H3N","C3H60","C3H8","C3H80",--30
                "C4H802","C4H8S","C4H10","C4H100","C5H12",--35
                "C6H6","C6H6S","C6H12","C6H14","C7H8",--40
                "C7H16","C8H8","C8H10","C8H18","CS2",--45
                "EX","ETO","F2","FX","GEH4",--50
                "H2","H2O2","H2S","HCL","HCN",--55
                "HBR","HE","HF","I2","NO",
                "N02","NOX","NF3","NH3","N2",
                "N20","N2H4","O2","O3","PH3",
                "PID","P2O5","SO2","SO2F2","SIH4",
                "SIF4","SF6","THT","TVOC","VOC",
                "VOCS","SO3","NMHC","温度","湿度",
                "风速","风向","降雨量","噪音"}
-- find the port first (0) scripting port port instance
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

-- 解析Modbus响应数据
local function parse_response(buffer_rx)
    if not buffer_rx or #buffer_rx < 5 then
        return nil
    end
    local DuiGasLocalAddrss =buffer_rx[1];                              --模组本地地址
    local DuiGasUnit =(buffer_rx[4]&0xf0)>>4; 
    local DuiGasUnit =DuiGasUnit/2;                                             --与单位显示数组对照
    local DuiGasDecimalDigits=((buffer_rx[4]&0x0f)>>2);                 --与转换小数点对照
    local DuiGasIform =buffer_rx[20];                                   --气体类型
    buffer_rx[5]=0x00;                                            --保留数据位
    local DucNowchroma =buffer_rx[6];
    local DucNowchroma = ((DucNowchroma<<8)&0xff00)+buffer_rx[7];       --实时浓度值					
    local DuiGasLowAlarm =buffer_rx[8];
    local DuiGasLowAlarm = ((DuiGasLowAlarm<<8)&0xff00)+buffer_rx[9];   --低报浓度值低8位	
    local DuiGasHighAlarm =buffer_rx[10];
    local DuiGasHighAlarm = ((DuiGasHighAlarm<<8)&0xff00)+buffer_rx[11];--高报浓度值低8位	
    local DuiCGasChroma =buffer_rx[12];
    local DuiCGasChroma = ((DuiCGasChroma<<8)&0xff00)+buffer_rx[13];    --全量程浓度值
    buffer_rx[14]=0x00;                                           --数据保留位
    local GasWorkState=buffer_rx[15];                                   --工作状态
    local DuiGas_Adc =buffer_rx[16];			
    local DuiGas_Adc = ((DuiGas_Adc<<8)&0xff00)+buffer_rx[17]; 
    if DEBUG then
        gcs:send_text(0,string.format("ID:%02X",DuiGasLocalAddrss))--当前AD值	
        gcs:send_text(0,string.format("气体类型:%s",DuiGasIform))   
        gcs:send_text(0,string.format("气体数值:%d",DucNowchroma))   
    end
    gcs:send_named_float(gas_type[DuiGasIform+1],DucNowchroma);  --lua脚本原因，需要额外+1
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
        if stat == 0 and byte >= 0x01 and byte <= 0x04 then -- 检查从站地址
            response[1] = byte
            cur_count = 1;
            stat = 1;
        elseif stat == 1 and byte == 0x03 then
            response[2] = byte
            cur_count = 2;
            stat = 2;
        elseif  stat == 2 and byte == 0x14 then
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
local address = 1;
local function update()
    
    -- 发送命令
    send_modbus_command(address)
    address = address + 1;
    if address >= 5 then
        address = 1;
    end
    
    -- 等待并接收响应
    local buffer_rx = receive_modbus_response()
    

    
    -- 1秒后再次执行
    return update, 1000
end
-- 启动主循环
return update()