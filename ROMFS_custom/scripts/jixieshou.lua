-- 机械臂增量控制脚本
local can_bus = 1          -- CAN总线编号
local arm_speed = 25       -- 运动速度百分比
local deadzone = 50        -- 遥控器死区阈值
local step_scaling = {     -- 增量步长比例W
    x = 1000,    -- 0.001mm/step
    y = 1000,
    z = 1000,
    rx = 500,    -- 0.001°/step
    ry = 500,
    rz = 500,
    j1 = 500,
    j2 = 500,
    j3 = 500,
    j4 = 500,
    j5 = 500,
    j6 = 500,
    jiazi = 500,
}


-- 当前位姿状态
local current_pose = {
    x = 99999, y = 99999, z = 99999,
    rx = 99999, ry = 99999, rz = 99999,
    j1 = -865, j2 = 0, j3 = -2182, j4 = -1091, j5 = 23271 , j6 = -60497,
    jiazi = 0
}
local driver = CAN:get_device(5)
-- 初始化函数
function update()
    -- 初始化CAN总线
    -- gcs:send_text(0, "Distance:"..tostring(distance).." Instance:"..tostring(instance))
    if arming:is_armed() then
        enable_motors()
        set_can_mode()
    end

    -- 主循环
    if not arming:is_armed() then
        
        -- emergency_stop()
        return update, 20
    end

    if rc:get_pwm(9) < 1800 then
        return update,20
    end
    -- 解析反馈数据
    -- if not process_can_feedback() then
    --     gcs:send_text(0, "CAN feedback error")
    --     return update, 20
    -- end

    -- if current_pose.x == 99999 and current_pose.y ==99999 and current_pose.z ==99999 and
    --    current_pose.rx == 99999 and current_pose.ry ==99999 and current_pose.rz ==99999 and
    --    current_pose.j1 == 99999 and current_pose.j2 ==99999 and current_pose.j3 ==99999 and
    --    current_pose.j4 == 99999 and current_pose.j5 ==99999 and current_pose.j6 ==99999 then
    --     gcs:send_text(0, "can't find the current pose")--查看该通道数值是否被读取到
    --     return update, 20
    -- end
    -- 读取遥控器增量输入
    local delta = get_rc_delta()

    -- 更新目标位姿
    update_target_pose(delta)

    -- 发送控制指令
    send_position()
    gcs:send_text('0',"j1:"..tostring(current_pose.j1).."j2:"..tostring(current_pose.j2).."j3:"..tostring(current_pose.j3))
    gcs:send_text('0',"j4:"..tostring(current_pose.j4).."j5:"..tostring(current_pose.j5).."j6:"..tostring(current_pose.j6))
    gcs:send_text('0',"jiazi"..tostring(current_pose.jiazi))
    return update, 20
end

-- 处理CAN反馈数据
function process_can_feedback()
    frame = driver:read_frame()

    -- noting waiting, return early
    if not frame then
      return false
    end
    local ID = frame:id() -- 使用位掩码确保32位范围
    -- gcs:send_text(0, "CAN["..can_bus.."] msg from " .. tostring(ID) .. ": " ..
    --         frame:data(0) .. ", " .. frame:data(1) .. ", " .. frame:data(2) .. ", " ..
    --         frame:data(3) .. ", " .. frame:data(4) .. ", " .. frame:data(5) .. ", " ..
    --         frame:data(6) .. ", " .. frame:data(7) )  

    
    -- 解析末端位姿反馈
    if ID == uint32_t(0x2A2) then  -- X/Y坐标
        current_pose.x = bytes_to_int32(frame:data(0), frame:data(1), frame:data(2), frame:data(3))
        current_pose.y = bytes_to_int32(frame:data(4), frame:data(5), frame:data(6), frame:data(7))
        return true
    elseif ID == uint32_t(0x2A3) then  -- Z/RX坐标
        current_pose.z = bytes_to_int32(frame:data(0), frame:data(1), frame:data(2), frame:data(3))
        current_pose.rx = bytes_to_int32(frame:data(4), frame:data(5), frame:data(6), frame:data(7))
        return true
    elseif ID == uint32_t(0x2A4) then  -- RY/RZ坐标
        current_pose.ry = bytes_to_int32(frame:data(0), frame:data(1), frame:data(2), frame:data(3))
        current_pose.rz = bytes_to_int32(frame:data(4), frame:data(5), frame:data(6), frame:data(7))
        return true
    elseif ID == uint32_t(0x256) then  -- 运动模式反馈
        return true
    elseif ID == uint32_t(0x255) then  -- 运动模式反馈
        return true
    elseif ID == uint32_t(0x264) then  -- 运动模式反馈
        return true
    elseif ID == uint32_t(0x254) then  -- 运动模式反馈
        return true
    elseif ID == uint32_t(0x253) then  -- 运动模式反馈
        return true
    elseif ID == uint32_t(0x252) then  -- 运动模式反馈
        return true        
    elseif ID == uint32_t(0x251) then  -- 运动模式反馈
        return true
    elseif ID == uint32_t(0x2A8) then  -- 运动模式反馈
        current_pose.jiazi = bytes_to_int32(frame:data(0), frame:data(1), frame:data(2), frame:data(3))
        return true
    elseif ID == uint32_t(0x2A7) then  -- 运动模式反馈
        current_pose.j5 = bytes_to_int32(frame:data(0), frame:data(1), frame:data(2), frame:data(3))
        current_pose.j6 = bytes_to_int32(frame:data(4), frame:data(5), frame:data(6), frame:data(7))            
        return true
    elseif ID == uint32_t(0x2A6) then  -- 运动模式反馈
        current_pose.j3 = bytes_to_int32(frame:data(0), frame:data(1), frame:data(2), frame:data(3))
        current_pose.j4 = bytes_to_int32(frame:data(4), frame:data(5), frame:data(6), frame:data(7))            
        return true
    elseif ID == uint32_t(0x2A5) then  -- 运动模式反馈
        current_pose.j1 = bytes_to_int32(frame:data(0), frame:data(1), frame:data(2), frame:data(3))
        current_pose.j2 = bytes_to_int32(frame:data(4), frame:data(5), frame:data(6), frame:data(7))    
        return true
    elseif ID == uint32_t(0x2A1) then  -- 运动模式反馈
        return true
    elseif ID == uint32_t(0x265) then  -- 运动模式反馈
        return true
    elseif ID == uint32_t(0x266) then  -- 运动模式反馈
        return true
    elseif ID == uint32_t(0x261) then  -- 运动模式反馈
        return true        
    elseif ID == uint32_t(0x262) then  -- 运动模式反馈
        return true
    elseif ID == uint32_t(0x263) then  -- 运动模式反馈
        return true    
    end
    return false
end

-- 获取遥控器增量值
function get_rc_delta()
    local rc_in = {
        j1 = rc:get_pwm(1),
        j2 = rc:get_pwm(2),
        j3 = rc:get_pwm(3),
        j4 = rc:get_pwm(4),
        j5 = rc:get_pwm(5),
        j6 = rc:get_pwm(6),
        jiazi = rc:get_pwm(11),
        
    }

    local delta = {}
    for axis, value in pairs(rc_in) do
        -- 转换为-1000~1000范围并应用死区
        local scaled = (value - 1500) / 450 * 1000
        if math.abs(scaled) < deadzone then
            delta[axis] = 0
        else
            delta[axis] = scaled * step_scaling[axis] / 1000
        end
    end
    return delta
end

-- 更新目标位姿
function update_target_pose(delta)
    for axis, value in pairs(delta) do
        
        if axis ~= "jiazi" then
            if value ~= 0 then
                current_pose[axis] = math.floor(current_pose[axis] + value)
                -- 添加物理限位保护（示例值，需根据实际情况调整）
                if axis == "j1" then
                    current_pose[axis] = math.max(-150000, math.min(150000, current_pose[axis]))
                elseif axis == "j2" then
                    current_pose[axis] = math.max(0, math.min(180000, current_pose[axis]))
                elseif axis == "j3" then
                    current_pose[axis] = math.max(-170000, math.min(0, current_pose[axis]))
                -- elseif axis == "j4" then
                --     current_pose[axis] = math.max(0, math.min(180000, current_pose[axis]))
                -- elseif axis == "j5" then
                --     current_pose[axis] = math.max(0, math.min(180000, current_pose[axis]))
                -- elseif axis == "j6" then
                --     current_pose[axis] = math.max(0, math.min(180000, current_pose[axis]))
                
                end
            end
        else
        -- gcs:send_text('0',"axis"..tostring(axis).."value"..tostring(value))
            if value < -490 then
                current_pose[axis] = 0;
            else
                current_pose[axis] = math.floor((value + 500) * 70)    
            end
        end
    end
    
end

-- 发送位姿指令
function send_position()
    -- -- 发送X/Y坐标 (ID 0x152)
    -- send(0x152, int32_to_bytes(current_pose.x, current_pose.y), 8)
    
    -- -- 发送Z/RX坐标 (ID 0x153)
    -- send(0x153, int32_to_bytes(current_pose.z, current_pose.rx), 8)
    
    -- -- 发送RY/RZ坐标 (ID 0x154)
    -- send(0x154, int32_to_bytes(current_pose.ry, current_pose.rz), 8)
    -- 发送j1/j2坐标 (ID 0x155)
    send(0x155, int32_to_bytes(current_pose.j1, current_pose.j2), 8)
    
    -- 发送j3/j4坐标 (ID 0x156)
    send(0x156, int32_to_bytes(current_pose.j3, current_pose.j4), 8)
    
    -- 发送j5/j6坐标 (ID 0x157)
    send(0x157, int32_to_bytes(current_pose.j5, current_pose.j6), 8)
     
    -- 发送jiazi坐标 (ID 0x159)
    send(0x159, {((current_pose.jiazi >> 24) & 0xFF),((current_pose.jiazi >> 16) & 0xFF),((current_pose.jiazi >> 8) & 0xFF),((current_pose.jiazi) & 0xFF),0x27,0x10,0x01,0x00}, 8)
    

    -- 设置运动模式（MOVE_J模式）
    send(0x151, {0x01, 0x01, arm_speed, 0, 0, 0, 0, 0}, 8)

    -- gcs:send_text('0',"current_pose:"..tostring(current_pose.x)..","..tostring(current_pose.y)..","..tostring(current_pose.z)..","..tostring(current_pose.rx)..","..tostring(current_pose.ry)..","..tostring(current_pose.rz))
    -- gcs:send_text('0',"current_pose:"..tostring(current_pose.j1)..","..tostring(current_pose.j2)..","..tostring(current_pose.j3)..","..tostring(current_pose.j4)..","..tostring(current_pose.j5)..","..tostring(current_pose.j6))

end

-- 共用功能函数
function enable_motors()
    send(0x471, {7, 2, 0, 0, 0, 0, 0, 0}, 8)
end

function set_can_mode()
    send(0x151, {1, 0, 0, 0, 0, 0, 0, 0}, 8)
end

function emergency_stop()
    send(0x150, {1, 0, 0, 0, 0, 0, 0, 0}, 8)
end


function send(target_ID,data,dlc)  --转子绝对值位置反馈
    msg = CANFrame()
    -- enable(target_ID) 
    msg:id(uint32_t(target_ID))
    for i = 0, dlc-1 do
        msg:data(i, data[i+1])
    end
    msg:dlc(dlc)
    -- gcs:send_text(0, "CAN send: " .. string.format("0x%X", target_ID) .. " " .. table.concat(data, ", "))

    driver:write_frame(msg, 10000)
end



function int32_to_bytes(...)
    local bytes = {}
    for _, value in ipairs({...}) do
        bytes[#bytes+1] = (value >> 24) & 0xFF
        bytes[#bytes+1] = (value >> 16) & 0xFF
        bytes[#bytes+1] = (value >> 8) & 0xFF
        bytes[#bytes+1] = value & 0xFF
    end
    return bytes
end

function bytes_to_int32(b1, b2, b3, b4)
    local value = b1 << 24 | b2 << 16 | b3 << 8 | b4
    if value > 0x7FFFFFFF then
        value = value - 0x100000000
    end
    return value
end

gcs:send_text(0, "Jixieshou script loaded")
return update()