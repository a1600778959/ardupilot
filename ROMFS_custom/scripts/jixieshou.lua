-- 机械臂增量控制脚本
local can_bus = 1          -- CAN总线编号
local arm_speed = 50       -- 运动速度百分比
local deadzone = 50        -- 遥控器死区阈值
local step_scaling = {     -- 增量步长比例
    x = 1000,    -- 0.001mm/step
    y = 1000,
    z = 1000,
    rx = 500,    -- 0.001°/step
    ry = 500,
    rz = 500
}


-- 当前位姿状态
local current_pose = {
    x = 0, y = 0, z = 0,
    rx = 0, ry = 0, rz = 0
}
local driver = CAN:get_device(5)
-- 初始化函数
function update()
    -- 初始化CAN总线
    -- gcs:send_text(0, "Distance:"..tostring(distance).." Instance:"..tostring(instance))
    if not arming:is_armed() then
        enable_motors()
        set_can_mode()
    end

    -- 主循环
    if not arming:is_armed() then
        return update, 20
    end

    -- 解析反馈数据
    process_can_feedback()

    -- 读取遥控器增量输入
    local delta = get_rc_delta()

    -- 更新目标位姿
    update_target_pose(delta)

    -- 发送控制指令
    send_position()

    return update, 20
end

-- 处理CAN反馈数据
function process_can_feedback()
    frame = driver:read_frame()

    -- noting waiting, return early
    if not frame then
      return
    end
    local ID = frame:data(0)

    
    -- 解析末端位姿反馈
    if ID == 0x2A2 then  -- X/Y坐标
        current_pose.x = bytes_to_int32(frame:data(1), frame:data(2), frame:data(3), frame:data(4))
        current_pose.y = bytes_to_int32(frame:data(5), frame:data(6), frame:data(7), frame:data(8))
    elseif ID == 0x2A3 then  -- Z/RX坐标
        current_pose.z = bytes_to_int32(frame:data(1), frame:data(2), frame:data(3), frame:data(4))
        current_pose.rx = bytes_to_int32(frame:data(5), frame:data(6), frame:data(7), frame:data(8))
    elseif ID == 0x2A4 then  -- RY/RZ坐标
        current_pose.ry = bytes_to_int32(frame:data(1), frame:data(2), frame:data(3), frame:data(4))
        current_pose.rz = bytes_to_int32(frame:data(5), frame:data(6), frame:data(7), frame:data(8))
    end
end

-- 获取遥控器增量值
function get_rc_delta()
    local rc_in = {
        x = rc:get_pwm(1),
        y = rc:get_pwm(2),
        z = rc:get_pwm(3),
        rx = rc:get_pwm(4),
        ry = rc:get_pwm(5),
        rz = rc:get_pwm(6)
    }

    local delta = {}
    for axis, value in pairs(rc_in) do
        -- 转换为-1000~1000范围并应用死区
        local scaled = (value - 1500) / 500 * 1000
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
        if value ~= 0 then
            current_pose[axis] = current_pose[axis] + value
            -- 添加物理限位保护（示例值，需根据实际情况调整）
            if axis:match('^r') then  -- 旋转轴
                current_pose[axis] = math.max(-180000, math.min(180000, current_pose[axis]))
            else  -- 平移轴
                current_pose[axis] = math.max(-500000, math.min(500000, current_pose[axis]))
            end
        end
    end
end

-- 发送位姿指令
function send_position()
    -- 发送X/Y坐标 (ID 0x152)
    send(0x152, int32_to_bytes(current_pose.x, current_pose.y), 8)
    
    -- 发送Z/RX坐标 (ID 0x153)
    send(0x153, int32_to_bytes(current_pose.z, current_pose.rx), 8)
    
    -- 发送RY/RZ坐标 (ID 0x154)
    send(0x154, int32_to_bytes(current_pose.ry, current_pose.rz), 8)
    
    -- 设置运动模式（MOVE_P模式）
    send(0x151, {0x01, 0x00, arm_speed, 0, 0, 0, 0, 0}, 8)
end

-- 共用功能函数
function enable_motors()
    send(0x471, {7, 2, 0, 0, 0, 0, 0, 0}, 8)
end

function set_can_mode()
    send(0x151, {1, 0, 0, 0, 0, 0, 0, 0}, 8)
end



function send(target_ID,data,dlc)  --转子绝对值位置反馈
    msg = CANFrame()
    -- enable(target_ID) 
    msg:id(uint32_t(target_ID))
    for i = 0, dlc-1 do
        msg:data(i, data[i+1])
    end
    msg:dlc(dlc)
    gcs:send_text(0, "CAN send: " .. string.format("0x%X", target_ID) .. " " .. table.concat(data, ", "))

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

return update()