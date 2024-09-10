-- Control MiniCheetah motor driver over CAN
-- https://os.mbed.com/users/benkatz/code/HKC_MiniCheetah/docs/tip/CAN__com_8cpp_source.html

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

-- Load CAN driver with a buffer size of 20
local driver = CAN:get_device(20)

local target_L_heart_ID = uint32_t(0x07000001)  --左边can心跳包
local target_R_heart_ID = uint32_t(0x07000002)  --右边can心跳包

local target_L_fb_ID    = uint32_t(0x05800001)  --左边can反馈包
local target_R_fb_ID    = uint32_t(0x05800002)  --右边can反馈包

local target_L_control_ID = uint32_t(0x06000001)  --左边can发送包
local target_R_control_ID = uint32_t(0x06000002)  --右边can发送包

local current_L_pos = 0;
local current_R_pos = 0;



local current_contro_status_L = 0;
local current_contro_status_R = 0;
local Err_status_L = 0;
local Err_status_R = 0;

local max = 10000
local min = -10000
--local vel_max = 65
--local Kp_min = 0
--local Kp_max = 500
--local Kd_min = 0
--local kd_max = 5
--local torque_max = 18

local position_des = 0
local position_inc = 0.01

-- convert decimal to int within given range and width
function to_uint(val, min, max, bits)
  local range = max - min
  local int_range = 0
  for i = 0, bits - 1 do
    int_range = int_range | (1 << i)
  end
  return math.floor((((val - min)/range) * int_range) + 0.5)
end

-- convert int to decimal within given range and width
function from_uint(val, min, max, bits)
  local range = max - min
  local int_range = 0
  for i = 0, bits - 1 do
    int_range = int_range | (1 << i)
  end
  if val >  int_range/2 then
    val = val - int_range;     
  end
  if val > max then
    val = max;
  elseif val < min then
    val = min;
  end  
  return val
end

-- send a motor command
function send(left_rpm,right_rpm , left_torque, right_torque)

  -- 16 bit left_rpm command, between 32767 and -32768  RPM
  -- 16 bit right_rpm command, between 32767 and -32768 RPM
  -- 16 bit left_torque, between 0 and 32767 and -32768  N
  -- 16 bit right_torque, between 0 and 32767 and -32768  N

  -- range check
  assert(math.abs(left_rpm) <= max, "left_rpm out of range")
  assert(math.abs(right_rpm) <= max, "right_rpm out of range")
  assert(math.abs(left_torque) <= max, "left_torque out of range")
  assert(math.abs(right_torque) <= max, "right_torque out of range")
  
  -- convert from decimal to integer
  -- left_rpm = to_uint(left_rpm, min,    max,    16)
  -- right_rpm = to_uint(right_rpm, min,    max,    16)
  -- right_torque = to_uint(right_torque, min,    max,    16)
  -- left_torque = to_uint(left_torque, min,    max,    16)

  gcs:send_named_float('left_rpm',left_rpm) 
  gcs:send_named_float('right_rpm',right_rpm) 
  
  msg = CANFrame()
  msg:id(target_ID)

  -- 0: [left_rpm[15-8]]
  msg:data(0, left_rpm & 0xFF)

  -- 1: [left_rpm[7-0]] 
  msg:data(1, (left_rpm >> 8) & 0xff)

  -- 2: [right_rpm[15-8]]
  msg:data(2, right_rpm & 0xFF)

  -- 3: [right_rpm[7-0]]
  msg:data(3, (right_rpm >> 8)& 0xff)

  -- 4: [right_rpm[15-8]]
  msg:data(4, left_torque & 0xFF)

  -- 5: [right_rpm[7-0]]
  msg:data(5, (left_torque >> 8)& 0xff)

  -- 6: [right_rpm[15-8]]
  msg:data(6, right_torque & 0xFF)

  -- 7: [right_rpm[7-0]]
  msg:data(7, right_torque>> 8 )


  -- sending 8 bytes of data
  msg:dlc(8)

  -- write the frame with a 10000us timeout
  driver:write_frame(msg, 10000)

end

-- send command to enable motor


-- send command to disable motor
function disable()
  msg = CANFrame()

  msg:id(target_ID)

  msg:data(0, 0xFF)
  msg:data(1, 0xFF)
  msg:data(2, 0xFF)
  msg:data(3, 0xFF)
  msg:data(4, 0xFF)
  msg:data(5, 0xFF)
  msg:data(6, 0xFF)
  msg:data(7, 0xFD)

  msg:dlc(8)

  driver:write_frame(msg, 10000)
end

-- send command to zero motor
function zero()
  msg = CANFrame()

  msg:id(target_ID)

  msg:data(0, 0xFF)
  msg:data(1, 0xFF)
  msg:data(2, 0xFF)
  msg:data(3, 0xFF)
  msg:data(4, 0xFF)
  msg:data(5, 0xFF)
  msg:data(6, 0xFF)
  msg:data(7, 0xFE)

  msg:dlc(8)

  driver:write_frame(msg, 10000)
end



-- receive data from motor
function receive()

    -- Read a message from the buffer
    frame = driver:read_frame()

    -- noting waiting, return early
    if not frame then
      return
    end

    return frame;
  -- local ID = (frame:id());
  -- local head_ID1 = (frame:data(0))
  -- local function_ID = (frame:data(1))
  -- local head_ID2 = (frame:data(2))
  -- local head_ID3 = (frame:data(3))
  -- if head_ID1 == 0x60 and function_ID == 0x04 and head_ID2 == 0x21 and head_ID3 == 0x01 then  --帧头
  --   local rotor_pos = (frame:data(6) << 8) | (frame:data(7))    
  --   rotor_pos = from_uint(rotor_pos,0,9999,8)
  --   return ID,rotor_pos;
  -- end

  -- gcs:send_named_float("CAN_RXID",ID);
  -- from integer to decimal
  -- ID = from_uint(ID,0,255,32)
  



end


function get_output()
  local left_rpm = SRV_Channels:get_output_pwm(73)   --获取通道1输出数值 
  local right_rpm = SRV_Channels:get_output_pwm(74)   --获取通道3输出数值
  left_rpm = math.floor(((left_rpm-1500)/500) * 5000)
  right_rpm = math.floor(((right_rpm-1500)/500) * 5000)
  

  
  send(left_rpm,right_rpm,0,0)
end

function get_degree()
    local degree_L,degree_R = 0,0;
    local P4_UD =   rc:get_pwm(10);    --获取P3上下遥感的数值
    local P5_UD =   rc:get_pwm(12);   --获取P5上下遥感的数值
    if(math.abs(P4_UD - 1500)>60) then
      degree_R = -(P4_UD - 1500)*4    --每度数值27.7，控制周期为200HZ，杆量推满时候每秒2度
    else
      degree_R = 0;
    end
    
    if(math.abs(P5_UD - 1500)>60) then
      degree_L = (P5_UD - 1500)*4   --每度数值27.7，控制周期为200HZ，杆量推满时候每秒2度
    else
      degree_L = 0;
    end
    return degree_L,degree_R
end

function enable(target_ID)  --使能函数
    msg = CANFrame()

    msg:id( (uint32_t(1) << 31) | target_ID)

    msg:data(0, 0x23)
    msg:data(1, 0x0D)
    msg:data(2, 0x20)
    msg:data(3, 0x01)
    msg:data(4, 0x00)
    msg:data(5, 0x00)
    msg:data(6, 0x00)
    msg:data(7, 0x00)

    msg:dlc(8)

    driver:write_frame(msg, 10000)  
end

function temp_fb(target_ID)  --转子绝对值位置反馈
    msg = CANFrame()
    -- enable(target_ID) 
    msg:id( (uint32_t(1) << 31) | target_ID)
    msg:data(0, 0x40)
    msg:data(1, 0x0F)
    msg:data(2, 0x21)
    msg:data(3, 0x01)
    msg:data(4, 0x00)
    msg:data(5, 0x00)
    msg:data(6, 0x00)
    msg:data(7, 0x00)

    msg:dlc(8)

    driver:write_frame(msg, 10000)    
    
end

function rotor_pos_fb(target_ID)  --转子绝对值位置反馈
    msg = CANFrame()
    -- enable(target_ID) 
    msg:id( (uint32_t(1) << 31) | target_ID)
    msg:data(0, 0x40)
    msg:data(1, 0x04)
    msg:data(2, 0x21)
    msg:data(3, 0x02)
    msg:data(4, 0x00)
    msg:data(5, 0x00)
    msg:data(6, 0x00)
    msg:data(7, 0x00)

    msg:dlc(8)

    driver:write_frame(msg, 10000)    
    
end

function in_range(num,min,max)
    if num < min then
        num = min;
    elseif num > max then
        num = max;
    end
    return num;
end

function speed_contro(target_ID,speed)  --需要进行角度限制
    enable(target_ID)  --使能
    msg = CANFrame()
    -- degree = in_range(degree,-60,60)  --暂定限制角度为-60～60
    msg:id( (uint32_t(1) << 31) | target_ID)
    msg:data(0, 0x23)
    msg:data(1, 0x00)
    msg:data(2, 0x20)
    msg:data(3, 0x01)
    msg:data(4, (speed >> 24)&0xff)
    msg:data(5, (speed >> 16)&0xff)
    msg:data(6, (speed >> 8)&0xff)
    msg:data(7, speed & 0xff)
    msg:dlc(8)
    driver:write_frame(msg, 10000)
end

function pos_contro(target_ID,degree)  --需要进行角度限制
    enable(target_ID)  --使能
    msg = CANFrame()
    -- degree = in_range(degree,-60,60)  --暂定限制角度为-60～60
    msg:id( (uint32_t(1) << 31) | target_ID)
    msg:data(0, 0x23)
    msg:data(1, 0x02)
    msg:data(2, 0x20)
    msg:data(3, 0x01)
    msg:data(4, (degree >> 24)&0xff)
    msg:data(5, (degree >> 16)&0xff)
    msg:data(6, (degree >> 8)&0xff)
    msg:data(7, degree & 0xff)
    msg:dlc(8)
    driver:write_frame(msg, 10000)
end


function update()
  -- enable(target_L_control_ID)
  local L_inc_degree,R_inc_degree = 0,0;
  rotor_pos_fb(target_L_control_ID)

  -- send(1000, 2000, 3000, 4000)
  -- gcs:send_named_float('TEST——ID',555)
  local receive_buff = receive();
  if receive_buff then
    ID = receive_buff:id()
    if ID == ((uint32_t(1) << 31) |target_L_fb_ID) then
        -- gcs:send_text(0,string.format("msg:"  .. ": %i, %i, %i, %i, %i, %i, %i, %i", receive_buff:data(0), receive_buff:data(1), receive_buff:data(2), receive_buff:data(3), receive_buff:data(4), receive_buff:data(5), receive_buff:data(6), receive_buff:data(7)))
        if ((receive_buff:data(0) == 0x60) and (receive_buff:data(1) == 0x04) and (receive_buff:data(2) == 0x21) and (receive_buff:data(3) == 0x02)) then  --帧头
          current_L_pos = (receive_buff:data(4) << 24) |(receive_buff:data(5) << 16) |(receive_buff:data(6) << 8) | (receive_buff:data(7)) 
          gcs:send_named_float('L_pos',current_L_pos)
          -- gcs:send_text(0,string.format("L_pos is:" .. tostring(current_L_pos/27.7)))
        end
      -- if ID == ((uint32_t(1) << 31) |target_R_fb_ID) then
      --   if receive_buff:data(1) == 0x60 and receive_buff:data(2) == 0x04 and receive_buff:data(3) == 0x21 and receive_buff:data(4) == 0x01 then  --帧头
      --     current_R_pos = (receive_buff:data(6) << 8) | (receive_buff:data(7)) 
      --     gcs:send_named_float('R_pos',current_R_pos/27.7)
      --   end
      -- end
    elseif ID == ((uint32_t(1) << 31) |target_L_heart_ID) then
        -- gcs:send_text(0,"target_L_heart_ID")
        if receive_buff:data(0) == 0x05 and receive_buff:data(1) == 0x00 then  --帧头
          current_contro_status_L = (receive_buff:data(4) << 8) | (receive_buff:data(5)) 
          Err_status_L = (receive_buff:data(6) << 8) | (receive_buff:data(7)) 
          gcs:send_named_float('L_current',current_contro_status_L)
          gcs:send_named_float('L_Err',Err_status_L)
          -- gcs:send_text(0,string.format("L_current is:" .. tostring(current_contro_status_L)))
        end
    end
      -- if ID == ((uint32_t(1) << 31) |target_R_heart_ID) then
      --   if receive_buff:data(1) == 0x05 and receive_buff:data(2) == 0x00 then  --帧头
      --     current_contro_status_R = (receive_buff:data(4) << 8) | (receive_buff:data(5)) 
      --     Err_status_R = (receive_buff:data(6) << 8) | (receive_buff:data(7)) 
      --     gcs:send_named_float('R_current',current_contro_status_L)
      --     gcs:send_named_float('R_Err',Err_status_L)
      --   end
      -- end    
  end
  rotor_pos_fb(target_R_control_ID)
  local receive_buff = receive();
  if receive_buff then
    ID = receive_buff:id()
    if ID == ((uint32_t(1) << 31) |target_R_fb_ID) then
        -- gcs:send_text(0,string.format("msg:"  .. ": %i, %i, %i, %i, %i, %i, %i, %i", receive_buff:data(0), receive_buff:data(1), receive_buff:data(2), receive_buff:data(3), receive_buff:data(4), receive_buff:data(5), receive_buff:data(6), receive_buff:data(7)))
        if ((receive_buff:data(0) == 0x60) and (receive_buff:data(1) == 0x04) and (receive_buff:data(2) == 0x21) and (receive_buff:data(3) == 0x01)) then  --帧头
          current_R_pos = (receive_buff:data(6) << 8) | (receive_buff:data(7)) 
          gcs:send_named_float('L_pos',current_R_pos/27.7)
          -- gcs:send_text(0,string.format("L_pos is:" .. tostring(current_L_pos/27.7)))
        end
    end
  end  

  temp_fb(target_R_control_ID)
  local receive_buff = receive();
  if receive_buff then
    ID = receive_buff:id()
    if ID == ((uint32_t(1) << 31) |target_R_fb_ID) then
        -- gcs:send_text(0,string.format("msg:"  .. ": %i, %i, %i, %i, %i, %i, %i, %i", receive_buff:data(0), receive_buff:data(1), receive_buff:data(2), receive_buff:data(3), receive_buff:data(4), receive_buff:data(5), receive_buff:data(6), receive_buff:data(7)))
        if ((receive_buff:data(0) == 0x60) and (receive_buff:data(1) == 0x0F) and (receive_buff:data(2) == 0x21) and (receive_buff:data(3) == 0x01)) then  --帧头
          R_temp = (receive_buff:data(6)) 
          gcs:send_named_float('R_temp',R_temp)
          -- gcs:send_text(0,string.format("L_pos is:" .. tostring(current_L_pos/27.7)))
        end
    end
  end  
  --     -- if ID == ((uint32_t(1) << 31) |target_R_fb_ID) then
  --     --   if receive_buff:data(1) == 0x60 and receive_buff:data(2) == 0x04 and receive_buff:data(3) == 0x21 and receive_buff:data(4) == 0x01 then  --帧头
  --     --     current_R_pos = (receive_buff:data(6) << 8) | (receive_buff:data(7)) 
  --     --     gcs:send_named_float('R_pos',current_R_pos/27.7)
  --     --   end
  --     -- end
  --   elseif ID == ((uint32_t(1) << 31) |target_R_heart_ID) then
  --       -- gcs:send_text(0,"target_L_heart_ID")
  --       if receive_buff:data(0) == 0x05 and receive_buff:data(1) == 0x00 then  --帧头
  --         current_contro_status_R = (receive_buff:data(4) << 8) | (receive_buff:data(5)) 
  --         Err_status_R = (receive_buff:data(6) << 8) | (receive_buff:data(7)) 
  --         gcs:send_named_float('R_current',current_contro_status_L)
  --         gcs:send_named_float('R_Err',Err_status_L)
  --         -- gcs:send_text(0,string.format("L_current is:" .. tostring(current_contro_status_L)))
  --       end
  --   end
  --     -- if ID == ((uint32_t(1) << 31) |target_R_heart_ID) then
  --     --   if receive_buff:data(1) == 0x05 and receive_buff:data(2) == 0x00 then  --帧头
  --     --     current_contro_status_R = (receive_buff:data(4) << 8) | (receive_buff:data(5)) 
  --     --     Err_status_R = (receive_buff:data(6) << 8) | (receive_buff:data(7)) 
  --     --     gcs:send_named_float('R_current',current_contro_status_L)
  --     --     gcs:send_named_float('R_Err',Err_status_L)
  --     --   end
  --     -- end    
  -- end
  local L_control = 0;
  local R_control = 0;
  if arming:is_armed() then
    L_inc_degree,R_inc_degree = get_degree()  --从遥控器获取角度增量
    if L_inc_degree then
      L_control = L_inc_degree;   

    end
    if R_inc_degree then
      R_control = R_inc_degree;   

    end
  else
    L_inc_degree,R_inc_degree = 0,0;
    L_control,R_control = 0,0;
  end

  L_control = math.floor(L_control);
  -- gcs:send_text(0,"L_control:"..tostring(L_control))
  speed_contro(target_L_control_ID,L_control);
  speed_contro(target_R_control_ID,R_control);

  -- -- pos_contro(target_R_control_ID,uint32_t(R_inc_degree+current_R_pos))
  -- pos_contro(target_L_control_ID,-30000)
  -- pos_contro(target_R_control_ID,-50000)

  -- speed_contro(target_R_control_ID,R_inc_degree);
  return update, 10  --实际周期为*20

end

function show_frame(dnum, frame)
    gcs:send_text(0,string.format("CAN[%u] msg from " .. tostring(frame:id()) .. ": %i, %i, %i, %i, %i, %i, %i, %i", dnum, frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6), frame:data(7)))
end

function init()
  gcs:send_text(6, "now is the zibao_motor")
  -- enable(target_L_control_ID)
  -- enable(target_L_control_ID)
  -- enable(target_R_control_ID)
  return update,10
end
gcs:send_text(6, "now is the zibao_motor")
return init, 10