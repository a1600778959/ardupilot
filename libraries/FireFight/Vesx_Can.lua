-- Control MiniCheetah motor driver over CAN
-- https://os.mbed.com/users/benkatz/code/HKC_MiniCheetah/docs/tip/CAN__com_8cpp_source.html

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 1
local PARAM_TABLE_PREFIX = "ZIBAO_"
local PARAM_TABLE_SIZE = 4

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format("Zibao: could not find %s parameter", name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("Zibao: could not add param %s", name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, PARAM_TABLE_SIZE), "Zibao: could not add param table")

-- Load CAN driver with a buffer size of 20

local LEFT_CAN_ID = bind_add_param('LEFT_CAN_ID', 1, 1)
local RIGHT_CAN_ID = bind_add_param('RIGHT_CAN_ID', 2, 2)

local driver = CAN:get_device(20)



--local vel_max = 65
--local Kp_min = 0
--local Kp_max = 500
--local Kd_min = 0
--local kd_max = 5
--local torque_max = 18

local position_des = 0
local position_inc = 0.01
--这是速度限幅通道，默认通道为5通道，可以通过地面站进行更改
local SPEED_RC = bind_add_param('SPEED_RC', 3, 5)
local max_rpm =  bind_add_param('MAX_RPM',4,4000) --最大转速
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


function left_motor(left_rpm)
  msg = CANFrame()
  msg:id((uint32_t(1) << 31) | uint32_t(LEFT_CAN_ID:get()|0x0300)) -- get the left motor ID from parameter

  -- 0: [left_rpm[0-7]]
  msg:data(0, left_rpm & 0xFF)

  -- 1: [left_rpm[8-15]] 
  msg:data(1, (left_rpm >> 8) & 0xff)

  -- 2: [right_rpm[16-23]]
  msg:data(2, (left_rpm >> 16) & 0xff)

  -- 3: [right_rpm[24-31]]
  msg:data(3, (left_rpm >> 24) & 0xff)

  -- sending 4 bytes of data
  msg:dlc(4)

  -- write the frame with a 10000us timeout
  driver:write_frame(msg, 10000)
  
end

function right_motor(right_rpm)
  msg = CANFrame()
  msg:id((uint32_t(1) << 31) | uint32_t(RIGHT_CAN_ID:get()|0x0300)) -- get the RIGHT motor ID from parameter

  -- 0: [left_rpm[0-7]]
  msg:data(0, right_rpm & 0xFF)

  -- 1: [left_rpm[8-15]] 
  msg:data(1, (right_rpm >> 8) & 0xff)

  -- 2: [right_rpm[16-23]]
  msg:data(2, (right_rpm >> 16) & 0xff)

  -- 3: [right_rpm[24-31]]
  msg:data(3, (right_rpm >> 24) & 0xff)

  -- sending 4 bytes of data
  msg:dlc(4)

  -- write the frame with a 10000us timeout
  driver:write_frame(msg, 10000)
  
end

-- send a motor command
function send(left_rpm,right_rpm )

  -- 16 bit left_rpm command, between 32767 and -32768  RPM
  -- 16 bit right_rpm command, between 32767 and -32768 RPM
  -- 16 bit left_torque, between 0 and 32767 and -32768  N
  -- 16 bit right_torque, between 0 and 32767 and -32768  N
  local max = max_rpm:get(); -- max rpm
  -- range check
  assert(math.abs(left_rpm) <= max, "left_rpm out of range")
  assert(math.abs(right_rpm) <= max, "right_rpm out of range")
  -- assert(math.abs(left_torque) <= max, "left_torque out of range")
  -- assert(math.abs(right_torque) <= max, "right_torque out of range")
  
  -- convert from decimal to integer
  -- left_rpm = to_uint(left_rpm, min,    max,    16)
  -- right_rpm = to_uint(right_rpm, min,    max,    16)
  -- right_torque = to_uint(right_torque, min,    max,    16)
  -- left_torque = to_uint(left_torque, min,    max,    16)

  gcs:send_named_float('left_rpm',left_rpm) 
  gcs:send_named_float('right_rpm',right_rpm) 
  left_motor(left_rpm)
  right_motor(right_rpm)

end





-- receive data from motor
function receive()

    -- Read a message from the buffer
    frame = driver:read_frame()

    -- noting waiting, return early
    if not frame then
      return
    end

  -- 8 bit ID
  -- 8 bit ECM_ControllerTemp;  单位摄氏度need -40 
  -- 8 bit ECM_MotorTemp;    need -40
  -- 16 bit ECM_BusVoltage;  单位0.1V  
  -- 16 bit ECM_BusCurrent;  单位0.1V 
  -- 16 bit ECM_EngineSpeedRPM;  单位rpm

  -- 0: [ID[7-0]]
  -- 1: [ECM_ControllerTemp[0-7]]
  -- 2: [ECM_MotorTemp[8-15]]
  -- 3: [ECM_BusVoltage[16-31]]
  -- 4: [ECM_BusCurrent[32-47]]
  -- 5: [ECM_EngineSpeedRPM[48-63]]

  local ID = (frame:id());
  local ECM_ControllerTemp = (frame:data(0))
  local ECM_MotorTemp = (frame:data(1))
  local ECM_BusVoltage = (frame:data(3) << 8) | (frame:data(2))
  local ECM_BusCurrent = (frame:data(5) << 8) | (frame:data(4))
  local ECM_EngineSpeedRPM = (frame:data(7) << 8) | (frame:data(6))
  -- gcs:send_named_float("CAN_RXID",ID);
  -- from integer to decimal
  -- ID = from_uint(ID,0,255,32)
  ECM_ControllerTemp = from_uint(ECM_ControllerTemp,0,255,8) - 40
  ECM_MotorTemp      = from_uint(ECM_MotorTemp,0,255,8) - 40
  ECM_BusVoltage     = from_uint(ECM_BusVoltage,0,65535,16)*0.1
  ECM_BusCurrent     = from_uint(ECM_BusCurrent,0,65535,16)*0.1
  ECM_EngineSpeedRPM = from_uint(ECM_EngineSpeedRPM,min,max,16)

  return ID, ECM_ControllerTemp, ECM_MotorTemp, ECM_BusVoltage,ECM_BusCurrent,ECM_EngineSpeedRPM

end


function get_output()
  local left_rpm = SRV_Channels:get_output_pwm(73)   --获取通道1输出数值 
  local right_rpm = SRV_Channels:get_output_pwm(74)   --获取通道3输出数值
  local rpm_set = max_rpm:get();  --获取最大转速
  local out_max_min = rc:get_pwm(SPEED_RC:get());   --限幅通道

  if out_max_min then
    out_max_min = (out_max_min - 1050)/900;
  else
    out_max_min = 0;
  end
  
  if left_rpm then
    left_rpm = math.floor(((1500-left_rpm)/500) *rpm_set*out_max_min)   --变成占空比输出-1,1
  else
    left_rpm = 0;
  end
  if right_rpm then
    right_rpm = math.floor(((1500-right_rpm)/500) *rpm_set*out_max_min)   --变成占空比输出-1,1
  else
    right_rpm = 0;
  end

  gcs:send_named_float("out_max_min",out_max_min)
  
  send(left_rpm,right_rpm)
end



function update()

  -- send(1000, 2000, 3000, 4000)
  -- gcs:send_named_float('TEST——ID',555)
  get_output();
  -- local ID, ECM_ControllerTemp, ECM_MotorTemp, ECM_BusVoltage,ECM_BusCurrent,ECM_EngineSpeedRPM = receive()
  -- if ID then
  --   -- gcs:send_named_float('ID',ID)
  --   gcs:send_named_float('LECM_ControllerTemp',ECM_ControllerTemp)
  --   gcs:send_named_float('LECM_MotorTemp',ECM_MotorTemp)
  --   gcs:send_named_float('LECM_BusVoltage',ECM_BusVoltage)
  --   gcs:send_named_float('LECM_BusCurrent',ECM_BusCurrent)
  --   gcs:send_named_float('LECM_EngineSpeedRPM',ECM_EngineSpeedRPM)
  -- -- elseif (ID == 516) then
  -- --   gcs:send_named_float('RECM_ControllerTemp',ECM_ControllerTemp)
  -- --   gcs:send_named_float('RECM_MotorTemp',ECM_MotorTemp)
  -- --   gcs:send_named_float('RECM_BusVoltage',ECM_BusVoltage)
  -- --   gcs:send_named_float('RECM_BusCurrent',ECM_BusCurrent)
  -- --   gcs:send_named_float('RECM_EngineSpeedRPM',ECM_EngineSpeedRPM)
  -- end

  return update, 2.5

end

function init()
  -- enable()
  return update, 100
end
gcs:send_text(6, "now is the zibao_motor")
return init, 1000