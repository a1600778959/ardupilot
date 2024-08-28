-- Control MiniCheetah motor driver over CAN
-- https://os.mbed.com/users/benkatz/code/HKC_MiniCheetah/docs/tip/CAN__com_8cpp_source.html

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

-- Load CAN driver with a buffer size of 20
local driver = CAN:get_device(20)

local target_ID = uint32_t(1168)

local max = 32767
local min = -32768
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
  return ((val / int_range) * range) + min
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
    left_rpm = to_uint(left_rpm, min,    max,    16)
    right_rpm = to_uint(right_rpm, min,    max,    16)
    right_torque = to_uint(right_torque, min,    max,    16)
    left_torque = to_uint(left_torque, min,    max,    16)

  msg = CANFrame()
  msg:id(target_ID)

  -- 0: [left_rpm[15-8]]
  msg:data(0, left_rpm >> 8)

  -- 1: [left_rpm[7-0]] 
  msg:data(1, left_rpm & 0xFF)

  -- 2: [right_rpm[15-8]]
  msg:data(2, right_rpm >> 8)

  -- 3: [right_rpm[7-0]]
  msg:data(3, right_rpm & 0xFF)

  -- 4: [right_rpm[15-8]]
  msg:data(4, left_torque >> 8)

  -- 5: [right_rpm[7-0]]
  msg:data(5, left_torque & 0xFF)

  -- 6: [right_rpm[15-8]]
  msg:data(6, right_torque >> 8)

  -- 7: [right_rpm[7-0]]
  msg:data(7, right_torque & 0xFF)


  -- sending 8 bytes of data
  msg:dlc(8)

  -- write the frame with a 10000us timeout
  driver:write_frame(msg, 10000)

end

-- send command to enable motor
function enable()
  msg = CANFrame()

  msg:id(target_ID)

  msg:data(0, 0xFF)
  msg:data(1, 0xFF)
  msg:data(2, 0xFF)
  msg:data(3, 0xFF)
  msg:data(4, 0xFF)
  msg:data(5, 0xFF)
  msg:data(6, 0xFF)
  msg:data(7, 0xFC)

  msg:dlc(8)

  driver:write_frame(msg, 10000)
end

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
  ID = from_uint(ID,0,255,32)
  ECM_ControllerTemp = from_uint(ECM_ControllerTemp,0,255,8) - 40
  ECM_MotorTemp      = from_uint(ECM_MotorTemp,0,255,8) - 40
  ECM_BusVoltage     = from_uint(ECM_BusVoltage,0,65535,16)*0.1
  ECM_BusCurrent     = from_uint(ECM_BusCurrent,0,65535,16)*0.1
  ECM_EngineSpeedRPM = from_uint(ECM_EngineSpeedRPM,min,max,16)

  return ID, ECM_ControllerTemp, ECM_MotorTemp, ECM_BusVoltage,ECM_BusCurrent,ECM_EngineSpeedRPM

end
function get_output()
  -- SRV_Channels:
end


function update()

  send(1000, 2000, 3000, 4000)
  -- gcs:send_named_float('TEST——ID',555)
  local ID, ECM_ControllerTemp, ECM_MotorTemp, ECM_BusVoltage,ECM_BusCurrent,ECM_EngineSpeedRPM = receive()
  if ID then
    -- gcs:send_named_float('ID',ID)
    gcs:send_named_float('LECM_ControllerTemp',ECM_ControllerTemp)
    gcs:send_named_float('LECM_MotorTemp',ECM_MotorTemp)
    gcs:send_named_float('LECM_BusVoltage',ECM_BusVoltage)
    gcs:send_named_float('LECM_BusCurrent',ECM_BusCurrent)
    gcs:send_named_float('LECM_EngineSpeedRPM',ECM_EngineSpeedRPM)
  -- elseif (ID == 516) then
  --   gcs:send_named_float('RECM_ControllerTemp',ECM_ControllerTemp)
  --   gcs:send_named_float('RECM_MotorTemp',ECM_MotorTemp)
  --   gcs:send_named_float('RECM_BusVoltage',ECM_BusVoltage)
  --   gcs:send_named_float('RECM_BusCurrent',ECM_BusCurrent)
  --   gcs:send_named_float('RECM_EngineSpeedRPM',ECM_EngineSpeedRPM)
  end

  return update, 10

end

function init()
  enable()
  return update, 100
end
gcs:send_text(6, "now is the zibao_motor")
return init, 1000