#using JuliaBerry
using JuliaBerry.PiGPIO

p = Pi()

const PWR_MGMT_1   = 0x6B
const SMPLRT_DIV   = 0x19
const CONFIG       = 0x1A
const GYRO_CONFIG  = 0x1B
const INT_ENABLE   = 0x38
const ACCEL_XOUT_H = 0x3B
const ACCEL_YOUT_H = 0x3D
const ACCEL_ZOUT_H = 0x3F
const GYRO_XOUT_H  = 0x43
const GYRO_YOUT_H  = 0x45
const GYRO_ZOUT_H  = 0x47

#PiGPIO.set_mode(p, 2, PiGPIO.OUTPUT)


#function i2c_write_byte(p::Pi, handle, byte_val)
#    return PiGPIO._pigpio_command(p.sl, PiGPIO._PI_CMD_I2CWS, handle, byte_val)
#end


function MPU()
    PiGPIO.i2c_write_byte(p, SMPLRT_DIV, 7)
    PiGPIO.i2c_write_byte(p, PWR_MGMT_1, 1)
    PiGPIO.i2c_write_byte(p, CONFIG, 0)
    PiGPIO.i2c_write_byte(p, GYRO_CONFIG, 24)
    PiGPIO.i2c_write_byte(p, INT_ENABLE, 1)
end


#function i2c_read_byte(p::Pi, handle)
#    return PiGPIO._pigpio_command(p.sl, PiGPIO._PI_CMD_I2CRS, handle, 0)
#end


function read_raw_data(addr)
    high = PiGPIO.i2c_read_byte(p, addr)
    low = PiGPIO.i2c_read_byte(p, addr+1)
    
    value = ((high << 8) | low)
    
    if value > 32768
        value = value - 65536
    else
        return value = value
    end
end

MPU()

try
    while true
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
    
    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0
    
    println("Ax= ", Ax, "\tAy= ", Ay,"\tAz= ", Az,"\tGx= ", Gx,"\tGy= ", Gy,"\tGz= ", Gz)
    
    sleep(1)
end
    
#finally
    #PiGPIO.set_mode(p, 2, PiGPIO.INPUT)
#end
