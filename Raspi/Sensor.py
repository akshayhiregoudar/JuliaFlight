import smbus            #import SMBus module of I2C
from time import sleep          #import

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


#AK8963_ST1         = 0x02
#AK8963_MAGNET_OUT  = 0X03
#AK8963_CNTL1       = 0X0A
#AK8963_CNTL2       = 0X0B
#AK8963_ASAX        = 0X10
#AK8963_MODE_DOWN   = 0X00
#AK8963_MODE_ONE    = 0X01
#AK8963_MODE_C8HZ   = 0X02
#AK8963_MODE_C100HZ = 0X06
#AK8963_MODE_BIT_14 = 0X00
#AK8963_MODE_BIT_16 = 0X01



def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    
    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
    sleep(1)
    
def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu9250
        if(value > 32768):
                value = value - 65536
        return value

#def AK8963(self, mode, mfs):
#    if mfs == AK8963_BIT_14:
#        self.mres = 4912.0/8190.0
#    else: #  mfs == AK8963_BIT_16:
#        self.mres = 4912.0/32760.0
#
#    bus.write_byte_data(AK8963_Address, AK8963_CNTL1, 0x00)
#    time.sleep(0.01)
#
#    # set read FuseROM mode
#    bus.write_byte_data(AK8963_Address, AK8963_CNTL1, 0x0F)
#    time.sleep(0.01)
#
#    # read coef data
#    data = bus.read_i2c_block_data(AK8963_Address, AK8963_ASAX, 3)
#
#    self.magXcoef = (data[0] - 128) / 256.0 + 1.0
#    self.magYcoef = (data[1] - 128) / 256.0 + 1.0
#    self.magZcoef = (data[2] - 128) / 256.0 + 1.0
#
#    # set power down mode
#    bus.write_byte_data(AK8963_Address, AK8963_CNTL1, 0x00)
#    time.sleep(0.01)
#
#    # set scale&continous mode
#    bus.write_byte_data(AK8963_Address, AK8963_CNTL1, (mfs<<4|mode))
#    time.sleep(0.01)
#    
#    
##def readMag(self):
#    x=0
#    y=0
#    z=0
#
#    # check data ready
#    drdy = bus.read_byte_data(AK8963_Address, AK8963_ST1)
#    if drdy & 0x01 :
#        data = bus.read_i2c_block_data(AK8963_Address, AK8963_MAGNET_OUT, 7)
#
#    # check overflow
#        if (data[6] & 0x08)!=0x08:
#            x = self.dataConv(data[0], data[1])
#            y = self.dataConv(data[2], data[3])
#            z = self.dataConv(data[4], data[5])
#
#            x = round(x * self.mres * self.magXcoef, 3)
#            y = round(y * self.mres * self.magYcoef, 3)
#            z = round(z * self.mres * self.magZcoef, 3)
#                
#            return {"x":x, "y":y, "z":z}




bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU9250 device address

MPU_Init()

print ("Reading Data of Gyroscope and Accelerometer")

while True:
    
    #Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    
    #Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    
    #Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
    
    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0
    

    print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)     
    sleep(1)