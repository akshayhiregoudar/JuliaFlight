
# Using Pkg
# Pkg.add(PackageSpec(url="https://github.com/JuliaBerry/JuliaBerry.jl"))


using JuliaBerry.PiGPIO

p = Pi()

DEVICE_ADDRESS       = 0x68

AK863_DEVICE_ADDRESS = 0x0C

DEVICE_ID            = 0x71

SMPLRT_DIV     = 0x19
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C
ACCEL_CONFIG_2 = 0x1D
LP_ACCEL_ODR   = 0x1E
WOM_THR        = 0x1F
FIFO_EN        = 0x23
I2C_MST_CTRL   = 0x24
I2C_MST_STATUS = 0x36
INT_PIN_CFG    = 0x37
INT_ENABLE     = 0x38
INT_STATUS     = 0x3A
ACCEL_OUT      = 0x3B
TEMP_OUT       = 0x41
GYRO_OUT       = 0x43

I2C_MST_DELAY_CTRL = 0x67
SIGNAL_PATH_RESET  = 0x68
MOT_DETECT_CTRL    = 0x69
USER_CTRL          = 0x6A
PWR_MGMT_1         = 0x6B
PWR_MGMT_2         = 0x6C
FIFO_R_W           = 0x74
WHO_AM_I           = 0x75

## Gyro Full Scale Select 250dps
GFS_250  = 0x00
## Gyro Full Scale Select 500dps
GFS_500  = 0x01
## Gyro Full Scale Select 1000dps
GFS_1000 = 0x02
## Gyro Full Scale Select 2000dps
GFS_2000 = 0x03
## Accel Full Scale Select 2G
AFS_2G   = 0x00
## Accel Full Scale Select 4G
AFS_4G   = 0x01
## Accel Full Scale Select 8G
AFS_8G   = 0x02
## Accel Full Scale Select 16G
AFS_16G  = 0x03

# AK8963 Register Addresses
AK8963_ST1        = 0x02
AK8963_MAGNET_OUT = 0x03
AK8963_CNTL1      = 0x0A
AK8963_CNTL2      = 0x0B
AK8963_ASAX       = 0x10

# CNTL1 Mode select
## Power down mode
AK8963_MODE_DOWN   = 0x00
## One shot data output
AK8963_MODE_ONE    = 0x01

## Continous data output 8Hz
AK8963_MODE_C8HZ   = 0x02
## Continous data output 100Hz
AK8963_MODE_C100HZ = 0x06

# Magneto Scale Select
## 14bit output
AK8963_BIT_14 = 0x00
## 16bit output
AK8963_BIT_16 = 0x01

struct MPU9250
    function __init__(self, address=DEVICE_ADDRESS)
        self.address = address
        self.configMPU9250(GFS_250, AFS_2G)
        self.configAK8963(AK8963_MODE_C100HZ, AK8963_BIT_16)
    end
    
    function searchDevice(self)
        who_am_i = PiGPIO.write_byte_data(p, self.address, WHO_AM_I)
        if(who_am_i == DEVICE_ID)
            return true
        else
            return false
        end
    end
    
     function configMPU9250(self, gfs, afs)
        if gfs == GFS_250
            self.gres = 250.0/32768.0
            elseif gfs == GFS_500
            self.gres = 500.0/32768.0
            elseif gfs == GFS_1000
            self.gres = 1000.0/32768.0
            else # gfs == GFS_2000
            self.gres = 2000.0/32768.0
        end

        if afs == AFS_2G
            self.ares = 2.0/32768.0
            elseif afs == AFS_4G
            self.ares = 4.0/32768.0
            elseif afs == AFS_8G
            self.ares = 8.0/32768.0
            else # afs == AFS_16G
            self.ares = 16.0/32768.0
        end
        
        PiGPIO.write_byte_data(p, self.address, PWR_MGMT_1, 0x00)
        sleep(0.01)
        PiGPIO.write_byte_data(p, self.address, PWR_MGMT_1, 0x01)
        sleep(0.01)
        
        PiGPIO.write_byte_data(p, self.address, CONFIG, 0x03)
        PiGPIO.write_byte_data(p, self.address, SMPLRT_DIV, 0x04)
        PiGPIO.write_byte_data(p, self.address, GYRO_CONFIG, gfs<<3)
        PiGPIO.write_byte_data(p, self.address, ACCEL_CONFIG, afs<<3)
        PiGPIO.write_byte_data(p, self.address, ACCEL_CONFIG_2, 0x03)
        PiGPIO.write_byte_data(p, self.address, INT_PIN_CFG, 0x02)
        sleep(0.1)
    end
    
    function configAK8963(self, mode, mfs)
        if mfs == AK8963_BIT_14
            self.mres = 4912.0/8190.0
            else
            self.mres = 4912.0/32760.0
        end
        
        PiGPIO.write_byte_data(p, AK8963_DEVICE_ADDRESS, AK8963_CNTL1, 0x00)
        sleep(0.01)
        PiGPIO.write_byte_data(p, AK8963_DEVICE_ADDRESS, AK8963_CNTL1, 0x0F)
        sleep(0.01)
        
        data = PiGPIO.i2c_read_i2c_block_data(p, AK8963_DEVICE_ADDRESS, AK8963_ASAX, 3)
        
        self.magXcoeff = (data[0] - 128) / 256.0 + 1.0
        self.magYcoeff = (data[1] - 128) / 256.0 + 1.0
        self.magZcoeff = (data[2] - 128) / 256.0 + 1.0

        # Set power down mode
        PIGPIO.write_byte_data(p, AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x00)
        sleep(0.01)
        
        # Set scale and continous mode
        PiGPIO.write_byte_data(p, AK8963_SLAVE_ADDRESS, AK8963_CNTL1, (mfs<<4|mode))
        sleep(0.01)
    end
    
    function checkDataReady(self)
        drdy = PiGPIO.read_byte_data(p, self.address, INT_STATUS)
        if drdy & 0x01
            return True
            else
            return False
        end
    end
    
     function readAccel(self)
        data = PiGPIO.read_i2c_block_data(p, self.address, ACCEL_OUT, 6)
        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.ares, 3)
        y = round(y*self.ares, 3)
        z = round(z*self.ares, 3)

        return {"x": -10*y, "y": -10*x, "z": 10*z}
    end
    
    function readGyro(self)
        data = PiGPIO.read_i2c_block_data(p, self.address, GYRO_OUT, 6)

        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.gres, 3)
        y = round(y*self.gres, 3)
        z = round(z*self.gres, 3)

        return {"x":y, "y":x, "z":-z}
    end
    
    function readMagnet(self)
        x=0
        y=0
        z=0

        drdy = PiGPIO.read_byte_data(p, AK8963_SLAVE_ADDRESS, AK8963_ST1)
        if drdy & 0x01
            data = bus.read_i2c_block_data(AK8963_SLAVE_ADDRESS, AK8963_MAGNET_OUT, 7)

            # To check overflow
            if (data[6] & 0x08)!=0x08
                x = self.dataConv(data[0], data[1])
                y = self.dataConv(data[2], data[3])
                z = self.dataConv(data[4], data[5])

                x = round(x * self.mres * self.magXcoef, 3)
                y = round(y * self.mres * self.magYcoef, 3)
                z = round(z * self.mres * self.magZcoef, 3)
            end
        end

        return {"x":x, "y":y, "z":z}
    end
    
    function dataConv(self, data1, data2)
        value = data1 | (data2 << 8)
        if(value & (1 << 16 - 1)):
            value -= (1<<16)
        return value
        end
    end
end
