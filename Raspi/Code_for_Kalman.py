from KalmanFilter import Kalman
import smbus
import time
import math

KalmanX = Kalman()
KalmanY = Kalman()

RtoD = 57.2958
Kalman_Angle_X = 0
Kalman_Angle_Y = 0

# Some MPU6050 Registers and their address
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


# Read Gyroscope and Accelerometer values from MPU9250
def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    # Set the Digital Low Pass Filter (DLPF) by assigning the last three bit of 0X1A '110'. It removes the noise due to vibration.)
    bus.write_byte_data(Device_Address, CONFIG, int('0000110',2))

    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
    
        # Accelerometer and Gyro values are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)

        # Concatenate higher and lower value
        value = ((high << 8) | low)

        # To obtain signed value from MPU9250
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1)
Device_Address = 0x68   # MPU9250 device address

MPU_Init()

time.sleep(0.01)   # Start time

# Read Accelerometer raw value
accX = read_raw_data(ACCEL_XOUT_H)
accY = read_raw_data(ACCEL_YOUT_H)
accZ = read_raw_data(ACCEL_ZOUT_H)

# print(accX,accY,accZ)
# print(math.sqrt((accY**2)+(accZ**2)))

Restrict_Pitch = True    # Comment out to restrict roll to ±90deg

if (Restrict_Pitch):
    roll = math.atan2(accY,accZ) * RtoD
    pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * RtoD
else:
    roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * RtoD
    pitch = math.atan2(-accX,accZ) * RtoD
    
print(roll)


KalmanX.set_angle(roll)
KalmanY.set_angle(pitch)

Gyro_Angle_X = roll;
Gyro_Angle_Y = pitch;

Comp_Angle_X = roll;
Comp_Angle_Y = pitch;

timer = time.time()
flag = 0


while True:
    if(flag >1000):   # Problem with the connection
        print("There is a problem with the connection")
        flag=0
        continue
    try:
        # Read Accelerometer raw value
        accX = read_raw_data(ACCEL_XOUT_H)
        accY = read_raw_data(ACCEL_YOUT_H)
        accZ = read_raw_data(ACCEL_ZOUT_H)

        # Read Gyroscope raw value
        gyroX = read_raw_data(GYRO_XOUT_H)
        gyroY = read_raw_data(GYRO_YOUT_H)
        gyroZ = read_raw_data(GYRO_ZOUT_H)

        dt = time.time() - timer
        timer = time.time()

        if (Restrict_Pitch):
            roll = math.atan2(accY,accZ) * RtoD
            pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * RtoD
        else:
            roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * RtoD
            pitch = math.atan2(-accX,accZ) * RtoD

        Gyro_Rate_X = gyroX/131
        Gyro_Rate_Y = gyroY/131

        if (Restrict_Pitch):

            if((roll < -90 and Kalman_Angle_X >90) or (roll > 90 and Kalman_Angle_X < -90)):
                KalmanX.set_angle(roll)
                complAngleX = roll
                Kalman_Angle_X   = roll
                Gyro_Angle_X  = roll
            else:
                Kalman_Angle_X = KalmanX.get_angle(roll,Gyro_Rate_X,dt)

            if(abs(Kalman_Angle_X)>90):
                Gyro_Rate_Y  = -Gyro_Rate_Y
                Kalman_Angle_Y  = KalmanY.get_angle(pitch,Gyro_Rate_Y,dt)
        else:

            if((pitch < -90 and Kalman_Angle_Y >90) or (pitch > 90 and Kalman_Angle_Y < -90)):
                KalmanY.set_angle(pitch)
                complAngleY = pitch
                Kalman_Angle_Y   = pitch
                Gyro_Angle_Y  = pitch
            else:
                Kalman_Angle_Y = KalmanY.get_angle(pitch,Gyro_Rate_Y,dt)

            if(abs(Kalman_Angle_Y)>90):
                Gyro_Rate_X  = -Gyro_Rate_X
                Kalman_Angle_X = KalmanX.get_angle(roll,Gyro_Rate_X,dt)

        # Angle = (rate of change of angle) * change in time
        Gyro_Angle_X = Gyro_Rate_X * dt
        Gyro_Angle_Y = Gyro_Angle_Y * dt

        # compAngle = constant * (old_compAngle + angle_obtained from Gyroscope) + constant * angle_obtained from Accelerometer
        Comp_Angle_X = 0.93 * (Comp_Angle_X + Gyro_Rate_X * dt) + 0.07 * roll
        Comp_Angle_Y = 0.93 * (Comp_Angle_Y + Gyro_Rate_Y * dt) + 0.07 * pitch

        if ((Gyro_Angle_X < -180) or (Gyro_Angle_X > 180)):
            Gyro_Angle_X = Kalman_Angle_X
        if ((Gyro_Angle_Y < -180) or (Gyro_Angle_Y > 180)):
            Gyro_Angle_Y = Kalman_Angle_Y

        print("Angle X: " + str(Kalman_Angle_X)+"   " +"Angle Y: " + str(Kalman_Angle_Y))
        #print(str(roll)+"  "+str(Gyro_Angle_X)+"  "+str(Comp_Angle_X)+"  "+str(Kalman_Angle_X)+"  "+str(pitch)+"  "+str(Gyro_Angle_Y)+"  "+str(Comp_Angle_Y)+"  "+str(Kalman_Angle_Y))
        
        time.sleep(0.01)   # To set the signal frequency

    except Exception as exc:
        flag += 1