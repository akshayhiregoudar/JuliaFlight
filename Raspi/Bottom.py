# To rotate the bottom servo at a rate of 30 degrees per 19 seconds
# Connect to GPIO23 (Pin 16)

import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)

pwm = GPIO.PWM(23,50)

pwm.start(7.5)

def SetAngle(angle):
    duty = angle / 27 + 2  # define the duty cycle in percentage
    GPIO.output(23, True)
    pwm.ChangeDutyCycle(duty)
    sleep(19)
    GPIO.output(23, False)
    pwm.ChangeDutyCycle(0)
    
SetAngle(0)
SetAngle(30)
SetAngle(60)
SetAngle(90)
SetAngle(120)
SetAngle(150)
SetAngle(180)


pwm.stop()
GPIO.cleanup()