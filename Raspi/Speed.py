import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

pwm = GPIO.PWM(18,50)

pwm.start(7.5)

def servo(ang):
    try:
        angle_end = ang
        for i in range (1,100):
            angle_step_end = (i*angle_step_end) + 2.5
            p.ChangeDutyCycle(duty_cycle)
            
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()