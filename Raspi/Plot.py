import matplotlib.pyplot as plt
import time
import RPi.GPIO as GPIO
from time import sleep

plt.rcParams['animation.html'] = 'jshtml'

fig = plt.figure()
ax = fig.add_subplot(111)
fig.show()

i = 0
x, y = [], []

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

pwm = GPIO.PWM(18,50)

pwm.start(7.5)

def SetAngle(angle):
    duty = angle / 27 + 2  # define the duty cycle in percentage
    GPIO.output(18, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(18, False)
    pwm.ChangeDutyCycle(0)

SetAngle(0)
SetAngle(5)
SetAngle(10)
SetAngle(15)
SetAngle(20)
SetAngle(25)
SetAngle(30)
SetAngle(35)

while True:
    p = SetAngle(i)
    x.append(i)
    y.append(p)
    ax.plot(x,y, color='b')
    fig.canvas.draw()
    time.sleep(0.1)
    i += 1


#SetAngle(0)
#SetAngle(5)
#SetAngle(10)
#SetAngle(15)
#SetAngle(20)
#SetAngle(25)
#SetAngle(30)
#SetAngle(35)
#SetAngle(40)
#SetAngle(45)
#SetAngle(50)
#SetAngle(55)
#SetAngle(60)
#SetAngle(70)
#SetAngle(75)
#SetAngle(80)
#SetAngle(85)
#SetAngle(90)
#SetAngle(95)
#SetAngle(100)
#SetAngle(105)
#SetAngle(110)
#SetAngle(115)
#SetAngle(120)
#SetAngle(125)
#SetAngle(130)
#SetAngle(135)
#SetAngle(140)
#SetAngle(145)
#SetAngle(150)
#SetAngle(155)
#SetAngle(160)
#SetAngle(165)
#SetAngle(170)
#SetAngle(175)
#SetAngle(180)


pwm.stop()
GPIO.cleanup()
