import PiGPIO
using PiGPIO

p = Pi() # Connects your Raspberry Pi to pigpiod daemon on localhost:8888

pin = 18 # GPIO BCM number

INPUT = 0
OUTPUT = 1

LOW = 0
HIGH = 1

set_mode(p, pin, OUTPUT)

try 
    for i = 1:20
        PiGPIO.write(p, pin, HIGH)
        sleep(0.5)
        PiGPIO.write(p, pin, LOW)
        sleep(0.5)
    end
finally
    set_mode(p, pin, INPUT)
end
