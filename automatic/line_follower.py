from time import sleep
from machine import UART
import machine
from machine import Pin
from machine import ADC
from machine import PWM
import time

BT = UART(0,9600)
rightSensor = ADC(Pin(27))
leftSensor = ADC(Pin(26))
THRESHOLD = 50000
def stop():
   motor_A_Speed.duty_u16(0 * 650)
   motor_B_Speed.duty_u16(0 * 650)

   motor_A1.low()
   motor_A2.low()
   motor_B1.low()
   motor_B2.low()

'''motor_A_Speed = PWM(Pin(10))
motor_A_Speed.freq(1500)
motor_A1 = Pin(11,Pin.OUT)
motor_A2 = Pin(12,Pin.OUT)
motor_A3 = Pin(16,Pin.OUT)
motor_A4 = Pin(17,Pin.OUT)

motor_B_Speed = PWM(Pin(13))
motor_B_Speed.freq(1500)
motor_B1 = Pin(14,Pin.OUT)
motor_B2 = Pin(15,Pin.OUT)
motor_B3 = Pin(18,Pin.OUT)
motor_B4 = Pin(19,Pin.OUT)'''

#motorA
motor_A_Speed = PWM(Pin(10))
motor_A_Speed.freq(1500)
motor_A1 = Pin(11, Pin.OUT)
motor_A2 = Pin(12, Pin.OUT)

#motorB
motor_B_Speed = PWM(Pin(13))
motor_B_Speed.freq(1500)
motor_B1 = Pin(14, Pin.OUT)
motor_B2 = Pin(15, Pin.OUT)

'''#motorC
motor_C_Speed = PWM(Pin(10))
motor_C_Speed.freq(1500)
motor_C1 = Pin(16, Pin.OUT)
motor_C2 = Pin(17, Pin.OUT)

#motorD
motor_D_Speed = PWM(Pin(13))
motor_D_Speed.freq(1500)
motor_D1 = Pin(18, Pin.OUT)
motor_D2 = Pin(19, Pin.OUT)'''

def forward(speedForward):
   motor_A_Speed.duty_u16(speedForward * 650)
   motor_B_Speed.duty_u16(speedForward * 650)

   motor_A1.low()
   motor_A2.high()
   motor_B1.low()
   motor_B2.high()
   '''motor_C1.low()
   motor_C2.low()
   motor_D1.low()
   motor_D2.low()'''
   return

def Line():
    global right_sensor, leftSensor
    if leftSensor.read_u16() < THRESHOLD and rightSensor.read_u16() < THRESHOLD:
        stop()
    elif leftSensor.read_u16() < THRESHOLD and rightSensor.read_u16() > THRESHOLD:
        motor_A_Speed.duty_u16(100 * 650)
        motor_A1.low()
        motor_A2.high()
        '''motor_A3.low()
        motor_A4.low()'''
        motor_B_Speed.duty_u16(0 * 650)
        motor_B1.low()
        motor_B2.high()
        '''motor_B3.low()
        motor_B4.low()'''
    elif leftSensor.read_u16() > THRESHOLD and rightSensor.read_u16() < THRESHOLD:
        motor_A_Speed.duty_u16(70 * 650)
        motor_A1.low()
        motor_A2.high()
        #motor_A3.low()
        #motor_A4.low()
        motor_B_Speed.duty_u16(0 * 650)
        motor_B1.low()
        motor_B2.high()
        #motor_B3.low()
        #motor_B4.low()
    else:
        forward(70)


def setup():
    global right_sensor, leftSensor
    right_sensor = rightSensor.read_u16()
    right_sensor = leftSensor.read_u16()
    print("Left")
    print(leftSensor)
    time.sleep((0.5))
    print("Right")
    print(right_sensor)
    time.sleep((0.5))

while True:
    setup()
    Line()
