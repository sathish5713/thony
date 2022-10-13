import utime
import machine
from time import sleep
from machine import Pin
import micropython
import rp2
import os
import utime as time
from machine import Pin, Timer
from machine import Pin,UART
from machine import ADC
from machine import Pin
from dht import DHT11, InvalidChecksum
from utime import sleep_us,ticks_us
from machine import I2C
from lcd_api import LcdApi
from pico_i2c_lcd import I2cLcd
from machine import PWM

tm=10 #sleep btw each data transfer (in milli seconds)
tim=1 #sleep after each full set transfer (in seconds)

moist_threshold=50 #Range(0-50) Moisture

pwm = PWM(Pin(18))
pwm.freq(50)
    
I2C_ADDR     = 0x27
I2C_NUM_ROWS = 4
I2C_NUM_COLS = 20

Turb = Pin(2, Pin.IN, Pin.PULL_UP)
Rain = Pin(3, Pin.IN, Pin.PULL_UP)
pin = Pin(13, Pin.OUT, Pin.PULL_DOWN)
echo= Pin(10, Pin.IN)  
trig= Pin(11, Pin.OUT)
m_pin = Pin(8, Pin.OUT)

i2c = I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)
trig.value(0)
uart = UART(1,9600)
adc = ADC(Pin(26))
sensor = DHT11(pin)
sleep_us(2)

MIN=1000000
MAX=2000000

lcd.clear()
lcd.move_to(0,0)
lcd.putstr("Temp:")
lcd.move_to(9,0)
lcd.putstr("M:")
lcd.move_to(13,0)
lcd.putstr("T:")

lcd.move_to(0,1)
lcd.putstr("M:")
lcd.move_to(6,1)
lcd.putstr("U:")
lcd.move_to(13,1)
lcd.putstr("R:")

time.sleep(1)

while True:
    
                
    try:  
        trig.high()  
        sleep_us(10)  
        trig.low()  
        while echo.value()==False:  
            st=ticks_us()  
        while echo.value()==True:  
            sto=ticks_us()  
        tt=sto-st  
        dis=(0.031594*tt)/2
        dis=int(dis)
        if(dis>100):
            dis=100
        lcd.move_to(8,1)
        lcd.putstr("   ")
        lcd.move_to(8,1)
        lcd.putstr(str(dis))
        
        
                
    except KeyboardInterrupt:  
        break
    
    pin = Pin(13, Pin.OUT, Pin.PULL_DOWN)
    sensor = DHT11(pin)
    t = (sensor.temperature)
    t=int(t)
    lcd.move_to(5,0)
    lcd.putstr(str(t))
    #h = (sensor.humidity)
    #h=int(h)
    #lcd.move_to(2,1)
    #lcd.putstr(str(h))
    
    m=adc.read_u16()
    m=int(100-(m/656))
    lcd.move_to(2,1)
    lcd.putstr(str(m))
    if(m > moist_threshold):
        #m_pin.value(1)
        pwm.duty_ns(MAX)
        utime.sleep(1)
    else:
        #m_pin.value(0)
        pwm.duty_ns(MIN)
        utime.sleep(1)
    
    if Turb.value()==0:
        lcd.move_to(15,0)
        lcd.putstr("Y")
        Turbval=1
    else:
        lcd.move_to(15,0)
        lcd.putstr("N")
        Turbval=0
        
    if Rain.value()==0:
        lcd.move_to(15,1)
        lcd.putstr("Y")
        Rainval=1
    else:
        lcd.move_to(15,1)
        lcd.putstr("N")
        Rainval=0
        
   
    if(m < 50):
        if(dis < 11):
            if(t > 30):
                m_pin.value(1)
                Motorval=1
                lcd.move_to(11,0)
                lcd.putstr("Y")
                
            else:
                m_pin.value(0)
                Motorval=0
                lcd.move_to(11,0)
                lcd.putstr("N")
        else:
            m_pin.value(0)
            Motorval=0
            lcd.move_to(11,0)
            lcd.putstr("N")
    else:
        m_pin.value(0)
        Motorval=0
        lcd.move_to(11,0)
        lcd.putstr("N")
    
    
    uart.write("I")
    uart.write(str(t))
    uart.write(" ")
    uart.write(str(Rainval))
    uart.write(" ")
    uart.write(str(m))
    uart.write(" ")
    uart.write(str(dis))
    uart.write(" ")
    uart.write(str(Turbval))
    uart.write(" ")
    uart.write(str(Motorval))
    uart.write(" E ")
    
    utime.sleep(1)   

















