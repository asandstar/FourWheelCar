from pyb import Pin, Timer
inverse_left=False  #change it to True to inverse left wheel
inverse_right=False #change it to True to inverse right wheel
from machine import UART
serial = UART(3,9600)

#ain1 =  Pin('P0', Pin.OUT_PP)
#ain2 =  Pin('P1', Pin.OUT_PP)
#bin1 =  Pin('P2', Pin.OUT_PP)
#bin2 =  Pin('P3', Pin.OUT_PP)
#ain1.low()
#ain2.low()
#bin1.low()
#bin2.low()

#pwma = Pin('P7')
#pwmb = Pin('P8')
#tim = Timer(4, freq=1000)
#ch1 = tim.channel(1, Timer.PWM, pin=pwma)
#ch2 = tim.channel(2, Timer.PWM, pin=pwmb)
#ch1.pulse_width_percent(0)
#ch2.pulse_width_percent(0)

POST = "@"

def trans(move,direction,round):
    str = POST
    str = str + move + direction + round +'\n'
    serial.write(str.encode('ascii'))




