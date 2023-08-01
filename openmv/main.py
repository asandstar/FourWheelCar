THRESHOLD = (28, 0, -55, 14, -14, 14) # Grayscale threshold for dark things...
#(13, 58, 11, 127, -128, 127) blue
#(6, 65, 30, 105, 10, 49)  red
# (28, 0, -55, 14, -14, 14) black
import sensor, image, time
from pyb import LED
from pid import PID
import car
rho_pid = PID(p=0.4, i=0)
theta_pid = PID(p=0.3, i=0)

LED(1).on()
LED(2).on()
LED(3).on()

sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
sensor.set_windowing((10,20,60,40))
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.

while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD])
    line = img.get_regression([(100,100)], robust = True)
    if (line):
        rho_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color = 127)
        print(rho_err,line.magnitude(),rho_err)
#        print(line.magnitude())
        if line.magnitude()>8:
            #if -40<b_err<40 and -30<t_err<30:
            rho_output = rho_pid.get_pid(rho_err,1)
            theta_output = theta_pid.get_pid(theta_err,1)
            output = rho_output+theta_output
            if(output >20):
                car.trans('f','R','B')
            elif output > 5:
                car.trans('f','R','S')
            elif output > -5:
                car.trans('f','N','S')
            elif output < -5 and output > -20:
                car.trans('f','L','S')
            elif output < -20 :
                car.trans('f','L','B')

        else:
            car.trans('s','N','S')
    else:
        car.trans('f','L','B')
        time.sleep_ms(300)
        car.trans('s','N','S')
    time.sleep_ms(100)
    #print(clock.fps())
#print("over")
