from maix import camera, display, image, gpio
import serial, time

set_LAB = [[(84, 96, -128, -9, -3, 59)], # green
           [(37, 64, 32, 121, -7, 53)], #red
           ]
camera.config(size=(240,240), _ai_size=(240, 240), exp_gain=(100, 10))

ser = serial.Serial("/dev/ttyS1", 115200, timeout=0.2)  # 连接串口
tmp = ser.readline()

now_time = time.time()
x_center = y_center = 0
flag = '0'
while 1:
    img = camera.capture()
    for j in range(len(set_LAB)):
        blobs = img.find_blobs(set_LAB[j]) 
        if blobs:
            for i in blobs:
            #for b in blobs:
                x_start = i["x"]
                x_end = i["x"] + i["w"]
                x_center = (x_start + x_end) / 2 # 中心坐标
                y_start = i["y"]
                y_end = i["y"] + i["h"]
                y_center = (y_start + y_end) /  2                
                img.draw_circle(int(x_center), int(y_center), int(i["h"] / 4 + 4), color=(255, 0, 0),
                                    thickness=2)  # 画一个中心点在（50,50），半径为20的空心圆
                # if (j == 0):
                #     string = 'Red'
                # elif j == 1:
                #     string = 'Blue'
                # str_size = image.get_string_size(string)
                # img.draw_string(x_center - int(str_size[0] / 2) - 5, y_start - 35, string, scale=1.5,
                #                         thickness=2)
                if flag == '1':
                    flag = '0'
                    now_time = time.time()  # time.asctime()
                #img.draw_rectangle(b["x"], b["y"], b["x"] + b["w"], b["y"] + b["h"], color=(255, 0, 0), thickness = 2)  # rect
    display.show(img)
    time_t = time.time() - now_time
    if (time_t > 0.03):
        #print(x_center,y_center)
        if flag == '0':
            tep = "x:" + str(x_center)   # 先获得x数据
            #print(tep)
            ser.write(tep.encode("ascii"))  # 变成串口能发送的格式发出去
            flag = 'x'  # 先发X再发Y，需要等等不然会反映不过来，但又不能阻塞

        elif flag == 'x':
            flag = '1'
            tep = "y:" + str(y_center)   # 获得y数据
            #print(tep)
            ser.write(tep.encode("ascii"))  # 变成串口能发送的格式发出去