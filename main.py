from maix import camera, display, image
import serial, time

set_LAB = [[(84, 96, -128, -9, -3, 59)], # green
           [(37, 64, 32, 121, -7, 53)], #red
           ]
camera.config(size=(240,240), _ai_size=(240, 240), exp_gain=(100, 10))

ser = serial.Serial("/dev/ttyS1", 115200, timeout=0.2)  # 连接串口
tmp = ser.readline()
now_time = time.time()
x_center = y_center = 0

# def find_max(blobs):
#     max_size=0
#     for blob in blobs:
#         if blob[2]*blob[3] > max_size:
#             max_blob=blob
#             max_size = blob[2]*blob[3]
#     return max_blob
flag = '0'
while 1:
    img = camera.capture()
    for j in range(len(set_LAB)):
        blobs = img.find_blobs(set_LAB[j]) 
        if blobs:
            # #max_blob = find_max(blobs)
            for b in blobs:
                x_center = b["centroid_x"]    
                y_center = b["centroid_y"]    
                img.draw_rectangle(b["x"], b["y"], b["x"] + b["w"], b["y"] + b["h"], color=(255, 0, 0), thickness = 2)  # rect
                if flag == '1':
                    flag = '0'
                now_time = time.time()  # time.asctime()
    display.show(img)
    time_t = time.time() - now_time
    if (time_t > 0.1):
        #print(x_center,y_center)
        if flag == '0':
            tep = "x" + str(x_center)   # 先获得x数据
            #print(tep)
            ser.write(tep.encode("ascii"))  # 变成串口能发送的格式发出去
            flag = 'x'  # 先发X再发Y，需要等等不然会反映不过来，但又不能阻塞

        elif flag == 'x':
            flag = '1'
            tep = "y" + str(y_center)   # 获得y数据
            #print(tep)
            ser.write(tep.encode("ascii"))  # 变成串口能发送的格式发出去