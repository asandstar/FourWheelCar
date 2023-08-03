import time
from machine import UART
from maix import camera, display, image
import math

# 初始化串口
uart = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)

# 定义控制状态
STATE_STRAIGHT = 1
STATE_LEFT = 2
STATE_RIGHT = 3
STATE_LEFT_LITTLE = 4
STATE_RIGHT_LITTLE = 5
STATE_MEETTEN = 6#遇见十字路口
STATE_MEETDING = 7#遇见丁字路口

# 初始化状态
state = STATE_STRAIGHT

while True:
    # 捕获摄像头图像
    img = camera.capture()
    
    # 对图像进行二值化处理
    img_gray = img.to_grayscale()
    img_binary = img_gray.threshold(120)
    
    # 查找直线并绘制
    lines = img_binary.find_lines((0, 0, 240, 240), 1, 1, 5, 10, 10)
    #(0, 0, 240, 240)：ROI区域，表示从图像左上角(0, 0)到右下角(240, 240)的矩形区域；
    #1：最小直线长度，表示检测到的直线长度至少为1个像素；
    #1：最大线段间隔，表示线段之间的间隔不超过1个像素；
    #5：直线数量，表示最多检测出5条直线；
    #10：直线角度阈值范围，表示直线偏离期望角度（水平或垂直）的角度范围不超过10度；
    #10：直线距离阈值范围，表示直线距离期望位置（ROI区域中心）的距离范围不超过10个像素。
    for l in lines:
        img.draw_line(l[0], l[1], l[2], l[3], color=(0, 255, 0))

    # 显示处理后的图像
    display.show(img)
'''
    #走直线
    if len(lines) > 0:
        line = lines[0]
        center = (line[0] + line[2]) // 2
        if center < 110:
            uart.write('4')
        elif center > 130:
            uart.write('5')
        else:
            uart.write('1')
    else:
        uart.write('1')




    # 判断十字路口，根据检测到的两条线夹角是否接近90度
    if len(lines) == 2:
        line1 = lines[0]
        line2 = lines[1]
        angle = abs(line1.theta() - line2.theta())
        if angle > 80 and angle < 100:
            uart.write('6')
            pass

    # 判断丁字路口，根据检测到一条长直线和两条短直线判断
    # 检测一条长直线和两条短直线
    long_line = None
    short_lines = []
    if len(lines) > 2 :
        for r in lines:
            x1, y1, x2, y2 = r
            length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            if length > 240:  # 长直线
                long_line = r
            elif length > 50:  # 短直线
                short_lines.append(r)
        # 判断是否为丁字路口
        if long_line is not None and len(short_lines) == 2:
            line1 = short_lines[0]
            line2 = short_lines[1]
            x1, y1, x2, y2 = long_line.line()
            for line in [line1, line2]:
                x1_, y1_, x2_, y2_ = line.line()
                if (x1 - x1_) * (x1 - x2_) < 0 and (y1 - y1_) * (y1 - y2_) < 0:
                    # 检测到丁字路口
                    uart.write('7')
                    break




    # 根据状态向串口发送控制指令
    if state == STATE_STRAIGHT:
        uart.write('3')
    elif state == STATE_LEFT:
        uart.write('1')
    elif state == STATE_RIGHT:
        uart.write('2')
    elif state == STATE_LEFT_LITTLE:
        uart.write('4')
    elif state == STATE_RIGHT_LITTLE:
        uart.write('5')
    
    #sleepms(10)，没有该方法
    delay_mark = time.time()    
    while True:
        offset = time.time() - delay_mark
        if offset > 0.0011:
            break
'''