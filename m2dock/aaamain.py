# #!/usr/bin/env python
# from maix import camera, display, image, nn
# image.load_freetype("/home/res/sans.ttf")
# qrcode = image.open('/home/res/qrcode.png')
# canvas = image.new((display.width(), display.height()), (0xFF, 0xFF, 0xFF), "RGB")
# canvas.draw_image(qrcode, (canvas.width - qrcode.width) // 2, (canvas.height - qrcode.height) // 2)
# info = "wiki.sipeed.com/maixpy3"
# w, h = image.get_string_size(info, 1.2)
# canvas.draw_string((canvas.width - w) // 2 + 5, canvas.height - h - 5, info, 1.2, color=(0x00, 0x00, 0x00))
# for i in range(120):
#     img = camera.capture().draw_image(canvas, alpha=0.7)
#     display.show(img)


class Number_recognition:
    labels = ["1", "2", "3", "4", "5", "6", "7", "8"]
    anchors = [2.44, 2.25, 5.03, 4.91, 3.5 , 3.53, 4.16, 3.94, 2.97, 2.84]
    model = {
        "param": "/root/number_awnn.param",
        "bin": "/root/number_awnn.bin"
    }
    options = {
        "model_type":  "awnn",
        "inputs": {
            "input0": (224, 224, 3)
        },
        "outputs": {
            "output0": (7, 7, (1+4+len(labels))*5)
        },
        "mean": [127.5, 127.5, 127.5],
        "norm": [0.0078125, 0.0078125, 0.0078125],
    }
    w = options["inputs"]["input0"][1]
    h = options["inputs"]["input0"][0]
    def __init__(self):
        from maix import nn
        from maix.nn import decoder
        self.m = nn.load(self.model, opt=self.options)
        self.yolo2_decoder = decoder.Yolo2(len(self.labels), self.anchors, net_in_size=(self.w, self.h), net_out_size=(7, 7))
    def map_face(self, box):                           #将224*224空间的位置转换到240*240空间内
        def tran(x):
            return int(x/224*240)
        box = list(map(tran, box))
        return box
print(Number_recognition)

from gpiod import chip, line, line_request
import time
from maix import camera, display, image
import math
import serial

uart = serial.Serial("/dev/ttyS1",9600)
number_recognition = Number_recognition()

STATE_STRAIGHT ='a'
STATE_LEFT = 'b'
STATE_RIGHT = 'c'
STATE_LEFT_LITTLE = 'q'
STATE_RIGHT_LITTLE = 'r'
STATE_DIAOTOU = 'd'
STATE_STOP = 'e'


state = 'b'


gpiochip1 = chip("gpiochip1")


def pin(gpio="PD", pos=11):
    return 32 * (ord(gpio.lower()[1]) - ord('a')) + pos


def gpio(line_offset=(224 + 14), line_mode=line_request.DIRECTION_OUTPUT):  # "default PH14 OUTPUT"
    try:
        tmp = None
        tmp = gpiochip1.get_line(line_offset)
        config = line_request()  # led.active_state == line.ACTIVE_LOW
        config.request_type = line_mode  # line.DIRECTION_INPUT
        tmp.request(config)
    except Exception as e:
        print(e)
    finally:
        return tmp

def readsensor(clk,dat):
    while True:
        ret = 0
        for i in range(8):
            # GPIO.set_pin(GW_GRAY_GPIO_CLK, GPIO.LOW)
            clk.set_value(0)  # 输出时钟下降沿
            delay_mark = time.time()
            while True:
                offset = time.time() - delay_mark
                if offset > 0.00005:
                    break
            ret |= dat.get_value() << i  # 读取数据并存到ret的第i位bit
            clk.set_value(1)  # 输出时钟上升沿

            while True:
                offset = time.time() - delay_mark
                if offset > 0.00005:
                    break
        return ret


def getnum():#最最开始的时候扫描得到的目标病房号
    img = camera.capture()
    display.show(img)
    AI_img = img.copy().resize(224, 224)
    out = number_recognition.m.forward(AI_img.tobytes(), quantize=True, layout="hwc")
    boxes, probs = number_recognition.yolo2_decoder.run(out, nms=0.3, threshold=0.5, img_size=(240, 240))
    for i, box in enumerate(boxes):
        class_id = probs[i][0]
        prob = probs[i][1][class_id]
        # box[0]横坐标，1纵坐标，2宽度，3长度
        if (prob >0.8):
                return number_recognition.labels[class_id]
    return 0





def firstcross(aim):
    if aim == 1:
        state2 = STATE_LEFT
        uart.write(b'b')
        time.sleep(0.05)
    if aim == 2:
        state2 = STATE_RIGHT
        uart.write(b'c')
        time.sleep(0.05)
    else:
        state2 = STATE_STRAIGHT


def secondthirdcross(aim):  # 以后反复数字、位置，判断是否有目标病房，如果有记下左右
    img = camera.capture()
    AI_img = img.copy().resize(224, 224)
    out = number_recognition.m.forward(AI_img.tobytes(), quantize=True, layout="hwc")
    boxes, probs = number_recognition.yolo2_decoder.run(out, nms=0.3, threshold=0.5, img_size=(240, 240))
    while boxes is not None:
        stopgo = 0  # 标志检测到的数字中是否有目标病房，如果有将stopgo设置为目标病房的box[0]
        newboxesnum = 0  # 检测到几个数字，理应说只有2or4
        newboxeslist = []
        for i, box in enumerate(boxes):
            class_id = probs[i][0]
            prob = probs[i][1][class_id]
            box = number_recognition.map_face(box)
            # box[0]横坐标，1纵坐标，2宽度，3长度
            if (prob > 0.65):
                newboxeslist.append(box[0])
                newboxesnum += 1
                if (number_recognition.labels[class_id] == aim):  # aim是第一次识别到的目标病房号
                    stopgo = box[0]
        # 如果没有检测到目标病房，直走
        if (stopgo == 0):
            state2 = STATE_STRAIGHT  # 为了区分灰度传感器设置的state状态，单独设一个state2状态，且state2优先级更高，若state2命令直走，则state命令停止时，即停即走
            uart.write(b'a')
            time.sleep(0.05)
        # 如果检测到目标病房，设置state2告知下个路口左转or右转
        else:
            newboxeslist.sort(reverse=False)  # 按照box[0]从小到大排列，即从左到右排列
            relativelocation = newboxeslist.index(stopgo)
            # 判断目标病房在序列的左边还是右边
            if relativelocation < (newboxesnum / 2):
                state2 = STATE_LEFT
                # 先前走几秒
                uart.write(b'b')
                time.sleep(0.05)
            else:
                state2 = STATE_RIGHT
                # 先前走几秒
                uart.write(b'c')
                time.sleep(0.05)

        newboxeslist.clear()


#---------------------------------------------开始-------------------------------------------
clk = gpio(pin("PH", 14), line_request.DIRECTION_OUTPUT)
dat = gpio(pin("PH", 13), line_request.DIRECTION_INPUT)

aim =0
while aim == 0:
    aim = getnum()


while True:
    sensor_data = readsensor(clk,dat)
    if sensor_data == 0b11100111 or sensor_data == 0b11101111 or sensor_data == 0b11110111:
        state = STATE_STRAIGHT
    elif sensor_data == 0b11000111 or sensor_data == 0b11001111 or sensor_data == 0b11011111 or se