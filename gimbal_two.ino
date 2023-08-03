#include <PinChangeInterrupt.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define XTAL_FREQ 25495000  // 我们手上的I2C舵机模块的内置晶振频率是25.9MHz
// 根据转向舵机和云台舵机的参数 设定脉宽范围
// #define SERVO_MIN_US 500   // 输入的PWM高电平脉宽最小为500us
// #define SERVO_MAX_US 2500  // 输入的PWM高电平脉宽最大为2500us

//gimbal_up
#define SERVO_UP_MIN_US 930   //激光灯摆放较为合适的位置的下限脉宽，最下
#define SERVO_UP_MAX_US 1330     //激光灯摆放较为合适的位置的上限脉宽，最上
#define SERVO_UP_MID_US 1130  // 接近上臂竖直的脉宽，中立位

//gimbal_down
#define SERVO_DOWN_MIN_US 930   //下云台较为合适的位置的下限脉宽，最右（中立位右偏45度）
#define SERVO_DOWN_MAX_US 1930     //下云台较为合适的位置的上限脉宽，最左（中立位左偏45度）
#define SERVO_DOWN_MID_US 1430  // 中立位

//16路i2c引脚和频率设定
#define SERVO_FREQ 50    // 舵机PWM的频率是50Hz
#define CHANNEL_GIMBAL_UP 15 //云台上层舵机脉冲 通道范围0~15
#define CHANNEL_GIMBAL_DOWN 12 //云台下层舵机脉冲

// 16路I2C舵机驱动模块默认地址0x40
// 如果硬件上做了修改进而修改了其从机地址如0x42
// 那么用法为Adafruit_PWMServoDriver(0x42);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setServoPulse(uint8_t channel, double pulse_us) {
  double pulselength;
  pulselength = 1000000;             // 1s有1,000,000us
  pulselength /= SERVO_FREQ;         // 每个PWM波的周期 以us为单
  pulselength /= 4096;               // 实际上一个周期固定是4096的长度 具体时长由setPWMFreq以及
                                     // setOscillatorFrequency决定 算出每个数对应的时长
  pulse_us /= pulselength;           // 算出正宽归一化到4096究竟占多少
  pwm.setPWM(channel, 0, pulse_us);  //  利用Adafruit库的setPWM库设置channel从0~pulse_us
                                     // 高电平 再然后全是低电平
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("start");
  pwm.begin();
  // 需要用示波器确认一下晶振的频率是多少Hz 才能保证PWM的频率够准确
  pwm.setOscillatorFrequency(XTAL_FREQ);  // 我们手上的i2c duoji晶振频率是25.8MHz
  pwm.setPWMFreq(SERVO_FREQ);             // 模块可接受频率40~1000 软件库可接受频率1~3500往高了不见得能实现

}

void loop() {
  // put your main code here, to run repeatedly:
//中立位
  setServoPulse(CHANNEL_GIMBAL_UP, SERVO_UP_MID_US);
  setServoPulse(CHANNEL_GIMBAL_DOWN, SERVO_DOWN_MID_US);
  delay(1000);
  //先断舵机的电，再断连arduino防止断电后脱离中立位

}




  //云台双舵机测试
  // //适合放激光笔的几个角度之一
  // setServoPulse(CHANNEL_GIMBAL_UP, 930);
  // delay(1000);
  // //中立位
  // setServoPulse(CHANNEL_GIMBAL_UP, 1130);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_UP, 1330);
  // delay(1000);
  // // setServoPulse(CHANNEL_GIMBAL_UP, 1330);
  // // delay(1000);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_UP, 1130);
  // delay(1000);

  // setServoPulse(CHANNEL_GIMBAL_DOWN, 130);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 330);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 530);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 730);
  // delay(1000);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 930);
  // delay(3000); 
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 1430);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 1930);
  // delay(3000);
  // delay(3000);
  //以上云台有线处为反方向，下云台逆时针数值变大，也就是左边数值大于右边
  //低头数值变小，抬头数值变大
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 530);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 330);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 130);
  // delay(1000);
  // delay(1000);
  // delay(1000);

  //start
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 930);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 1130);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 1330);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 1530);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 1730);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 1930);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 1530);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 1130);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 730);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 330);
  // delay(1000);
  // //后面的不转了
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 2130);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 2330);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_DOWN, 2530);
  // delay(1000);
