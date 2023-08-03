#include <PinChangeInterrupt.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define XTAL_FREQ 25495000  // 我们手上的I2C舵机模块的内置晶振频率是25.9MHz
// 根据转向舵机和云台舵机的参数 设定脉宽范围
// #define SERVO_MIN_US 500   // 输入的PWM高电平脉宽最小为500us
// #define SERVO_MAX_US 2500  // 输入的PWM高电平脉宽最大为2500us

#define SERVO_MIN_US 544   // 输入的PWM高电平脉宽最小为500us
#define SERVO_MAX_US 2400  // 输入的PWM高电平脉宽最大为2500us
//min(可选)，舵机为最小角度（0度）时的脉冲宽度，单位为微秒，默认为544
//max(可选)，舵机为最大角度（180度时）的脉冲宽度，单位为微秒，默认为2400
#define SERVO_MID_US 1505  // 输入的PWM中立位脉宽最为1530us 每个人的车不太一样 微调
// 以车在转弯半径中后轮的最靠近圆心一侧为基准测量转弯半径

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
  pwm.setPWM(channel, 0, pulse_us);  // 利用Adafruit库的setPWM库设置channel从0~pulse_us
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
  //云台双舵机测试
   setServoPulse(CHANNEL_GIMBAL_UP, 102);
   delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_UP, 187);
  // delay(1000);
   setServoPulse(CHANNEL_GIMBAL_UP, 280);
   delay(1000);  
  // setServoPulse(CHANNEL_GIMBAL_UP, 373);
  // delay(1000);
  setServoPulse(CHANNEL_GIMBAL_UP, 510);
  delay(1000);
  setServoPulse(CHANNEL_GIMBAL_UP, 900);
  delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_UP, 1000);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_UP, 1100);
  // delay(1000);
  setServoPulse(CHANNEL_GIMBAL_UP, 1140);
  delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_UP, 1200);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_UP, 1300);
  // delay(1000);
  // setServoPulse(CHANNEL_GIMBAL_UP, 1400);
  // delay(1000);

}
