#include <PinChangeInterrupt.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define XTAL_FREQ 25495000  // 我们手上的I2C舵机模块的内置晶振频率是25.9MHz
// 根据转向舵机和云台舵机的参数 设定脉宽范围
#define SERVO_MIN_US 500   // 输入的PWM高电平脉宽最小为500us
#define SERVO_MAX_US 2500  // 输入的PWM高电平脉宽最大为2500us
#define SERVO_MID_US 1505  // 输入的PWM中立位脉宽最为1530us 每个人的车不太一样 微调
// 以车在转弯半径中后轮的最靠近圆心一侧为基准测量转弯半径
#define TRACK_WIDTH 160.0          // 胎距160mm
#define WHEELBASE 143.0            // 轴距143mm
#define TIRE_WIDTH 26.0            // 胎宽26mm
#define TURNIN_RADIUS_SMALL 160.0  // 方向打得比较大时转弯半径160mm
#define SERVO_LEFT_TRS_US 1030     // 左转弯半径160mm时转向舵机PWM脉宽
#define SERVO_RIGHT_TRS_US 2280    // 右转弯半径160mm时转向舵机PWM脉宽
#define TURNIN_RADIUS_LARGE 320.0  // 方向打得比较小时转弯半径320mm
#define SERVO_LEFT_TRL_US 1205     // 左转弯半径320mm时转向舵机PWM脉宽
#define SERVO_RIGHT_TRL_US 1880    // 右转弯半径320mm时转向舵机PWM脉宽

#define SERVO_FREQ 50    // 舵机PWM的频率是50Hz
#define CHANNEL_STEER 8  //默认我设在8通道 通道范围0~15

#define TARGET_SPEED_BASE 100  //某个速度

// 平衡小车shield的相关引脚定义
#define KEY 3   //按键引脚
#define IN1 12  //TB6612FNG驱动模块控制信号 共6个
#define IN2 13
#define IN3 7
#define IN4 6
#define PWMA 10
#define PWMB 9
#define ENCODER_L 4  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 8
#define ENCODER_R 2
#define DIRECTION_R 5

// 方向盘的5种可选位置
enum STEERING {
  LEFT_LARGE,
  LEFT_SMALL,
  NEUTRAL,
  RIGHT_SMALL,
  RIGHT_LARGE,
};

// 不同转弯半径下内外轮速度比
float SPEED_RATE_TRS = (TRACK_WIDTH + TIRE_WIDTH / 2.0 + TURNIN_RADIUS_SMALL) / (TURNIN_RADIUS_SMALL + TIRE_WIDTH / 2.0);
float SPEED_RATE_TRL = (TRACK_WIDTH + TIRE_WIDTH / 2.0 + TURNIN_RADIUS_LARGE) / (TURNIN_RADIUS_LARGE + TIRE_WIDTH / 2.0);

// 16路I2C舵机驱动模块默认地址0x40
// 如果硬件上做了修改进而修改了其从机地址如0x42
// 那么用法为Adafruit_PWMServoDriver(0x42);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int Motor1, Motor2;                                      //电机叠加之后的PWM
volatile long Velocity_L, Velocity_R = 0;                //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0;                   //左右轮速度
volatile float Target_speed_left, Target_speed_right = 0;  //左右论期望速度
float* Velocity_Pwm;                                     //速度环的PWM
double Velocity_Kp = 2, Velocity_Ki = 0.001;              //左右轮速度控制器P和I参数
float Battery_Voltage;                                   //电池电压 单位是V
unsigned char Flag_Stop = 1;                             //停止标志位

unsigned char Turn_Off(float voltage) {
  unsigned char temp;
  if (1 == Flag_Stop || voltage < 11.1)  //电池电压低于11.1V关闭电机 || Flag_Stop置1关闭电机
  {
    temp = 1;
    analogWrite(PWMA, 0);  //PWM输出为0
    analogWrite(PWMB, 0);  //PWM输出为0
  } else temp = 0;         //不存在异常，返回0
  return temp;
}

unsigned char My_click(void) {
  static unsigned char flag_key = 1;  //按键按松开标志
  unsigned char Key;
  Key = digitalRead(KEY);    //读取按键状态
  if (flag_key && Key == 0)  //如果发生单击事件
  {
    flag_key = 0;
    return 1;  // 单击事件
  } else if (1 == Key) flag_key = 1;
  return 0;  //无按键按下
}

float* velocity(int encoder_left, int encoder_right) {
  static float Velocity[2], Encoder_LeastL, EncoderL, Encoder_LeastR, EncoderR;
  static float Encoder_IntegralL, Encoder_IntegralR;

  Encoder_LeastL = Target_speed_left - encoder_left;                       //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
  EncoderL *= 0.7;                                                         //===一阶低通滤波器
  EncoderL += Encoder_LeastL * 0.3;                                        //===一阶低通滤波器
  Encoder_IntegralL += EncoderL;                                           //===积分出位移 积分时间：40ms
  if (Encoder_IntegralL > 21000) Encoder_IntegralL = 21000;                //===积分限幅
  if (Encoder_IntegralL < -21000) Encoder_IntegralL = -21000;              //===积分限幅
  Velocity[0] = EncoderL * Velocity_Kp + Encoder_IntegralL * Velocity_Ki;  //===速度控制

  Encoder_LeastR = Target_speed_right- encoder_right;                     //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
  EncoderR *= 0.7;                                                         //===一阶低通滤波器
  EncoderR += Encoder_LeastR * 0.3;                                        //===一阶低通滤波器
  Encoder_IntegralR += EncoderR;                                           //===积分出位移 积分时间：40ms
  if (Encoder_IntegralR > 21000) Encoder_IntegralR = 21000;                //===积分限幅
  if (Encoder_IntegralR < -21000) Encoder_IntegralR = -21000;              //===积分限幅
  Velocity[1] = EncoderR * Velocity_Kp + Encoder_IntegralR * Velocity_Ki;  //===速度控制

  if (Turn_Off(Battery_Voltage) == 1 || Flag_Stop == 1) {
    Encoder_IntegralL = 0;  //小车停止的时候积分清零
    Encoder_IntegralR = 0;
  }

  return Velocity;
}


void Set_Pwm(int moto1, int moto2) {
  if (moto1 > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);  //TB6612的电平控制
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);  //TB6612的电平控制
  }
  analogWrite(PWMA, abs(moto1));  //赋值给PWM寄存器
  if (moto2 < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);  //TB6612的电平控制
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);  //TB6612的电平控制
  }
  analogWrite(PWMB, abs(moto2));  //赋值给PWM寄存器
}

void Xianfu_Pwm(void) {
  int Amplitude = 250;  //===PWM满幅是255 限制在250
  if (Motor1 < -Amplitude) Motor1 = -Amplitude;
  if (Motor1 > Amplitude) Motor1 = Amplitude;
  if (Motor2 < -Amplitude) Motor2 = -Amplitude;
  if (Motor2 > Amplitude) Motor2 = Amplitude;
}

void control() {
  static int Velocity_Count, Turn_Count, Encoder_Count;
  static float Voltage_All, Voltage_Count;
  int Temp;
  sei();  //全局中断开启

  if (++Velocity_Count >= 8)  //速度控制，控制周期40ms
  {
    Velocity_Left = Velocity_L;
    Velocity_L = 0;  //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
    Velocity_Right = Velocity_R;
    Velocity_R = 0;                                          //读取右轮编码器数据，并清零
    Velocity_Pwm = velocity(Velocity_Left, Velocity_Right);  //速度PI控制，控制周期40ms
    Velocity_Count = 0;
  }

  Motor1 = Velocity_Pwm[0];  //直立速度转向环的叠加
  Motor2 = Velocity_Pwm[1];  //直立速度转向环的叠加

  Xianfu_Pwm();             //限幅
  if (Turn_Off(Battery_Voltage) == 0) {
    Set_Pwm(Motor1, Motor2); //如果不存在异常，赋值给PWM寄存器控制电机
  }     
  if (My_click()) {
    Flag_Stop = !Flag_Stop;  //中断剩余的时间扫描一下按键状态
  }
  Temp = analogRead(0);  //采集一下电池电压
  Voltage_Count++;       //平均值计数器
  Voltage_All += Temp;   //多次采样累积
  if (Voltage_Count == 200) {
    Battery_Voltage = Voltage_All * 0.05371 / 200;
    Voltage_All = 0;
    Voltage_Count = 0;  //求平均值
  }
}

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


void steerCar(enum STEERING choice, bool forward) {
  switch (choice) {
    case LEFT_LARGE:
      {
        // 方向盘左打满
        setServoPulse(CHANNEL_STEER, SERVO_LEFT_TRS_US);
        // 后轮电机左小右大
        Target_speed_left = TARGET_SPEED_BASE;
        Target_speed_right = TARGET_SPEED_BASE * SPEED_RATE_TRS;
        break;
      }
    case LEFT_SMALL:
      {
        // 方向盘左打一半
        setServoPulse(CHANNEL_STEER, SERVO_LEFT_TRL_US);
        // 后轮电机左小右大
        Target_speed_left = TARGET_SPEED_BASE;
        Target_speed_right = TARGET_SPEED_BASE * SPEED_RATE_TRL;
        break;
      }
    case NEUTRAL:
      {
        // 方向盘回中
        setServoPulse(CHANNEL_STEER, SERVO_MID_US);
        // 后轮电机相同
        Target_speed_left = TARGET_SPEED_BASE;
        Target_speed_right = TARGET_SPEED_BASE;
        break;
      }
    case RIGHT_SMALL:
      {
        // 方向盘右打一半
        setServoPulse(CHANNEL_STEER, SERVO_RIGHT_TRL_US);
        // 后轮电机右小左大
        Target_speed_left = TARGET_SPEED_BASE * SPEED_RATE_TRL;
        Target_speed_right = TARGET_SPEED_BASE;
        break;
      }
    case RIGHT_LARGE:
      {
        // 方向盘右打满
        setServoPulse(CHANNEL_STEER, SERVO_RIGHT_TRS_US);
        // 后轮电机右小左大
        Target_speed_left = TARGET_SPEED_BASE * SPEED_RATE_TRS;
        Target_speed_right = TARGET_SPEED_BASE;
        break;
      }
  }
  if (!forward) {
    Target_speed_left = -Target_speed_left;
    Target_speed_right = -Target_speed_right;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("转向在第8通道 编号从0开始的");

  pwm.begin();
  // 需要用示波器确认一下晶振的频率是多少Hz 才能保证PWM的频率够准确
  pwm.setOscillatorFrequency(XTAL_FREQ);  // 我们手上的i2c duoji晶振频率是25.8MHz
  pwm.setPWMFreq(SERVO_FREQ);             // 模块可接受频率40~1000 软件库可接受频率1~3500往高了不见得能实现

  pinMode(IN1, OUTPUT);   //TB6612控制引脚，控制电机1的方向，01为正转，10为反转
  pinMode(IN2, OUTPUT);   //TB6612控制引脚，
  pinMode(IN3, OUTPUT);   //TB6612控制引脚，控制电机2的方向，01为正转，10为反转
  pinMode(IN4, OUTPUT);   //TB6612控制引脚，
  pinMode(PWMA, OUTPUT);  //TB6612控制引脚，电机PWM
  pinMode(PWMB, OUTPUT);  //TB6612控制引脚，电机PWM
  digitalWrite(IN1, 0);   //TB6612控制引脚拉低
  digitalWrite(IN2, 0);   //TB6612控制引脚拉低
  digitalWrite(IN3, 0);   //TB6612控制引脚拉低
  digitalWrite(IN4, 0);   //TB6612控制引脚拉低
  analogWrite(PWMA, 0);   //TB6612控制引脚拉低
  analogWrite(PWMB, 0);   //TB6612控制引脚拉低

  pinMode(2, INPUT);  //编码器引脚
  pinMode(4, INPUT);  //编码器引脚
  pinMode(5, INPUT);  //编码器引脚
  pinMode(8, INPUT);  //编码器引脚
  pinMode(3, INPUT);  //按键引脚

  MsTimer2::set(5, control);                                          //使用Timer2设置5ms定时中断
  MsTimer2::start();                                                  //使用中断使能
  attachInterrupt(digitalPinToInterrupt(2), READ_ENCODER_R, CHANGE);  //开启外部中断 编码器接口1
  attachPCINT(digitalPinToPCINT(4), READ_ENCODER_L, CHANGE);          //开启外部中断 编码器接口2
}

void loop() {
  // steerCar(RIGHT_LARGE, 1);
  // delay(4000);
  // steerCar(LEFT_LARGE, 1);
  // delay(4000);
  // steerCar(RIGHT_SMALL, 1);
  // delay(4000);
  // steerCar(RIGHT_SMALL, 0);
  // delay(4000);
  // steerCar(LEFT_SMALL, 1);
  // delay(4000);
  // steerCar(LEFT_SMALL, 0);
  // delay(4000);
  // steerCar(RIGHT_LARGE, 0);
  // delay(4000);
  // steerCar(LEFT_LARGE, 0);
  // delay(4000);
  // steerCar(NEUTRAL, 1);
  // delay(4000);
  steerCar(NEUTRAL, 0);
  delay(4000);  
}

void READ_ENCODER_L() {
  if (digitalRead(ENCODER_L) == LOW) {                  //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW) Velocity_L++;  //根据另外一相电平判定方向
    else Velocity_L--;
  } else {                                              //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW) Velocity_L--;  //根据另外一相电平判定方向
    else Velocity_L++;
  }
}

void READ_ENCODER_R() {
  if (digitalRead(ENCODER_R) == LOW) {                  //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW) Velocity_R++;  //根据另外一相电平判定方向
    else Velocity_R--;
  } else {                                              //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW) Velocity_R--;  //根据另外一相电平判定方向
    else Velocity_R++;
  }
}
