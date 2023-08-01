# FourWheelCar
1.unsigned char Turn_Off(float voltage) 电压检测（防止电压过低损坏锂电池，自动关断）

2.unsigned char My_click(void)  按键启动

3.float* velocity(int encoder_left, int encoder_right) 控制速度函数（通过编码器获取）

4.void Set_Pwm(int moto1, int moto2) 设置PWM用于控制电机

5.void Xianfu_Pwm(void) 电机PWM限幅（最大255，限幅至250）

6.void control() 主控制函数 

开启全局中断，用Timer2设置5ms定时中断，即每5ms进入一次中断，调用中断服务函数控制小车的方向

速度控制，周期40ms

7.void setServoPulse(uint8_t channel, double pulse_us) 舵机控制 （通道，脉宽）

外接16路I2C舵机控制板（PWM）通道范围0~15

①#define CHANNEL_STEER 8  //轮子舵机脉冲 设在8通道 

②#define CHANNEL_GIMBAL_up 7 //云台上层舵机脉冲 

③#define CHANNEL_GIMBAL_dowm 6 //云台下层舵机脉冲 

8.void steerCar(enum STEERING choice, bool forward) 转向控制

（方向选择：大偏x2，小偏x2，直行，前进标志位：1正向直行，0反向直行）

方向盘左打满（方向打得比较大，转弯半径小），后轮电机左小右大

方向盘左打一半（方向打得比较小，转弯半径大），后轮电机左小右大

方向盘回中，后轮电机速度相同，走直线

方向盘右打满（方向打得比较大，转弯半径小），后轮电机右小左大

方向盘右打一半（方向打得比较小，转弯半径大），后轮电机右小左大

9.void setup()  初始化函数 引脚

10.void loop()  循环执行控制方向函数

11.void READ_ENCODER_L() 左编码器读数

12.void READ_ENCODER_R() 右编码器读数


