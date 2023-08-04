#include <PinChangeInterrupt.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define XTAL_FREQ 25495000
#define SERVO_UP_MIN_US 930
#define SERVO_UP_MAX_US 1330
#define SERVO_UP_MID_US 1130
#define SERVO_DOWN_MIN_US 930   //右偏45
#define SERVO_DOWN_MAX_US 1930  //左偏45
#define SERVO_DOWN_MID_US 1430 
#define SERVO_FREQ 50    
#define CHANNEL_GIMBAL_UP 15 
#define CHANNEL_GIMBAL_DOWN 12 

String inputString = "";         // a String to hold incoming data
volatile bool stringComplete = false;  // whether the string is complete

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setServoPulse(uint8_t channel, double pulse_us) {
  double pulselength;
  pulselength = 1000000;
  pulselength /= SERVO_FREQ;
  pulselength /= 4096;
  pulse_us /= pulselength;
  pwm.setPWM(channel, 0, pulse_us);

}

void openmvloop(){
  if (stringComplete) {
    Serial.println(inputString);
    if(inputString[0] == '@'){
      if(inputString[0] == 's'){
         Flag_Stop = 1; 
      }
      if(inputString[1] == 'f'){
        Flag_Stop = 0;
        if(inputString[2] == 'L'){
          if(inputString[3] == 'B'){
            steerCar(LEFT_LARGE, 1);  
          }
          else{
            steerCar(LEFT_SMALL, 1);
          }  
        }
        if(inputString[2] == 'R'){
          if(inputString[3] == 'B'){
            steerCar(RIGHT_LARGE, 1);  
          }
          else{
            steerCar(RIGHT_SMALL, 1);
          }  
        }
        if(inputString[2] == 'N'){
          steerCar(NEUTRAL, 1);
        }   
      }
      if(inputString[1] == 'b'){
        Flag_Stop = 0;
        if(inputString[2] == 'L'){
          if(inputString[3] == 'B'){
            steerCar(LEFT_LARGE, 0);  
          }
          else{
            steerCar(LEFT_SMALL, 0);
          }  
        }
        if(inputString[2] == 'R'){
          if(inputString[3] == 'B'){
            steerCar(RIGHT_LARGE, 0);  
          }
          else{
            steerCar(RIGHT_SMALL, 0);
          }  
        }
        if(inputString[2] == 'N'){
          steerCar(NEUTRAL, 0);
        }   
      }   
    }
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  Serial.println("serialIn");
  while (Serial.available()) {
    // get the new byte:
    Serial.println("serialEvent");
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      Serial.println("stringComplete");
      stringComplete = true;
      // Serial.readString();
      break;
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("start");
  pwm.begin();
  pwm.setOscillatorFrequency(XTAL_FREQ);
  pwm.setPWMFreq(SERVO_FREQ);

}

void loop() {
  setServoPulse(CHANNEL_GIMBAL_UP, SERVO_UP_MID_US);
  setServoPulse(CHANNEL_GIMBAL_DOWN, SERVO_DOWN_MID_US);
  delay(1000);
}