/********************************************************************
 * 电机测速
 * 使用millis（）函数，记录时间，单位为毫秒
 * 使用中断函数，当引脚电平改变时触发，计数一次
 * 磁环为12cpr，将得到的值Val除以24，得到转动圈数
 * 除以当时的时间，因为时间单位为毫秒，除以60000转换为分
 * 这时候得到电机转速Speed ，单位 转/分
 * 注意；测出来的值会有些误差，电机转速在达到设置值后还会有些许的值增加
 * BY YFROBOT
 ********************************************************************/
#include "PID_v1.h"

#define pi 3.1415926f

//设置模块引脚接到数字引脚18（程序用到中断函数，UNO中断引脚为数字引脚2和3）
int DIR3A = 6;     //M1 Direction Control
int DIR3B = 7;     
int PWM3 = 11;
int encoder3A = 2;
int encoder3B = 8;
//float time;  //设置变量time，计时
float Speed3;  //设置变量Speed，存储转速
long duration3;//the number of the pulses of Moter3  //volatile
byte encoder3ALast; 
boolean Direction3;//the rotation Direction1 


double sampleTime = 10;

unsigned long beginTime3;
unsigned long endTime3;

//PID variables
double Kp3=21.8,Ki3=93.12, Kd3=0.986; //Kp3=12,Ki3=11.12, Kd3=0.306  //Adjust the PID to fit the motor
double Setpoint3,Input3,Output3;
double set3 = 6.0;  // fixed setPoint <=9cm/s
PID myPID3(&Input3,&Output3,&Setpoint3,Kp3,Ki3,Kd3,DIRECT);

//kalman filter added
// Kalman Filter application (Barun Basnet)
// Initialization of Kalman Variables

float RC = 6e-3, QC = 1e-4;  //Q = process noise covariance, R = measurement noise covariance
double Xpe0C = 0.0;  // Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1C = 0.0;  //Xe1 = estimation of X at time t=1 (previous state)
double Ppe0C = 0.0;  //Ppe0 = prior estimation of "error covariance" at t=0,  
double P1C = 1, P0C = 0; //P1 = error covariance at t=1, P0 = error covariance at t=0
double  KC = 0.0, Xe0C = 0.0, ZC = 0.0; //K = Kalman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0

float testPWM = 60; 

void setup(){
  Serial.begin(115200);
  EncoderInit3();//Initialize encoder
  digitalWrite(encoder3A,INPUT);
  digitalWrite(encoder3B,INPUT);
  digitalWrite(DIR3A,OUTPUT);
  digitalWrite(DIR3B,OUTPUT);
  digitalWrite(PWM3,OUTPUT);
  
  digitalWrite(PWM3,LOW);
  myPID3.SetMode(AUTOMATIC);
  myPID3.SetOutputLimits(-60,60);
  myPID3.SetSampleTime(sampleTime);
  myPID3.SetTunings(Kp3,Ki3,Kd3);
  beginTime3 = millis();

}

void loop(){
  endTime3 = millis();
  
  PIDMovement3 (set3);  //  <=9 cm/s
  //Movement3(56);

  //Movement3(testPWM);
 
  
  if(endTime3 - beginTime3 >=sampleTime)
  { 
    Speed3= float(duration3/53735*5.8*pi*1000/(endTime3 - beginTime3));           // /0.01;
    Serial.print(Speed3,5);Serial.print("  ");
    beginTime3 = endTime3;

    ZC = Speed3;
    Xpe0C = Xe1C; //Assumption or prediction 1
    Ppe0C = P1C + QC; //Assumption or prediction 2
    KC = Ppe0C/(Ppe0C + RC); // Measurement update or correction of "Kalman gain"
    Xe0C = Xpe0C + KC * (ZC - Xpe0C); // Measurement update or correction of "estimated signal"
    P0C = (1 - KC) * Ppe0C; // Measurement update or correction of "error covariance"
    Xe1C = Xe0C; //Update: current t=0 becomes t=1 in the next step
    P1C = P0C; //Update: current t=0 becomes t=1 in the next step
    Input3 = Xe0C;
    Serial.println(Xe0C,5);
    duration3 = 0;
  }
  
}

void Stop()
{
  analogWrite (PWM3,0);      //PWM Speed Control
  digitalWrite(DIR3A,LOW);  
  digitalWrite(DIR3B,LOW);
  analogWrite(PWM3,0);
}

void Movement3(double pwmSpeed)          
{
  if (pwmSpeed>0) 
  {
    analogWrite (PWM3,pwmSpeed);      //PWM Speed Control
    digitalWrite(DIR3A,HIGH);  
    digitalWrite(DIR3B,LOW);
  }  
  else if(pwmSpeed<0)
  {
    analogWrite (PWM3,-pwmSpeed);      //PWM Speed Control
    digitalWrite(DIR3A,LOW);  
    digitalWrite(DIR3B,HIGH);
  }
}
//PID modules
void PIDMovement3(double set3)
{
  Setpoint3=set3;
  //Input=4;
  myPID3.Compute();
  //Output = Speed;
  Movement3(Output3);
}

//Encoder modules 
void EncoderInit3() //Initialize encoder interruption
{
  Direction3 = true;//default -> Forward  
  pinMode(encoder3B,INPUT);  
  attachInterrupt(digitalPinToInterrupt(encoder3A) , wheelSpeed3, CHANGE);
} //Interrupt18

void wheelSpeed3()  //motor1 speed count
{
  int Lstate = digitalRead(encoder3A);
  if((encoder3ALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder3B);
    if(val == LOW && Direction3)
    {
      Direction3 = false;  //when Direction keeps false, duration++
    }
    else if(val == HIGH && !Direction3)
    {
      Direction3 = true;
    }
  }
  encoder3ALast = Lstate;
 
  if(!Direction3)  duration3++;
  else  duration3--;
}



