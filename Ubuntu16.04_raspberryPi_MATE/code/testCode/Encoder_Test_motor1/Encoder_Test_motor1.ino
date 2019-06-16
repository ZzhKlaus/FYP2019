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

#define Interrupt18 5
#define Interrupt19 4
#define Interrupt20 3
#define pi 3.1415926f

//设置模块引脚接到数字引脚18（程序用到中断函数，UNO中断引脚为数字引脚2和3）
int DIR1A = 24;     //M1 Direction Control
int DIR1B = 25;     
int PWM1 = 6;
int encoder1A = 20;
int encoder1B = 30;
float Speed1;  //设置变量Speed，存储转速
double duration1;//the number of the pulses of Moter3
byte encoder1ALast; 
boolean Direction1;//the rotation Direction1 

double sampleTime = 10;

unsigned long beginTime1;
unsigned long endTime1;

//PID variables
double Kp1=29.8,Ki1=93.12, Kd1=1.286; //,46 Ki= 10 15, Kd=0.306;  //Adjust the PID to fit the motor
double Setpoint1,Input1,Output1;
double set1 = 5.0;  // fixed setPoint
PID myPID1(&Input1,&Output1,&Setpoint1,Kp1,Ki1,Kd1,DIRECT);

//KAlman filter added
// KAlman Filter application (Barun Basnet)
// Initialization of KAlman Variables
float RA = 6e-3, QA = 1e-4;  //Q = process noise covariance, R = measurement noise covariance
double Xpe0A = 0.0;  // Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1A = 0.0;  //Xe1 = estimation of X at time t=1 (previous state)
double Ppe0A = 0.0;  //Ppe0 = prior estimation of "error covariance" at t=0,  
double P1A = 1, P0A = 0; //P1 = error covariance at t=1, P0 = error covariance at t=0
double  KA = 0.0, Xe0A = 0.0, ZA = 0.0; //K = KAlman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0


void setup(){
  Serial.begin(115200);
  EncoderInit1();//Initialize encoder
  digitalWrite(encoder1A,INPUT);
  digitalWrite(encoder1B,INPUT);
  digitalWrite(DIR1A,OUTPUT);
  digitalWrite(DIR1B,OUTPUT);
  digitalWrite(PWM1,OUTPUT);
  
  digitalWrite(PWM1,LOW);
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-55,55);
  myPID1.SetSampleTime(sampleTime);
  myPID1.SetTunings(Kp1,Ki1,Kd1);
  beginTime1 = millis();

  //Input = 1;
}

void loop(){
  //Movement(75);
  PIDMovement1 (set1);  //4 cm/s
  
  endTime1 = millis();
  if(endTime1 - beginTime1 >=sampleTime)
  { 
    Speed1=duration1/53735*5.8*pi*1000/(endTime1 - beginTime1);           // /0.01;
    //Serial.println(Input1);
    //Serial.print("   ");
    //Serial.print(Output);
    //Serial.print("   ");
    //Serial.print(endTime - beginTime);
    //Serial.print("   ");
    //Serial.println(Speed,8);

    beginTime1 = endTime1;
    duration1 = 0;

    ZA = Speed1;
    Serial.print(Speed1, 5);Serial.print("\t");
    Xpe0A = Xe1A; //Assumption or prediction 1
    Ppe0A = P1A + QA; //Assumption or prediction 2
    KA = Ppe0A/(Ppe0A + RA); // Measurement update or correction of "KAlman gain"
    Xe0A = Xpe0A + KA * (ZA - Xpe0A); // Measurement update or correction of "estimated signal"
    P0A = (1 - KA) * Ppe0A; // Measurement update or correction of "error covariance"
    Serial.println(Xe0A , 5);
    Xe1A = Xe0A; //Update: current t=0 becomes t=1 in the next step
    P1A = P0A; //Update: current t=0 becomes t=1 in the next step
  
  }
  Input1 = Xe0A;
  
  //delay(100);
}
void Stop()
{
  analogWrite (PWM1,0);      //PWM Speed Control
  digitalWrite(DIR1A,LOW);  
  digitalWrite(DIR1B,LOW);
  analogWrite(PWM1,0);
}

void Movement1(double pwmSpeed)          
{
  if (pwmSpeed>0) 
  {
    analogWrite (PWM1,pwmSpeed);      //PWM Speed Control
    digitalWrite(DIR1A,HIGH);  
    digitalWrite(DIR1B,LOW);
  }  
  else if(pwmSpeed<0)
  {
    analogWrite (PWM1,-pwmSpeed);      //PWM Speed Control
    digitalWrite(DIR1A,LOW);  
    digitalWrite(DIR1B,HIGH);
  }
}
//PID modules
void PIDMovement1(double set1)
{
  Setpoint1=set1;
  //Input=4;
  myPID1.Compute();
  //Output = Speed;
  Movement1(Output1);
}

//Encoder modules 
void EncoderInit1() //Initialize encoder interruption
{
  Direction1 = true;//default -> Forward  
  pinMode(encoder1B,INPUT);  
  attachInterrupt(Interrupt20, wheelSpeed1, CHANGE);
}

void wheelSpeed1()  //motor1 speed count
{
  int Lstate = digitalRead(encoder1A);
  if((encoder1ALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder1B);
    if(val == LOW && Direction1)
    {
      Direction1 = false;  //when Direction keeps false, duration++
    }
    else if(val == HIGH && !Direction1)
    {
      Direction1 = true;
    }
  }
  encoder1ALast = Lstate;
 
  if(!Direction1)  duration1++;
  else  duration1--;
}

