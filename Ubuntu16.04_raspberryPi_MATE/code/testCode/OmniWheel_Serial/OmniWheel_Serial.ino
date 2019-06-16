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

const byte encoder1A = 20;//A pin -> the interrupt pin 18
const byte encoder1B = 30;//B pin -> the digital pin 22
const byte encoder2A = 19;//A pin -> the interrupt pin 19
const byte encoder2B = 29;//B pin -> the digital pin 23
const byte encoder3A = 18;//A pin -> the interrupt pin 20
const byte encoder3B = 28;//B pin -> the digital pin 24
byte encoder1ALast;  
byte encoder2ALast;
byte encoder3ALast;

double duration1;//the number of the pulses of Moter1
double duration2;//the number of the pulses of Moter2
double duration3;//the number of the pulses of Moter3
boolean Direction1;//the rotation Direction1 
boolean Direction2;//the rotation Direction1 
boolean Direction3;//the rotation Direction1 

float Speed1;
float Speed2;
float Speed3;

double sampleTime = 10;

//to calculate the speed of motor
unsigned long beginTime;
unsigned long endTime;

//Motor Driver variables
int DIR1A = 24;     //M1 Direction Control
int DIR1B = 25;     
int PWM1 = 6;

int DIR2A = 22;     //M2 Direction Control
int DIR2B = 23;     
int PWM2 = 5;

int DIR3A = 26;    //M3 Direction Control
int DIR3B = 27;
int PWM3 = 7;

//PID variables
double Kp1=42, Ki1=18, Kd1=0.12;  //Adjust the PID to fit the motor 1
double Kp2=33, Ki2=16, Kd2=0.15;  //Adjust the PID to fit the motor 2
double Kp3=12, Ki3=11.12, Kd3=0.306;  //Adjust the PID to fit the motor 3
double Setpoint1,Input1,Output1;
double Setpoint2,Input2,Output2;
double Setpoint3,Input3,Output3;
double SET1 = 0;  // fixed setPoint
double SET2 = 0;  // fixed setPoint
double SET3 = 5.0;  // fixed setPoint
//PID myPID1(&Input1,&Output1,&Setpoint1,Kp1,Ki1,Kd1,DIRECT);
//PID myPID2(&Input2,&Output2,&Setpoint2,Kp2,Ki2,Kd2,DIRECT);
PID myPID3(&Input3,&Output3,&Setpoint3,Kp3,Ki3,Kd3,DIRECT);


void setup()
{  
  Serial.begin(115200);//Initialize the serial port
  EncoderInit();//Initialize encoder
  digitalWrite(DIR1A,OUTPUT);
  digitalWrite(DIR1B,OUTPUT);
  digitalWrite(DIR2A,OUTPUT);
  digitalWrite(DIR2B,OUTPUT);
  digitalWrite(DIR3A,OUTPUT);
  digitalWrite(DIR3B,OUTPUT);
  
  digitalWrite(PWM1,OUTPUT);
  digitalWrite(PWM2,OUTPUT);
  digitalWrite(PWM3,OUTPUT);
  digitalWrite(PWM1,LOW);   
  digitalWrite(PWM2,LOW);  
  digitalWrite(PWM3,LOW);  
  
  //myPID1.SetMode(AUTOMATIC);
  //myPID2.SetMode(AUTOMATIC);
  myPID3.SetMode(AUTOMATIC);
  //myPID1.SetOutputLimits(-100,100);
  //myPID2.SetOutputLimits(-100,100);
  myPID3.SetOutputLimits(-100,100);
  //myPID1.SetTunings(Kp1,Ki1,Kd1);
  //myPID2.SetTunings(Kp2,Ki2,Kd2);
  myPID3.SetTunings(Kp3,Ki3,Kd3);
  //myPID1.SetSampleTime(sampleTime);
  //myPID2.SetSampleTime(sampleTime);
  myPID3.SetSampleTime(sampleTime);
  beginTime = millis();
}

void loop()
{
  PIDMovement (SET1, SET2, SET3);  //5 cm/s
  endTime = millis();
  if(endTime - beginTime >=sampleTime)
  { 
    //Speed1=duration1/53735*5.8*pi*1000/(endTime - beginTime);  
    //Speed2=duration2/53735*5.8*pi*1000/(endTime - beginTime);  
    Speed3=duration3/53735*5.8*pi*1000/(endTime - beginTime);  // /0.01;
    Serial.print(duration1);Serial.print("   ");
    Serial.print(duration2);Serial.print("   ");
    Serial.println(duration3);
    beginTime = endTime;
    //duration1 = 0;
    //duration2 = 0;
    duration3 = 0;
  }
  //Input1 = Speed1;
  //Input2 = Speed2;
  Input3 = Speed3;
}

void Movement1(int pwmSpeed)          
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
void Movement2(int pwmSpeed)          
{
  if (pwmSpeed>0) 
  {
    analogWrite (PWM2,pwmSpeed);      //PWM Speed Control
    digitalWrite(DIR2A,HIGH);  
    digitalWrite(DIR2B,LOW);
  }  
  else if(pwmSpeed<0)
  {
    analogWrite (PWM2,-pwmSpeed);      //PWM Speed Control
    digitalWrite(DIR2A,LOW);  
    digitalWrite(DIR2B,HIGH);
  }
}
void Movement3(int pwmSpeed)          
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
void PIDMovement(int set1,int set2,int set3)
{
  //Setpoint1=set1;
  //Setpoint2=set2;
  Setpoint3=set3;
  //myPID1.Compute();
  //myPID2.Compute();
  myPID3.Compute();
  //Movement1(Output1);
  //Movement2(Output2);
  Movement3(Output3);
}
   
//Encoder modules 
void EncoderInit() //Initialize encoder interruption
{
  //Direction1 = true;//default -> Forward  
  //Direction2 = true;//default -> Forward  
  Direction3 = true;//default -> Forward  
  //pinMode(encoder1B,INPUT);  
  //pinMode(encoder2B,INPUT);  
  pinMode(encoder3B,INPUT);  
  //attachInterrupt(Interrupt20, wheelSpeed1, CHANGE);
  //attachInterrupt(Interrupt19, wheelSpeed2, CHANGE);
  attachInterrupt(Interrupt18, wheelSpeed3, CHANGE);
}
 
void wheelSpeed1()  //motor1 speed count
{
  int Lstate = digitalRead(encoder1A);
  if((encoder1ALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder1B);
    if(val == LOW && Direction1)
    {
      Direction1 = false;
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

void wheelSpeed2()  //motor2 speed count
{
  int Lstate = digitalRead(encoder2A);
  if((encoder2ALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder2B);
    if(val == LOW && Direction2)
    {
      Direction2 = false;
    }
    else if(val == HIGH && !Direction2)
    {
      Direction2 = true;
    }
  }
  encoder2ALast = Lstate;
 
  if(!Direction2)  duration2++;
  else  duration2--;
}

void wheelSpeed3()  //motor3 speed count
{
  int Lstate = digitalRead(encoder3A);
  if((encoder3ALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder3B);
    if(val == LOW && Direction3)
    {
      Direction3 = false;
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

/**
//Motor modules
void Stop(void)                 //Stop
{
  digitalWrite(DIR1A,LOW);
  digitalWrite(DIR1B,LOW);
  digitalWrite(DIR2A,LOW);
  digitalWrite(DIR2B,LOW);
  digitalWrite(DIR3A,LOW);
  digitalWrite(DIR3B,LOW);
  analogWrite(PWM1,0);
  analogWrite(PWM2,0);
  analogWrite(PWM3,0);
}
**/
