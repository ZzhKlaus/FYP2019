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
int DIR1A = 22;     //M1 Direction Control
int DIR1B = 23;     
int PWM1 = 5;
int encoder1A = 19;
int encoder1B = 29;

int DIR2A = 24;     //M1 Direction Control
int DIR2B = 25;     
int PWM2 = 6;

int encoder2A = 20;
int encoder2B = 30;
//float time;  //设置变量time，计时
float Speed1,Speed2, Speed3;  //设置变量Speed，存储转速
double duration1, duration2, duration3;//the number of the pulses of Moter3
byte encoder1ALast, encoder2ALast, encoder3ALast; 
boolean Direction1, Direction2, Direction3;//the rotation Direction1 

double sampleTime = 10;

unsigned long beginTime1,beginTime2;
unsigned long endTime1,endTime2;

//PID variables
double Kp2=12,Ki2=11.12, Kd2=0.306; //,46 Ki= 10 15, Kd=0.306;  //Adjust the PID to fit the motor
double Kp1=33,Ki1=16, Kd1=0.15; //,46 Ki= 10 15, Kd=0.306;  //Adjust the PID to fit the motor
double Setpoint1,Input1,Output1;
double Setpoint2,Input2,Output2;
double set1 = 5.0;  // fixed setPoint
double set2 = 5.0;  // fixed setPoint
PID myPID1(&Input1,&Output1,&Setpoint1,Kp1,Ki1,Kd1,DIRECT);
PID myPID2(&Input2,&Output2,&Setpoint2,Kp2,Ki2,Kd2,DIRECT);

void setup(){
  Serial.begin(115200);
  EncoderInit();//Initialize encoder
  digitalWrite(encoder1A,INPUT);
  digitalWrite(encoder1B,INPUT);
  digitalWrite(encoder2A,INPUT);
  digitalWrite(encoder2B,INPUT);
  digitalWrite(DIR1A,OUTPUT);
  digitalWrite(DIR1B,OUTPUT);
  digitalWrite(DIR2A,OUTPUT);
  digitalWrite(DIR2B,OUTPUT);
  digitalWrite(PWM1,OUTPUT);
  digitalWrite(PWM2,OUTPUT);
  
  digitalWrite(PWM1,LOW);
  digitalWrite(PWM2,LOW);
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-100,100);
  myPID1.SetSampleTime(sampleTime);
  myPID1.SetTunings(Kp1,Ki1,Kd1);

  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-100,100);
  myPID2.SetSampleTime(sampleTime);
  myPID2.SetTunings(Kp2,Ki2,Kd2);
  
  beginTime1 = millis();
  beginTime2 = millis();
}

void loop(){
  /***
  Serial.print("Val:");
  Serial.print(val);
  Serial.print("    Speed:");
  Serial.print(Speed);

  Serial.print("Duration:");
  Serial.print(val);
  Serial.print("    1:");
  Serial.print(duration);
  ***/
  
  
  
  //Movement(75);
  PIDMovement1(set1);  //4 cm/s
  //PIDMovement2(set2);  //4 cm/s
  
  endTime1 = millis();
  endTime2 = millis();
  if(endTime1 - beginTime1 >=sampleTime ) //&& duration1!=0
  { 
    Speed1=duration1/53735*5.8*pi*1000/(endTime1 - beginTime1);           // /0.01;  
    beginTime1 = endTime1;
    Input1 = Speed1;
    Serial.print(duration1);
    Serial.print("   ");
    Serial.println(duration2);
    duration1 = 0;
  }
  /*
  if(endTime2 - beginTime2 >=sampleTime ) //&& duration1!=0
  { 
    Speed2=duration2/53735*5.8*pi*1000/(endTime2 - beginTime2); 
    beginTime2 = endTime2;
    Input2 = Speed2;
    duration2 = 0;
  }
  */
    
    //Serial.print("   ");
    //Serial.print(endTime - beginTime);
    //Serial.print("   ");
    //Serial.println(Speed1,8);
  //delay(100);
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
void Movement2(double pwmSpeed)          
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

//PID modules
void PIDMovement1(double set1)
{
  Setpoint1=set1;
  //Input=4;
  myPID1.Compute();
  //Output = Speed;
  Movement1(Output1);
}
//PID modules
void PIDMovement2(double set2)
{
  Setpoint2=set2;
  //Input=4;
  myPID2.Compute();
  //Output = Speed;
  Movement2(Output2);
}

//Encoder modules 
void EncoderInit() //Initialize encoder interruption
{
  Direction1 = true;//default -> Forward  
  Direction2 = true;//default -> Forward  
  pinMode(encoder1B,INPUT);  
  pinMode(encoder2B,INPUT);
  attachInterrupt(Interrupt18, wheelSpeed1, CHANGE);
  attachInterrupt(Interrupt19, wheelSpeed2, CHANGE);
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

void wheelSpeed2()  //motor1 speed count
{
  int Lstate = digitalRead(encoder2A);
  if((encoder2ALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder2B);
    if(val == LOW && Direction2)
    {
      Direction1 = false;  //when Direction keeps false, duration++
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


