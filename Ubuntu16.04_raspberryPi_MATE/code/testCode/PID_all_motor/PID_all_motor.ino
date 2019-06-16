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
#define AFA 60
#define L 11

struct ActThreeVell
  {
      float v1;
      float v2;
      float v3;
  };

int DIR1A = 26;     //M1 Direction Control
int DIR1B = 27;     
int PWM1 = 7;
int encoder1A = 20;
int encoder1B = 52;
float Speed1;  //设置变量Speed，存储转速
double duration1;//the number of the pulses of Moter3
byte encoder1ALast; 
boolean Direction1;//the rotation Direction1 

int DIR2A = 24;     //M1 Direction Control
int DIR2B = 25;     
int PWM2 = 6;
int encoder2A = 19;
int encoder2B = 50;
float Speed2;  //设置变量Speed，存储转速
double duration2;//the number of the pulses of Moter3
byte encoder2ALast; 
boolean Direction2;//the rotation Direction1 

int DIR3A = 22;     //M1 Direction Control
int DIR3B = 23;     
int PWM3 = 5;
int encoder3A = 18;
int encoder3B = 28;
//float time;  //设置变量time，计时
float Speed3;  //设置变量Speed，存储转速
double duration3;//the number of the pulses of Moter3
byte encoder3ALast; 
boolean Direction3;//the rotation Direction1 

double sampleTime1 = 10;
double sampleTime2 = 10;
double sampleTime3 = 10;

unsigned long beginTime1;
unsigned long endTime1;
unsigned long beginTime2;
unsigned long endTime2;
unsigned long beginTime3;
unsigned long endTime3;

ActThreeVell ThreeWheelVellControl2(float Vx, float Vy, float angularVell)
{
ActThreeVell vell;
float theta = 0;

vell.v1 = (float)(-cos((AFA + theta) / 180.0f*pi) * Vx - sin((theta + AFA) / 180.0f*pi) * Vy + L * angularVell);

vell.v2 = (float)(cos(theta / 180.0f*pi) * Vx + sin(theta /180.0f*pi) * Vy      + L * angularVell);

vell.v3 = (float)(-cos((AFA - theta) / 180.0f * pi) * Vx + sin((AFA - theta) / 180.0f*pi) * Vy + L * angularVell);

return vell;
}
ActThreeVell finalSpeed = ThreeWheelVellControl2(0,-5,0);
double set1 = 6;//finalSpeed.v1;
double set2 = 6;//finalSpeed.v2;
double set3 = 6;//finalSpeed.v3;


double Kp1=29.8,Ki1=93.12, Kd1=1.286; //,46 Ki= 10 15, Kd=0.306;  //Adjust the PID to fit the motor
double Setpoint1,Input1,Output1;
//double set1 = -8.0;  // fixed setPoint
PID myPID1(&Input1,&Output1,&Setpoint1,Kp1,Ki1,Kd1,DIRECT);

double Kp2=20.5,Ki2=81, Kd2=1.194; //,46 Ki= 10 15, Kd=0.306;  //Adjust the PID to fit the motor
double Setpoint2,Input2,Output2;
//double set2 = 8.0;  // fixed setPoint
PID myPID2(&Input2,&Output2,&Setpoint2,Kp2,Ki2,Kd2,DIRECT);

double Kp3=21.8,Ki3=93.12, Kd3=0.986; //,46 Ki= 10 15, Kd=0.306;  //Adjust the PID to fit the motor
double Setpoint3,Input3,Output3;
//double set3 = 0.0;  // fixed setPoint
PID myPID3(&Input3,&Output3,&Setpoint3,Kp3,Ki3,Kd3,DIRECT);

// KAlman Filter application (Barun Basnet)
// Initialization of KAlman Variables
float RA = 6e-3, QA = 1e-4;  //Q = process noise covariance, R = measurement noise covariance
double Xpe0A = 0.0;  // Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1A = 0.0;  //Xe1 = estimation of X at time t=1 (previous state)
double Ppe0A = 0.0;  //Ppe0 = prior estimation of "error covariance" at t=0,  
double P1A = 1, P0A = 0; //P1 = error covariance at t=1, P0 = error covariance at t=0
double  KA = 0.0, Xe0A = 0.0, ZA = 0.0; //K = KAlman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0

// KBlman Filter application (Barun Basnet)
// Initialization of KBlman Variables
float RB = 6e-3, QB = 1e-4;  //Q = process noise covariance, R = measurement noise covariance
double Xpe0B = 0.0;  // Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1B = 0.0;  //Xe1 = estimation of X at time t=1 (previous state)
double Ppe0B = 0.0;  //Ppe0 = prior estimation of "error covariance" at t=0,  
double P1B = 1, P0B = 0; //P1 = error covariance at t=1, P0 = error covariance at t=0
double  KB = 0.0, Xe0B = 0.0, ZB = 0.0; //K = KBlman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0
// Kalman Filter application (Barun Basnet)
// Initialization of Kalman Variables
float RC = 6e-3, QC = 1e-4;  //Q = process noise covariance, R = measurement noise covariance
double Xpe0C = 0.0;  // Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1C = 0.0;  //Xe1 = estimation of X at time t=1 (previous state)
double Ppe0C = 0.0;  //Ppe0 = prior estimation of "error covariance" at t=0,  
double P1C = 1, P0C = 0; //P1 = error covariance at t=1, P0 = error covariance at t=0
double  KC = 0.0, Xe0C = 0.0, ZC = 0.0; //K = Kalman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0


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
  myPID1.SetOutputLimits(-60,60);
  myPID1.SetSampleTime(sampleTime1);
  myPID1.SetTunings(Kp1,Ki1,Kd1);
  beginTime1 = millis();

  EncoderInit2();//Initialize encoder
  digitalWrite(encoder2A,INPUT);
  digitalWrite(encoder2B,INPUT);
  digitalWrite(DIR2A,OUTPUT);
  digitalWrite(DIR2B,OUTPUT);
  digitalWrite(PWM2,OUTPUT);
  digitalWrite(PWM2,LOW);
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-80,80);
  myPID2.SetSampleTime(sampleTime2);
  myPID2.SetTunings(Kp2,Ki2,Kd2);
  beginTime2 = millis();

  EncoderInit3();//Initialize encoder
  digitalWrite(encoder3A,INPUT);
  digitalWrite(encoder3B,INPUT);
  digitalWrite(DIR3A,OUTPUT);
  digitalWrite(DIR3B,OUTPUT);
  digitalWrite(PWM3,OUTPUT);
  digitalWrite(PWM3,LOW);
  myPID3.SetMode(AUTOMATIC);
  myPID3.SetOutputLimits(-80,80);
  myPID3.SetSampleTime(sampleTime3);
  myPID3.SetTunings(Kp3,Ki3,Kd3);
  beginTime3 = millis();
}

void loop(){
  
  PIDMovement1 (set1);
  endTime1 = millis();
  if(endTime1 - beginTime1 >=sampleTime1)
  { 
    Speed1=duration1/53735*5.8*pi*1000/(endTime1 - beginTime1);           // /0.01;
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

  //Serial.print(set1); Serial.print("\t");
  //Serial.print(set2); Serial.print("\t");
  //Serial.println(set3); 
  }
  Input1 = Xe0A;

  PIDMovement2(set2);
  endTime2 = millis();
  if(endTime2 - beginTime2 >=sampleTime2  && -3000<duration2<3000)
  { 
    Speed2=duration2/53735*5.8*pi*1000/(endTime2 - beginTime2);  
    beginTime2 = endTime2;
   
    ZB = Speed2;
    //Serial.print(Speed2, 5);Serial.print("\t");
    Xpe0B = Xe1B; //Assumption or prediction 1
    Ppe0B = P1B + QB; //Assumption or prediction 2
    KB = Ppe0B/(Ppe0B + RB); // Measurement update or correction of "KBlman gain"
    Xe0B = Xpe0B + KB * (ZB - Xpe0B); // Measurement update or correction of "estimated signal"
    P0B = (1 - KB) * Ppe0B; // Measurement update or correction of "error covariance"
    //Serial.println(Xe0B , 5);
    Xe1B = Xe0B; //Update: current t=0 becomes t=1 in the next step
    P1B = P0B; //Update: current t=0 becomes t=1 in the next step
    
    duration2 = 0;
  }
  Input2 = Xe0B;
  
   PIDMovement3 (set3);
   endTime3 = millis();
  if(endTime3 - beginTime3 >=sampleTime3)
  { 
    Speed3=duration3/53735*5.8*pi*1000/(endTime3 - beginTime3);   
    beginTime3 = endTime3;
    

    ZC = Speed3;
    //Serial.print(Speed3, 5);Serial.print("\t");
    Xpe0C = Xe1C; //Assumption or prediction 1
    Ppe0C = P1C + QC; //Assumption or prediction 2
    KC = Ppe0C/(Ppe0C + RC); // Measurement update or correction of "Kalman gain"
    Xe0C = Xpe0C + KC * (ZC - Xpe0C); // Measurement update or correction of "estimated signal"
    P0C = (1 - KC) * Ppe0C; // Measurement update or correction of "error covariance"
   // Serial.println(Xe0C , 5);
    
    Xe1C = Xe0C; //Update: current t=0 becomes t=1 in the next step
    P1C = P0C; //Update: current t=0 becomes t=1 in the next step
    duration3 = 0;
  }
  Input3 = Xe0C;
/*
  Serial.print(Speed1);
  Serial.print("  ");
  Serial.print(Speed2);
  Serial.print("  ");
  Serial.println(Speed3);
  */
  
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
  attachInterrupt(digitalPinToInterrupt(encoder1A), wheelSpeed1, CHANGE);
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
void PIDMovement2(double set2)
{
  Setpoint2=set2;
  //Input=4;
  myPID2.Compute();
  //Output = Speed;
  Movement2(Output2);
}

//Encoder modules 
void EncoderInit2() //Initialize encoder interruption
{
  Direction2 = true;//default -> Forward  
  pinMode(encoder2B,INPUT);  
  attachInterrupt(digitalPinToInterrupt(encoder2A), wheel2Speed, CHANGE);
}

void wheel2Speed()  //motor1 speed count
{
  int Lstate = digitalRead(encoder2A);
  if((encoder2ALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder2B);
    if(val == LOW && Direction2)
    {
      Direction2 = false;  //when Direction keeps false, duration++
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
  attachInterrupt(digitalPinToInterrupt(encoder3A), wheelSpeed3, CHANGE);
}

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


