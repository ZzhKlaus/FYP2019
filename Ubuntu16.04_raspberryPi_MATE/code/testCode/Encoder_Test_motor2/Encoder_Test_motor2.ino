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
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#define Interrupt18 5
#define Interrupt19 4
#define pi 3.1415926f
ros::NodeHandle nh;
geometry_msgs::Twist odom_msg;

//设置模块引脚接到数字引脚18（程序用到中断函数，UNO中断引脚为数字引脚2和3）
int DIR2A = 22;     //M1 Direction Control
int DIR2B = 23;     
int PWM2 = 5;
int encoder2A = 19;
int encoder2B = 29;
float Speed2;  //设置变量Speed，存储转速
double duration2;//the number of the pulses of Moter3
byte encoder2ALast; 
double set = 5; 
boolean Direction2;//the rotation Direction1 

double sampleTime = 10;

unsigned long beginTime2;
unsigned long endTime2;

//PID variables
double Kp2=20.5,Ki2=81, Kd2=1.194; //,46 Ki= 10 15, Kd=0.306;  //Adjust the PID to fit the motor
double Setpoint2,Input2,Output2;
double set2 = 5.0;  // fixed setPoint
PID myPID2(&Input2,&Output2,&Setpoint2,Kp2,Ki2,Kd2,DIRECT);

//KBlman filter added
// KBlman Filter application (Barun Basnet)
// Initialization of KBlman Variables
float RB = 6e-3, QB = 1e-4;  //Q = process noise covariance, R = measurement noise covariance
double Xpe0B = 0.0;  // Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1B = 0.0;  //Xe1 = estimation of X at time t=1 (previous state)
double Ppe0B = 0.0;  //Ppe0 = prior estimation of "error covariance" at t=0,  
double P1B = 1, P0B = 0; //P1 = error covariance at t=1, P0 = error covariance at t=0
double  KB = 0.0, Xe0B = 0.0, ZB = 0.0; //K = KBlman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0

void desiredSpeed( const std_msgs::Float64& cmd_msg){
 set = cmd_msg.data;
}
std_msgs::Float64 realSpeed;
ros::Subscriber<std_msgs::Float64> sub("hope_speed_2", &desiredSpeed );
ros::Publisher statePub("real_speed_2", &realSpeed);


void setup(){
  Serial.begin(115200);
  EncoderInit2();//Initialize encoder
  digitalWrite(encoder2A,INPUT);
  digitalWrite(encoder2B,INPUT);
  digitalWrite(DIR2A,OUTPUT);
  digitalWrite(DIR2B,OUTPUT);
  digitalWrite(PWM2,OUTPUT);
  
  digitalWrite(PWM2,LOW);
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-55,55);
  myPID2.SetSampleTime(sampleTime);
  myPID2.SetTunings(Kp2,Ki2,Kd2);
  beginTime2 = millis();
  
  //nh.initNode();
  nh.subscribe(sub);
  nh.advertise(statePub);
}

void loop(){
  
  //Movement(75);
  PIDMovement2(set2);  //4 cm/s
  
  endTime2 = millis();
  if(endTime2 - beginTime2 >=sampleTime)
  { 
    Speed2=duration2/53735*5.8*pi*1000/(endTime2 - beginTime2);           // /0.01;
    //Serial.println(Input2);
    //Serial.print("   ");
    //Serial.print(Output);

    beginTime2 = endTime2;

    ZB = Speed2;
    Serial.print(duration2, 5);Serial.print("\t");
    Xpe0B = Xe1B; //Assumption or prediction 1
    Ppe0B = P1B + QB; //Assumption or prediction 2
    KB = Ppe0B/(Ppe0B + RB); // Measurement update or correction of "KBlman gain"
    Xe0B = Xpe0B + KB * (ZB - Xpe0B); // Measurement update or correction of "estimated signal"
    P0B = (1 - KB) * Ppe0B; // Measurement update or correction of "error covariance"
    Serial.println(Xe0B , 5);
    Xe1B = Xe0B; //Update: current t=0 becomes t=1 in the next step
    P1B = P0B; //Update: current t=0 becomes t=1 in the next step
    
    //realSpeed.data = Xe0B;
    //statePub.publish( &realSpeed );
    
    duration2 = 0;
  }
  Input2 = Xe0B;
  
  //delay(100);
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
  attachInterrupt(Interrupt19, wheel2Speed, CHANGE);
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

