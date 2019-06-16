//File "PID_ros_Motor2"Created by Zhenghang Zhong
//email: 729170049@qq.com
//This is an Arduino file used in ROS system 
//Cntrol of motor1, with encoder, PID and Kalman filter used

#include "PID_v1.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <Arduino.h>

#define Interrupt2 0

#define pi 3.1415926f

ros::NodeHandle nh;
geometry_msgs::Twist odom_msg;

int DIRA = 6;     //M1 Direction Control
int DIRB = 7;     
int PWM = 10;
int encoderA = 2;
int encoderB = 8;

float Val = 0;      //设置变量Val，计数
float Speed;  //设置变量Speed，存储转速
double duration;//the number of the pulses of Moter3
byte encoderALast; 
boolean Direction;//the rotation Direction1 

double sampleTime = 10;

unsigned long beginTime;
unsigned long endTime;

//PID variables
double Kp=20.5,Ki=81, Kd=1.194; //,46 Ki= 10 15, Kd=0.306;  //Adjust the PID to fit the motor
double Setpoint,Input,Output;
double set = 0;  // fixed setPoint
PID myPID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);

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
  //Serial.begin(115200);
  EncoderInit();//Initialize encoder
  digitalWrite(encoderA,INPUT);
  digitalWrite(encoderB,INPUT);
  digitalWrite(DIRA,OUTPUT);
  digitalWrite(DIRB,OUTPUT);
  digitalWrite(PWM,OUTPUT);
  
  digitalWrite(PWM,LOW);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-125,125);
  myPID.SetSampleTime(sampleTime);
  myPID.SetTunings(Kp,Ki,Kd);
  
  beginTime = millis();
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(statePub);
}

void loop(){
  //Movement(testSpeed);
  PIDMovement (set);
  endTime = millis();
  if(endTime - beginTime >=10)
  { 
    Speed= float( duration/53735*5.8*pi*1000/(endTime - beginTime));           // /0.01;
    //Serial.print(duration);
    //Serial.print("   ");
    //Serial.print(endTime - beginTime);
    //Serial.print("   ");
    //Serial.println(Speed,8);

    beginTime = endTime;
    
    ZB = Speed;
    //Serial.print(duration2, 5);Serial.print("\t");
    Xpe0B = Xe1B; //Assumption or prediction 1
    Ppe0B = P1B + QB; //Assumption or prediction 2
    KB = Ppe0B/(Ppe0B + RB); // Measurement update or correction of "KBlman gain"
    Xe0B = Xpe0B + KB * (ZB - Xpe0B); // Measurement update or correction of "estimated signal"
    P0B = (1 - KB) * Ppe0B; // Measurement update or correction of "error covariance"
    //Serial.println(Xe0B , 5);
    Xe1B = Xe0B; //Update: current t=0 becomes t=1 in the next step
    P1B = P0B; //Update: current t=0 becomes t=1 in the next step

    Input = Xe0B;
    
    realSpeed.data = Xe0B;
   
    if(abs(realSpeed.data) < 0.01)
    {
      realSpeed.data=0;
    }
    statePub.publish( &realSpeed );
    
    duration = 0;
    
    nh.spinOnce();
  }
  
}



void Stop()
{
  analogWrite (PWM,0);      //PWM Speed Control
  digitalWrite(DIRA,LOW);  
  digitalWrite(DIRB,LOW);
  analogWrite(PWM,0);
}

void Movement(int a)          
{
  if (a>0) 
  {
    analogWrite (PWM,a);      //PWM Speed Control
    digitalWrite(DIRA,HIGH);  
    digitalWrite(DIRB,LOW);
  }  
  else if(a<0)
  {
    analogWrite (PWM,-a);      //PWM Speed Control
    digitalWrite(DIRA,LOW);  
    digitalWrite(DIRB,HIGH);
  }
  else if (a==0)
  {
    digitalWrite(DIRA,LOW);  
    digitalWrite(DIRB,LOW);
    analogWrite(PWM,0);
  }
}

//PID modules
void PIDMovement(int a)
{
  Setpoint=a;
  Input=Speed;
  myPID.Compute();
  Movement (Output);
}

//Encoder modules 
void EncoderInit() //Initialize encoder interruption
{
  Direction = true;//default -> Forward  
  pinMode(encoderB,INPUT);  
  attachInterrupt(Interrupt2, wheelSpeed, CHANGE);
}

void wheelSpeed()  //motor1 speed count
{
  int Lstate = digitalRead(encoderA);
  if((encoderALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoderB);
    if(val == LOW && Direction)
    {
      Direction = false;  //when Direction keeps false, duration++
    }
    else if(val == HIGH && !Direction)
    {
      Direction = true;
    }
  }
  encoderALast = Lstate;
 
  if(!Direction)  duration++;
  else  duration--;
}

