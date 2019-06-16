//File "PID_ros_Motor3"Created by Zhenghang Zhong
//email: 729170049@qq.com
//This is an Arduino file used in ROS system 
//Cntrol of motor1, with encoder, PID and Kalman filter used

#include "PID_v1.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <Arduino.h>

#define Interrupt3 0

#define pi 3.1415926f

ros::NodeHandle nh;
geometry_msgs::Twist odom_msg;

int DIRA = 6;     //M1 Direction Control
int DIRB = 7;     
int PWM = 11;
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
double Kp=21.8,Ki=93.12, Kd=0.986; //,46 Ki= 10 15, Kd=0.306;  //Adjust the PID to fit the motor
double Setpoint,Input,Output;
double set = 5;  // fixed setPoint
PID myPID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);

//kalman filter added
// Kalman Filter application (Barun Basnet)
// Initialization of Kalman Variables
float RC = 6e-3, QC = 1e-4;  //Q = process noise covariance, R = measurement noise covariance
double Xpe0C = 0.0;  // Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1C = 0.0;  //Xe1 = estimation of X at time t=1 (previous state)
double Ppe0C = 0.0;  //Ppe0 = prior estimation of "error covariance" at t=0,  
double P1C = 1, P0C = 0; //P1 = error covariance at t=1, P0 = error covariance at t=0
double  KC = 0.0, Xe0C = 0.0, ZC = 0.0; //K = Kalman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0

void desiredSpeed( const std_msgs::Float64& cmd_msg){
 set = cmd_msg.data;
}
std_msgs::Float64 realSpeed;
ros::Subscriber<std_msgs::Float64> sub("hope_speed_3", &desiredSpeed );
ros::Publisher statePub("real_speed_3", &realSpeed);



void setup(){
  Serial.begin(115200);
  EncoderInit();//Initialize encoder
  digitalWrite(encoderA,INPUT);
  digitalWrite(encoderB,INPUT);
  digitalWrite(DIRA,OUTPUT);
  digitalWrite(DIRB,OUTPUT);
  digitalWrite(PWM,OUTPUT);
  
  digitalWrite(PWM,LOW);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-120,120);  //125 125
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
    beginTime = endTime;
    
    ZC = Speed;
    //Serial.print(Speed, 5);Serial.print("\t");
    Xpe0C = Xe1C; //Assumption or prediction 1
    Ppe0C = P1C + QC; //Assumption or prediction 2
    KC = Ppe0C/(Ppe0C + RC); // Measurement update or correction of "Kalman gain"
    Xe0C = Xpe0C + KC * (ZC - Xpe0C); // Measurement update or correction of "estimated signal"
    P0C = (1 - KC) * Ppe0C; // Measurement update or correction of "error covariance" 
    Xe1C = Xe0C; //Update: current t=0 becomes t=1 in the next step
    P1C = P0C; //Update: current t=0 becomes t=1 in the next step
    //Serial.println(Xe0C,5);
    
    Input = Xe0C;
    
    realSpeed.data = Xe0C;
    
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
  attachInterrupt(Interrupt3, wheelSpeed, CHANGE);
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

