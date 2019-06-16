//File "PID_ros_Motor1"Created by Zhenghang Zhong
//email: 729170049@qq.com
//This is an Arduino file used in ROS system 
//Cntrol of motor1, with encoder, PID and Kalman filter used

#include "PID_v1.h"
#include <ros.h>   //ros library
#include <geometry_msgs/Twist.h>  //message type, geometry
#include <std_msgs/Float64.h>     //ros standard float type
#include <Arduino.h>

#define Interrupt1 0   		//Interruption pin 0, in Arduino Uno it's pin 2

#define pi 3.1415926f

ros::NodeHandle nh;  		//Define ros node handler
geometry_msgs::Twist odom_msg;  //Initialize odometry message

int DIRA = 6;    		 //M1 Direction Control positive
int DIRB = 7;  			//M1 Direction Control positive   
int PWM = 9;			//PWM control pin
int encoderA = 2;		//encoder light sensor A pin 
int encoderB = 8;		//encoder light sensor B pin 

int stateB = 0;      		//store light sensor B's reading at present	
float Speed;  			//store Speed value
double duration;		//the number of the pulses of Moter1 in a short time
byte encoderALast; 
boolean Direction;		//the rotation Direction1, False means Positive 

double sampleTime = 10;		//The time interval of detecting speed as well as refresh PID and KF parameters

unsigned long beginTime;	//For time counting
unsigned long endTime;

//PID variables
double Kp=29.8,Ki=83.12, Kd=1.286; 	//Adjust the PID to fit the motor
double Setpoint,Input,Output;		//required speed
double set = 0;  			// fixed setPoint, will be changed by receiving data

PID myPID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);	//Initialize the PID controller

//KBlman filter added
// Initialization of KBlman Variables
float R = 6e-3, Q = 1e-4; 	 //Q = process noise covariance, R = measurement noise covariance
double Xpe0 = 0.0;  		// Xpe0 = prior estimation of signal X at time t=0 (current state)
double Xe1 = 0.0;  		//Xe1 = estimation of X at time t=1 (previous state)
double Ppe0 = 0.0;  		//Ppe0 = prior estimation of "error covariance" at t=0,  
double P1 = 1, P0 = 0; 		//P1 = error covariance at t=1, P0 = error covariance at t=0
double  K = 0.0, Xe0 = 0.0, Z = 0.0; //K = KBlman gain, Xe0 = estimation of signal at t=0, Z = measured signal at t=0

//To obtian the desired speed info from messages
void desiredSpeed( const std_msgs::Float64& cmd_msg){
 set = cmd_msg.data;
}

std_msgs::Float64 realSpeed;    //actual speed value
ros::Subscriber<std_msgs::Float64> sub("hope_speed_1", &desiredSpeed );  //Initialize a ros subscriber which would subscribe message of desired speed of motor1
ros::Publisher statePub("real_speed_1", &realSpeed); 			//Initialize a ros state publisher which would publish message of actual speed of motor1
	

void setup(){
  EncoderInit();		//Initialize encoder
  digitalWrite(encoderA,INPUT);	//Set the working mode of encoder A as input
  digitalWrite(encoderB,INPUT);
  digitalWrite(DIRA,OUTPUT); 	//Set the working mode of motor direction controller A as output
  digitalWrite(DIRB,OUTPUT);
  digitalWrite(PWM,OUTPUT);
  
  digitalWrite(PWM,LOW);
  myPID.SetMode(AUTOMATIC);  //Set the PID mode as automatic updating
  myPID.SetOutputLimits(-125,125);	//limiting the PWM outputs withim (-125,125), as large speed would make Arduino collapse, stated in report
  myPID.SetSampleTime(sampleTime);	//set PID's sampling time, or parameter refresh period
  myPID.SetTunings(Kp,Ki,Kd);		//Tuning of PID parameters
  
  beginTime = millis();			//return present time in millisecoond
  
  nh.initNode();			// Initialize ros node handler
  nh.subscribe(sub);			//regist subscriber
  nh.advertise(statePub);		//regist publisher
}

void loop(){
  //Movement(testSpeed); //test motor motion 
  
  PIDMovement (set);  //feed required speed value to PID
  endTime = millis();
  if(endTime - beginTime >=sampleTime)  
  { 
    Speed= float( duration/53735*5.8*pi*1000/(endTime - beginTime));           // every rotation circle around 53735 pulses generated, wheel diameter is 5.8cm, all units are in cm/s
    beginTime = endTime;
    
	//Kalman Filter parameters refreshment
    Z = Speed;
    //Serial.print(duration2, 5);Serial.print("\t"); //for testing, plot output
    Xpe0 = Xe1; //Assumption or prediction 1
    Ppe0 = P1 + Q; //Assumption or prediction 2
    K = Ppe0/(Ppe0 + R); // Measurement update or correction of "KBlman gain"
    Xe0 = Xpe0 + K * (Z - Xpe0); // Measurement update or correction of "estimated signal"
    P0 = (1 - K) * Ppe0; // Measurement update or correction of "error covariance"
    //Serial.println(Xe0 , 5);
    Xe1 = Xe0; //Update: current t=0 becomes t=1 in the next step
    P1 = P0; //Update: current t=0 becomes t=1 in the next step
    Input = Xe0; //represh Input value to PID from KF output
    
    realSpeed.data = Xe0;  // feed to publishser
    if(abs(realSpeed.data) < 0.01)  //prevent noise
    {
      realSpeed.data=0;
    }
    statePub.publish( &realSpeed ); //publishing
    
    duration = 0; 	//reset
    nh.spinOnce(); 	//node handler working once
  }
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
  else if (a==0)    //stop
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
    myPID.Compute();   //compute and execute PID
    Movement (Output);  //Move according to PID output which refresh itself automatically 
}

//Encoder modules, it is illustrated in report
void EncoderInit() //Initialize encoder interruption
{
  Direction = true;//default -> Forward  
  pinMode(encoderB,INPUT);  
  attachInterrupt(Interrupt1, wheelSpeed, CHANGE);
}

void wheelSpeed()  //motor1 speed count
{
  int Lstate = digitalRead(encoderA);
  if((encoderALast == LOW) && Lstate==HIGH)
  {
    int stateB = digitalRead(encoderB);
    if(stateB == LOW && Direction)
    {
      Direction = false;  //when Direction keeps false, duration++
    }
    else if(stateB == HIGH && !Direction)
    {
      Direction = true;
    }
  }
  encoderALast = Lstate;
 
  if(!Direction)  duration++;
  else  duration--;
}

