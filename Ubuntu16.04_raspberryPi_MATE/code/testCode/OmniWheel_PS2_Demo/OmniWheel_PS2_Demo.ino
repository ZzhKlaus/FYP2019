/*************************************************************************************
 * The routine is the PS2 handle control code, the joystick part is controlled; 
 * the omnidirectional wheel chassis is 3 wheels, 
 * and the angle between adjacent 2 wheels is 120 degrees.
 * Distribution as follows:
 *                  motor 1  //          \\motor2
 *                  
 *                  
 *                                 ==  motor3
 * The motor uses GM25 Hall encoder, 12cpr, speed ratio 74.8:1 wheel 58mm
 * Motherboard for Arduino MEGA 2560
 * Motor drive module L298N
 * Power supply 3s :18650 or Li-Po
 * by:YFRobot
 *************************************************************************************/
#include <PID_v1.h>
#include <PS2X_lib.h> //PS2 Library can be found here: 
PS2X ps2x; 

const byte encoder1A = 18;//A pin -> the interrupt pin 18
const byte encoder1B = 22;//B pin -> the digital pin 22
const byte encoder2A = 19;//A pin -> the interrupt pin 19
const byte encoder2B = 23;//B pin -> the digital pin 23
const byte encoder3A = 20;//A pin -> the interrupt pin 20
const byte encoder3B = 24;//B pin -> the digital pin 24
byte encoder1ALast;
byte encoder2ALast;
byte encoder3ALast;

int duration1;//the number of the pulses of Moter1
int duration2;//the number of the pulses of Moter2
int duration3;//the number of the pulses of Moter3
boolean Direction1;//the rotation Direction1 
boolean Direction2;//the rotation Direction1 
boolean Direction3;//the rotation Direction1 

int Speed1;
int Speed2;
int Speed3;
int Val;

#define PS2_DAT        A0     //PS2 Pin
#define PS2_CMD        A1
#define PS2_SEL        A2
#define PS2_CLK        A3

//Motor Driver variables
int DIR1A = 4;     //M1 Direction Control
int DIR1B = 5;     
int PWM1 = 6;

int DIR2A = 7;     //M2 Direction Control
int DIR2B = 8;     
int PWM2 = 9;

int DIR3A = 11;    //M3 Direction Control
int DIR3B = 12;
int PWM3 = 10;

//PID variables
double Kp1=1, Ki1=4, Kd1=0.01;  //Adjust the PID to fit the motor
double Kp2=1, Ki2=4, Kd2=0.01;  //Adjust the PID to fit the motor
double Kp3=1, Ki3=4, Kd3=0.01;  //Adjust the PID to fit the motor
double Setpoint1,Input1,Output1;
double Setpoint2,Input2,Output2;
double Setpoint3,Input3,Output3;
PID myPID1(&Input1,&Output1,&Setpoint1,Kp1,Ki1,Kd1,DIRECT);
PID myPID2(&Input2,&Output2,&Setpoint2,Kp2,Ki2,Kd2,DIRECT);
PID myPID3(&Input3,&Output3,&Setpoint3,Kp3,Ki3,Kd3,DIRECT);

int error = 0;
byte type = 0;
byte vibrate = 0;
void(* resetFunc) (void) = 0;

void setup() 
{
  Serial.begin(115200);//Initialize the serial port
  EncoderInit();//Initialize encoder
  pinMode(DIR1A,OUTPUT);
  pinMode(DIR1B,OUTPUT);
  pinMode(PWM1,OUTPUT);
  
  pinMode(DIR2A,OUTPUT);
  pinMode(DIR2B,OUTPUT);
  pinMode(PWM2,OUTPUT);
  
  pinMode(DIR3A,OUTPUT);
  pinMode(DIR3B,OUTPUT);
  pinMode(PWM3,OUTPUT);

  digitalWrite(PWM1,LOW);   
  digitalWrite(PWM2,LOW);  
  digitalWrite(PWM3,LOW);  
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID3.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-255,255);
  myPID2.SetOutputLimits(-255,255);
  myPID3.SetOutputLimits(-255,255);
  delay(300);
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);
}

void loop() {
  if (error == 1) //skip loop if no controller found
    resetFunc();
   //read controller and set large motor to spin at 'vibrate' speed
   ps2x.read_gamepad(false, vibrate); 

  if (ps2x.ButtonPressed(PSB_PAD_UP)) {          //Forward pressed
    Serial.println("PAD_UP just pressed");

  }else if(ps2x.ButtonReleased(PSB_PAD_UP)) {    //Forward released
    Serial.println(" PAD_UP just released");

  }

  if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {         //Down pressed
    Serial.println("PAD_DOWN just pressed");

  }else if(ps2x.ButtonReleased(PSB_PAD_DOWN)) {   //Down released
    Serial.println(" PAD_DOWN just released");

  }

  if (ps2x.ButtonPressed(PSB_PAD_LEFT)) {         //LEFT pressed
    Serial.println("PAD_LEFT just pressed");

  }else if(ps2x.ButtonReleased(PSB_PAD_LEFT)) {   //LEFT released
    Serial.println(" PAD_LEFT just released");

  }

  if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {         //RIGHT pressed
    Serial.println("PAD_RIGHT just pressed");

  }else if(ps2x.ButtonReleased(PSB_PAD_RIGHT)) {   //RIGHT released
    Serial.println(" PAD_RIGHT just released");

  }

  if(ps2x.ButtonPressed(PSB_TRIANGLE)){            //TRIANGLE pressed
    Serial.println("Triangle pressed");
  }else if(ps2x.ButtonReleased(PSB_TRIANGLE)){     //TRIANGLE released
    Serial.println(" Triangle released");
  }

  if(ps2x.ButtonPressed(PSB_CIRCLE)){               //CIRCLE pressed
    Serial.println("Circle just pressed");
  }else if(ps2x.ButtonReleased(PSB_CIRCLE)){        //CIRCLE released
    Serial.println(" Circle just released");
  }

  if(ps2x.ButtonPressed(PSB_SQUARE)){               //SQUARE pressed
    Serial.println("SQUARE just pressed");
  }else if(ps2x.ButtonReleased(PSB_SQUARE)){        //CIRCLE released
    Serial.println(" SQUARE just released");
  }
  
//will be TRUE if button was JUST pressed OR released
  if(ps2x.NewButtonState(PSB_CROSS)){               
      Serial.println("X just changed");
  }
/************************************************************************************
 * The X axis of the left and right joysticks, the Y axis is transformed into 
 * a complex number and a positive number, and 0 is a critical value. 
 * It is convenient to write a motor drive function below.
 * The PS2 joystick value is an analog value of 0-255, the median value is 127 or 128. 
 * Please use the routine to detect the median value of the PS2 joystick in your hand.
 ************************************************************************************/
  if (0 <= ps2x.Analog(PSS_LX) && ps2x.Analog(PSS_LX) <= 120) {
    Val = map((ps2x.Analog(PSS_LX) - 128), -127,0,100,0);
    PIDMovement (-Val/2,-Val/2,Val);
    } else if (255 >= ps2x.Analog(PSS_LX) && ps2x.Analog(PSS_LX) >= 135) {
      Val = map((ps2x.Analog(PSS_LX) - 128), 0,127,0,100);
      PIDMovement (Val/2,Val/2,-Val);
    }else if (0 <= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) <= 120) {
      Val = map((ps2x.Analog(PSS_LY) - 127), -128,0,100,0);
      PIDMovement (Val,-Val,0);
    } else if (255 >= ps2x.Analog(PSS_LY) && ps2x.Analog(PSS_LY) >= 135) {
      Val = map((ps2x.Analog(PSS_LY) - 127), 0,128,0,100);
      PIDMovement (-Val,Val,0);
    }else if (0 <= ps2x.Analog(PSS_RX) && ps2x.Analog(PSS_RX) <= 120) {
      Val = map((ps2x.Analog(PSS_RX) - 128), -127,0,100,0);
      PIDMovement (Val,Val,Val);
    } else if (255 >= ps2x.Analog(PSS_RX) && ps2x.Analog(PSS_RX) >= 135) {
      Val = map((ps2x.Analog(PSS_RX) - 128), 0,127,0,100);
      PIDMovement (-Val,-Val,-Val);
    }else if (0 <= ps2x.Analog(PSS_RY) && ps2x.Analog(PSS_RY) <= 120) {
      Val = map((ps2x.Analog(PSS_RY) - 127), -128,0,100,0);
      PIDMovement (0,0,0);
    } else if (255 >= ps2x.Analog(PSS_RY) && ps2x.Analog(PSS_RY) >= 135) {
      Val = map((ps2x.Analog(PSS_RY) - 127), 0,128,0,100);
      PIDMovement (0,0,0);
    }else{
      Stop();
    }
    //Serial.print("Val: ");
    //Serial.print(Val);
    //Serial.print("   PS2_LX: ");
    //Serial.print(ps2x.Analog(PSS_LX)-128);
    //Serial.print("   PS2_LY: ");
    //Serial.print(ps2x.Analog(PSS_LY)-127);
    //Serial.print("   PS2_RX: ");
    //Serial.print(ps2x.Analog(PSS_RX)-128);
    //Serial.print("   PS2_RY: ");
    //Serial.println(ps2x.Analog(PSS_RY)-127);

  Speed1=duration1*43/10;
  Speed2=duration2*43/10;
  Speed3=duration3*43/10;
  duration1 = 0;
  duration2 = 0;
  duration3 = 0;
  delay(10);
}

//Motor modules
void Stop()                //Stop
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

void Movement(int a,int b,int c)          
{
  if (a>0) 
  {
    analogWrite (PWM1,a);      //PWM1 Speed Control
    digitalWrite(DIR1A,HIGH);  
    digitalWrite(DIR1B,LOW);
  }  
  else if(a<0)
  {
    analogWrite (PWM1,-a);      //PWM1 Speed Control
    digitalWrite(DIR1A,LOW);  
    digitalWrite(DIR1B,HIGH);
  }
  if (b>0) 
  {
    analogWrite (PWM2,b);       //PWM2 Speed Control
    digitalWrite(DIR2A,HIGH);
    digitalWrite(DIR2B,LOW);
  }  
  else if(b<0)
  {
    analogWrite (PWM2,-b);     //PWM2 Speed Control
    digitalWrite(DIR2A,LOW);
    digitalWrite(DIR2B,HIGH);
  }
  if (c>0) 
  {
    analogWrite (PWM3,c);     //PWM3 Speed Control
    digitalWrite(DIR3A,HIGH);
    digitalWrite(DIR3B,LOW);  
  } else if(c<0)
  {
    analogWrite (PWM3,-c);     //PWM3 Speed Control
    digitalWrite(DIR3A,LOW);
    digitalWrite(DIR3B,HIGH);  
  }
}

//PID modules
void PIDMovement(int a,int b,int c)
{
  Setpoint1=a;
  Setpoint2=b;
  Setpoint3=c;
  Input1=Speed1;
  Input2=Speed2;
  Input3=Speed3;
  myPID1.Compute();
  myPID2.Compute();
  myPID3.Compute();
  Movement (Output1,Output2,Output3);
}
   
//Encoder modules 
void EncoderInit() //Initialize encoder interruption
{
  Direction1 = true; 
  Direction2 = true; 
  Direction3 = true;
  pinMode(encoder1B,INPUT);  
  pinMode(encoder2B,INPUT);  
  pinMode(encoder3B,INPUT);  
  attachInterrupt(5, wheelSpeed1, CHANGE);
  attachInterrupt(4, wheelSpeed2, CHANGE);
  attachInterrupt(3, wheelSpeed3, CHANGE);
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
