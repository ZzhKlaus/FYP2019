/*****************************
 * 3WD Omni Wheel Control Demo
 * 
 * moror1 //        \\motor2
 * 
 * 
 *            === motor3
 *            
 *电机3与Y轴重合，前进后退时考虑Y轴即可，
 *横向移动时，考虑X轴的速度即可            
 *                  BY YFRobot
 ****************************/

#define DIR1A 24    //Motor 1
#define DIR1B 25 
#define PWM1  6    //Motor 1 PWM

#define DIR2A 22    //Motor 2
#define DIR2B 23
#define PWM2  5    //Motor 2 PWM

#define DIR3A 26   //Motor 3
#define DIR3B 27
#define PWM3   7   //Motor 3 PWM

struct ActThreeVell
{
    float v1;
    float v2;
    float v3;
};
ActThreeVell ThreeWheelVellControl2(float Vx, float Vy, float angularVell)
{
#define AFA 60
#define L 2
#define pi 3.1415926f
//#define ActThreeVell vell;
ActThreeVell vell;
float theta = 0;

vell.v1 = (float)(-cos((AFA + theta) / 180.0f*pi) * Vx - sin((theta + AFA) / 180.0f*pi) * Vy + L * angularVell);

vell.v2 = (float)(cos(theta / 180.0f*pi) * Vx + sin(theta /180.0f*pi) * Vy      + L * angularVell);

vell.v3 = (float)(-cos((AFA - theta) / 180.0f * pi) * Vx + sin((AFA - theta) / 180.0f*pi) * Vy + L * angularVell);

return vell;
}

float Vx,Vy;
float V1,V2,V3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(DIR1A,OUTPUT);
  pinMode(DIR1B,OUTPUT);
  pinMode(DIR2A,OUTPUT);
  pinMode(DIR2B,OUTPUT);
  pinMode(DIR3A,OUTPUT);
  pinMode(DIR3B,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(PWM3,OUTPUT);
  
  pinMode(PWM1,0);
  pinMode(PWM2,0);
  pinMode(PWM3,0);
}

void Rotate(float speed)
{
  ActThreeVell vell = ThreeWheelVellControl2(0, 0, speed);
  V1 = vell.v1;
  V2 = vell.v2;
  V3 = vell.v3;
  if (V1 >= 0)
  {
    digitalWrite(DIR1A,HIGH);
    digitalWrite(DIR1B,LOW);
    analogWrite(PWM1, V1);
  }
  else
  {
    digitalWrite(DIR1A,LOW);
    digitalWrite(DIR1B,HIGH);
    analogWrite(PWM1, -V1);
  }

  if (V2 >= 0)
  {
    digitalWrite(DIR2A,HIGH);
    digitalWrite(DIR2B,LOW);
    analogWrite(PWM2, V2);
  }
  else
  {
    digitalWrite(DIR2A,LOW);
    digitalWrite(DIR2B,HIGH);
    analogWrite(PWM2, -V2);
  }

  if (V3 >= 0)
  {
    digitalWrite(DIR3A,HIGH);
    digitalWrite(DIR3B,LOW);
    analogWrite(PWM3, V3);
  }
  else
  {
    digitalWrite(DIR3A,LOW);
    digitalWrite(DIR3B,HIGH);
    analogWrite(PWM3, -V3);
  }
}
void Forward(float speed)      //前进，Y轴上下移动
{
  Vy = - speed;
  
  V1 = - sqrt(3)/2 * Vy;
  V2 = sqrt(3)/2 * Vy;
  
  digitalWrite(DIR1A,HIGH);
  digitalWrite(DIR1B,LOW);
  analogWrite(PWM1, V1);
  
  digitalWrite(DIR2A,LOW);
  digitalWrite(DIR2B,HIGH);
  analogWrite(PWM2,- V2); 
  
  analogWrite(PWM3,0); 
}

void Back(float speed)          //后退，Y轴上下移动
{
  Vy = - speed;
  V1 = - sqrt(3)/2 * Vy;
  V2 = sqrt(3)/2 * Vy;
  
  digitalWrite(DIR1A,LOW);
  digitalWrite(DIR1B,HIGH);
  analogWrite(PWM1, V1);
  
  digitalWrite(DIR2A,HIGH);
  digitalWrite(DIR2B,LOW);
  analogWrite(PWM2, - V2); 

  analogWrite(PWM3,0);
}

void Right(float speed)        //右平移，X轴移动
{
  Vx = speed;
  V1 = - 0.5 * Vx;
  V2 = - 0.5 * Vx;
  V3 = Vx;
  
  digitalWrite(DIR1A,HIGH);
  digitalWrite(DIR1B,LOW);
  analogWrite(PWM1, - V1);
  
  digitalWrite(DIR2A,HIGH);
  digitalWrite(DIR2B,LOW);
  analogWrite(PWM2, - V2); 
  
  digitalWrite(DIR3A,LOW);
  digitalWrite(DIR3B,HIGH);
  analogWrite(PWM3,V3);
}

void Left(float speed)         //左平移，X轴移动
{
  Vx = speed;
  V1 = - 0.5 * Vx;
  V2 = - 0.5 * Vx;
  V3 = Vx;
  
  digitalWrite(DIR1A,LOW);
  digitalWrite(DIR1B,HIGH);
  analogWrite(PWM1,- V1);
  
  digitalWrite(DIR2A,LOW);
  digitalWrite(DIR2B,HIGH);
  analogWrite(PWM2,- V2); 
  
  digitalWrite(DIR3A,HIGH);
  digitalWrite(DIR3B,LOW);
  analogWrite(PWM3,V3);
}

void Stop()            //停止
{
  analogWrite(PWM1,0);
  analogWrite(PWM2,0); 
  analogWrite(PWM3,0);
}

void loop() {
  Rotate(-50); delay(2000);
  Stop();     delay(1000);
  /******
  Forward(100);delay(2000);
  Stop();     delay(1000);
  Right(100);  delay(2000);
  Stop();     delay(1000);
  Back(100);   delay(2000);
  Stop();     delay(1000);
  Left(100);   delay(2000);
  Stop();     delay(1000);
  *********/
}
/*****************************************************************

  float Vx,Vy;     //定义正交分解之后的速度
  float Speed1,Speed2,Speed3;  //定义各个轮子的速度
  
  Vx = speed * cos(β);  //计算X方向速度  前进、后退时β= 270°，Vx = 0，Vy = -speed
  Vy = speed * sin(β);  //计算Y方向速度   左右移动时β= 0°，Vx = speed，Vy = 0

  L是底盘中心到轮子的距离，theta是偏航角度,前进，后退，左右平移时为0,ω为角速度，值为0

  V 和 L 单位用 厘米 cm

  轮子直接5.8cm
  

  V1 = -sin(30 + theta) * Vx - cos(30 + theta) * Vy + L * ω; 
  V2 = -sin(30 + theta) * Vx + cos(30 + theta) * Vy + L * ω;
  V3 = cos(theta) * Vx - sin(theta) * Vy + L * ω;

  //The end
*****************************************************************/
