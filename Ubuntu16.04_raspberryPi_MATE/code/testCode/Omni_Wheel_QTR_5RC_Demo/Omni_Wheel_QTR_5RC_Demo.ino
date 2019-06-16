//Please calibrate between the sensors, otherwise the following commands cannot be executed.
#include <QTRSensors.h>

#define NUM_SENSORS   5     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

#define DIR1A 4    //Motor 1
#define DIR1B 5 
#define PWM1  6    //Motor 1 PWM

#define DIR2A 7    //Motor 2
#define DIR2B 8
#define PWM2  9    //Motor 2 PWM

#define DIR3A 11   //Motor 3
#define DIR3B 12
#define PWM3  10   //Motor 3 PWM

int IR1;    //Get the sensor value
int IR2;
int IR3;
int IR4;
int IR5;

int val1;        //Define motor speed values
int val2;
int val3;

// sensors 1 through 5 are connected to digital pins A0 through A4, respectively
QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4},NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS]; 

void setup()
{
  Serial.begin(9600);
  pinMode(DIR1A,OUTPUT);
  pinMode(DIR1B,OUTPUT);
  pinMode(DIR2A,OUTPUT);
  pinMode(DIR2B,OUTPUT);
  pinMode(DIR3A,OUTPUT);
  pinMode(DIR3B,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(PWM3,OUTPUT);

  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
}

void loop()
{
   IR1 = sensorValues[0];    //Get the sensor value
   IR2 = sensorValues[1];
   IR3 = sensorValues[2];
   IR4 = sensorValues[3];
   IR5 = sensorValues[4];
  unsigned int position = qtrrc.readLine(sensorValues);
/*
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.print(); // uncomment this line if you are using raw values
  //Serial.print(position); // comment this line out if you are using raw values
*/

//Determine the sensor value, less than 2000 to the right, 
//greater than 2000 to the left, 2000 straight.
//Determine the value of the sensor to execute the motor drive function
  if(position >= 0 && position < 1000)
  {
    Right_more();
  }
  else if(position >= 1000 && position <= 1800)
  {
    Right();
  }
  else if(position > 1800 && position < 2200)
  {
    Forward();
  }
  else if(position >= 2200 && position <= 3000)
  {
    Left();
  }
  else if(position > 3000 && position <= 4000)
  {
    Left_more();
  }
  //Serial.print("  ");   //To print the value, remove the comment
  //Serial.print(IR1);
  //Serial.print("  ");
  //Serial.print(IR2);
  //Serial.print("  ");
  //Serial.print(IR3);
  //Serial.print("  ");
  //Serial.print(IR4);
  //Serial.print("  ");
  //Serial.print(IR5);
  //Serial.print("  val1:");
  //Serial.print(val1);
  //Serial.print("  val2:");
  //Serial.print(val2);
  //Serial.print("  val3:");
  //Serial.println(val3);
  delay(20);
}

//Motor drive function
//The value can be modified according to actual usage
void Forward()      
{
  digitalWrite(DIR1A,HIGH);
  digitalWrite(DIR1B,LOW);
  val1 = map(IR3,500,1000,100,100);
  analogWrite(PWM1, val1);
  
  digitalWrite(DIR2A,LOW);
  digitalWrite(DIR2B,HIGH);
  val2 = map(IR3,500,1000,100,100);
  analogWrite(PWM2, val2); 

  digitalWrite(DIR3A,LOW);
  digitalWrite(DIR3B,LOW);
  val3 = 0;
  analogWrite(PWM3,val3);
}

void Left()
{
  digitalWrite(DIR1A,HIGH);
  digitalWrite(DIR1B,LOW);
  val1 = map(IR4,0,1000,100,70);
  analogWrite(PWM1, val1);
  
  digitalWrite(DIR2A,LOW);
  digitalWrite(DIR2B,HIGH);
  val2 = 100;
  analogWrite(PWM2, val2); 

  digitalWrite(DIR3A,LOW);
  digitalWrite(DIR3B,HIGH);
  val3 = map(IR4,0,1000,0,70);
  analogWrite(PWM3, val3);
}

void Left_more()
{
  digitalWrite(DIR1A,HIGH);
  digitalWrite(DIR1B,LOW);
  val1 = map(IR5,0,1000,70,0);
  analogWrite(PWM1, val1);
  
  digitalWrite(DIR2A,LOW);
  digitalWrite(DIR2B,HIGH);
  val2 = 100;
  analogWrite(PWM2, val2); 

  digitalWrite(DIR3A,LOW);
  digitalWrite(DIR3B,LOW);
  val3 = map(IR5,0,1000,0,100);
  analogWrite(PWM3, val3);
}

void Right()
{
  digitalWrite(DIR1A,HIGH);
  digitalWrite(DIR1B,LOW);
  val1 = map(IR2,0,1000,100,100);
  analogWrite(PWM1, val1);
  
  digitalWrite(DIR2A,LOW);
  digitalWrite(DIR2B,HIGH);
  val2 = map(IR2,0,1000,100,70);
  analogWrite(PWM2, val2); 

  digitalWrite(DIR3A,HIGH);
  digitalWrite(DIR3B,LOW);
  val3 = map(IR2,0,1000,0,70);
  analogWrite(PWM3, val3); 
}

void Right_more()
{
  digitalWrite(DIR1A,HIGH);
  digitalWrite(DIR1B,LOW);
  val1 = 100;
  analogWrite(PWM1, val1);
  
  digitalWrite(DIR2A,LOW);
  digitalWrite(DIR2B,HIGH);
  val2 = map(IR1,0,1000,70,0);
  analogWrite(PWM2, val2);

  digitalWrite(DIR3A,LOW);
  digitalWrite(DIR3B,LOW);
  val3 = map(IR1,0,1000,0,100);
  analogWrite(PWM3,val3);  
}

