#include "PID_v1.h"

#define pi 3.1415926f
#define encoder1A  2
#define encoder1B  8

volatile byte INTFLAG1 = 0;
volatile long duration1 = 0;
void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  attachInterrupt(0, wheelSpeed1, RISING);  
  // interrupt 0 digital pin 2 positive edge trigger
}

void loop() {
  // put your main code here, to run repeatedly:
  if (INTFLAG1)   {
       Serial.println(duration1);
       delay(100);
     INTFLAG1 = 0; // clear flag
  }
}

void wheelSpeed1()  //motor1 speed count
{
   INTFLAG1 = 1;
  if(digitalRead(encoder1A) && !digitalRead(encoder1B))
    {
      duration1++;
    }
  if(digitalRead(encoder1A) && digitalRead(encoder1B))
    {
      duration1--;
    }  
}
