#include <iostream> 
#include<math.h>
using namespace std;

struct ActThreeVell
{
    float v1;
    float v2;
    float v3;
};

ActThreeVell ThreeWheelVellControl2(float Vx, float Vy, float angularVell)
{
#define AFA 60
#define L 9.9  // L is the distance from center of car to center of wheel, in cm
#define pi 3.1415926f
//#define ActThreeVell vell;
ActThreeVell vell;
float theta = 0;

vell.v1 = (float)(-cos((AFA + theta) / 180.0f*pi) * Vx - sin((theta + AFA) / 180.0f*pi) * Vy + L * angularVell);

vell.v2 = (float)(cos(theta / 180.0f*pi) * Vx + sin(theta /180.0f*pi) * Vy      + L * angularVell);

vell.v3 = (float)(-cos((AFA - theta) / 180.0f * pi) * Vx + sin((AFA - theta) / 180.0f*pi) * Vy + L * angularVell);

return vell;
}

int main()
{
	ActThreeVell test =  ThreeWheelVellControl2(0,-5,0);
	cout<< test.v1<<"  "<<test.v2<<"  "<<test.v3<<"  "<<endl;
	return 0;
	
}
