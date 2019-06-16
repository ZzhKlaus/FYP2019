//File "odom.cpp"Created by Zhenghang Zhong
//email: 729170049@qq.com
//This is an C++ file used in ROS system 
//odometry 

#include <iostream> 
#include <ros/ros.h>		//ros library
#include <tf/transform_broadcaster.h>	
#include <std_msgs/Int16MultiArray.h>	//sandard ros variable type: int array
#include <std_msgs/Float64.h>		//sandard ros variable type: 64 bit long float 
#include <nav_msgs/Odometry.h>		//sandard ros variable type: odometry type 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include<math.h>
using namespace std;

struct ActThreeVell			//define a structure type to store speed in  three wheels
{
    float v1;
    float v2;
    float v3;
};

//matrix for inverse speed calculation
const float A_inv[3][3] ={-0.3333333333, 0.66666666667, -0.333333333333,
						-0.57735027, 0, 0.57735027,
					0.0336700337, 0.0336700337, 0.0336700337
};
//Rotation matrix
const float RotMatrixInv[3][3] = {
							1.0, 0.0, 0.0,
							0.0, 1.0, 0.0,
							0.0, 0.0, 1.0
};


ActThreeVell real_Speed;
ActThreeVell hope_Speed;

//ros publisher initialization
ros::Publisher pub_hope_speed_1;
ros::Publisher pub_hope_speed_2;
ros::Publisher pub_hope_speed_3;

ActThreeVell ThreeWheelVellControl2(float Vx, float Vy, float angularVell)
{
#define AFA 60	//alpha angle
#define L 9.9  // L is the distance from center of car to center of wheel, in cm
#define pi 3.1415926f
ActThreeVell vell;

float theta = 0;
vell.v1 = (float)(-cos((AFA + theta) / 180.0f*pi) * Vx - sin((theta + AFA) / 180.0f*pi) * Vy + L * angularVell);

vell.v2 = (float)(cos(theta / 180.0f*pi) * Vx + sin(theta /180.0f*pi) * Vy + L * angularVell);

vell.v3 = (float)(-cos((AFA - theta) / 180.0f * pi) * Vx + sin((AFA - theta) / 180.0f*pi) * Vy + L * angularVell);

return vell;
}

void real_speed_1_setter(const std_msgs::Float64& speed){
	real_Speed.v1 = speed.data;
}
void real_speed_2_setter(const std_msgs::Float64& speed){
	real_Speed.v2 = speed.data;
}
void real_speed_3_setter(const std_msgs::Float64& speed){
	real_Speed.v3 = speed.data;
}

//transform from speed in x-y to wheels' speed
void cmd_vel_setter(const geometry_msgs::Twist& cmd_vel){
	std_msgs::Float64 v1;
	std_msgs::Float64 v2;
	std_msgs::Float64 v3;
	ActThreeVell vell = ThreeWheelVellControl2(cmd_vel.linear.x*3, cmd_vel.linear.y*3, cmd_vel.angular.z/3);
	v1.data = vell.v1;
	v2.data = vell.v2;
	v3.data = vell.v3;
	pub_hope_speed_1.publish(v1);
	pub_hope_speed_2.publish(v2);
	pub_hope_speed_3.publish(v3);
	
	ROS_INFO("v1: [%f], v2: [%f], v3: [%f]", v1.data, v2.data, v3.data);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_publisher");
	ros::NodeHandle nh; 
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/base_controller/odom", 50);
	tf::TransformBroadcaster odom_broadcaster;
 	ros::Subscriber real_speed_1_sub = nh.subscribe("real_speed_1",50,real_speed_1_setter);
	ros::Subscriber real_speed_2_sub = nh.subscribe("real_speed_2",50,real_speed_2_setter);
	ros::Subscriber real_speed_3_sub = nh.subscribe("real_speed_3",50,real_speed_3_setter);
	
	pub_hope_speed_1 = nh.advertise<std_msgs::Float64>("hope_speed_1", 1);
	pub_hope_speed_2 = nh.advertise<std_msgs::Float64>("hope_speed_2", 1);
	pub_hope_speed_3 = nh.advertise<std_msgs::Float64>("hope_speed_3", 1);
	
	
	float A_times_Rot[3][3];	
	float R_Speed_wheel[3];
	float R_Speed_world[3];
	double x=0.0;
	double y=0.0;
	double th=0.0;
	double vx = 0;
	double vy = 0;
	double vth = 0;
	float temp =0;
	
	temp = 0.0;
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			for(int m=0; m<3; m++)
			{
				temp += RotMatrixInv[i][m]*A_inv[m][j];
			}
			A_times_Rot[i][j] = temp;
			temp =0;
		}
	}

	//real speed need calcu to return speed in x,y, angular
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
  	last_time = ros::Time::now();
	ros::Rate r(1.0);

	ros::Subscriber cmd_vel_sub = nh.subscribe("/turtle1/cmd_vel", 50, cmd_vel_setter);
	
	
while(nh.ok()){
	//the size of the message queue.  If messages are arriving faster 	than they are being processed, this is the number of messages that 	will be buffered up before beginning to throw away the oldest ones.	
	ros::spinOnce();               // check for incoming messages
	current_time = ros::Time::now();	
	
	//compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
	
	temp=0;
	//float R_Speed_wheel[3] = {real_Speed.v1, real_Speed.v2, real_Speed.v3};
	R_Speed_wheel[0] = real_Speed.v1;
	R_Speed_wheel[1] = real_Speed.v2;
	R_Speed_wheel[2] = real_Speed.v3;
	//To get real speed of AGV from wheels' speed
	for(int i=0; i<3; i++)
	{
		for(int m=0; m<3; m++)
		{
			temp += A_times_Rot[i][m]*R_Speed_wheel[m];
		}
		R_Speed_world[i] = temp;
		temp =0;
	}
	//speed in x,y and yaw speed
	vx = R_Speed_world[0];
	vy = R_Speed_world[1];
	vth = R_Speed_world[2];

    ROS_INFO("R_Speed_world_vx: [%f], vy: [%f], vth: [%f]", R_Speed_world[0], R_Speed_world[1], R_Speed_world[2]);

    double delta_x = vx * dt;
    double delta_y = vy * dt;
    double delta_th = vth * dt;
    //ROS_INFO("delta_x_y_th: [%f], vy: [%f], vth: [%f]", delta_x, delta_y, delta_th);
    x += delta_x;
    y += delta_y;
    th += delta_th;
	
    //since odometry is 6DOF, a quaternion is created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	
    //publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

	//send the transform
    odom_broadcaster.sendTransform(odom_trans);
	
	// publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
	
	//set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
	
	//set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

	//publish the message
    odom_pub.publish(odom);

 	last_time = ros::Time::now();
	r.sleep();
	}
}
