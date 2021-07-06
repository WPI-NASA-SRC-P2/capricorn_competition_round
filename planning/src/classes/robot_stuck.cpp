#include <iostream>
#include <geometry_msgs/Twist.h>


// to know if there is an obstacle on the path 
void VelocityCB(geometry_msgs::Twist &vel_msg)
{
    
}
int main(int argc, char ** argv)
{

ros::NodeHandle nh;

ros::Subscriber robot_sub = nh.subscribe("robot_name_/",1000,&VelocityCB);
ros::Publisher vel_pub = nh.publish.advertise<geometry_msgs::Twist>("robot_name_/cmd_vel", 1000);

//if( how to know if the robot is stuck for now lets keep it true)

while(true)
{
geometry_msgs::vel_msg;
vel_pub.publish(vel_msg);           
}
ros::spin(); 
}
