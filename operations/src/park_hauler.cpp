
 
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <operations/navigation_algorithm.h>

#define UPDATE_HZ 10

std::string robot_name;
ros::Publisher fl_steer, fr_steer, bl_steer, br_steer, fl_drive, fr_drive, bl_drive, br_drive;

void publishMessage(float angle, float velocity)
{
    std_msgs::Float64 angle_msg, vel_msg;
    angle_msg.data = angle;
    vel_msg.data = velocity;
    fl_steer.publish(angle_msg);
    fr_steer.publish(angle_msg);
    bl_steer.publish(angle_msg);
    br_steer.publish(angle_msg);
    
    fl_drive.publish(vel_msg);
    fr_drive.publish(vel_msg);
    bl_drive.publish(vel_msg);
    br_drive.publish(vel_msg);    
}

int main(int argc, char *argv[])
{
    std::string name(argv[1]);
    robot_name = name;

    ros::init(argc, argv, "radial_turn_tester");

    ros::NodeHandle nh;
    
    fl_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/steer/command/position", 10);
    fr_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/steer/command/position", 10);
    bl_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/steer/command/position", 10);
    br_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/steer/command/position", 10);

    fl_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/drive/command/velocity", 10);
    fr_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/drive/command/velocity", 10);
    bl_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/drive/command/velocity", 10);
    br_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/drive/command/velocity", 10);

    ros::Rate update_rate(UPDATE_HZ);
    float angle = std::atof(argv[2]);
    float velocity = std::atof(argv[3]);

    while (ros::ok())
    {
        publishMessage(angle, velocity);
        ros::spinOnce();
        update_rate.sleep();
    }

    ROS_INFO("Exiting");
}