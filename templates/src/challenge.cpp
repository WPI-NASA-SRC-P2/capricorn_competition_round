#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <math.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 10

std::string robot_name;


int main(int argc, char *argv[])
{

    //Robot name, store it globally.
    robot_name = "small_scout_1";

    //Initialize the node, and create a private NodeHandle.
    ros::init(argc, argv, "challenge");
    ros::NodeHandle nh("~");

    //Create the rate limiter for the loop.
    ros::Rate update_rate(UPDATE_HZ);

    //Initialize publishers. This example create publishers to command each wheel on this robot.
    ros::Publisher fl_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/drive/command/velocity", 10);
    ros::Publisher fr_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/drive/command/velocity", 10);
    ros::Publisher bl_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/drive/command/velocity", 10);
    ros::Publisher br_speed = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/drive/command/velocity", 10);

    ros::Publisher fl_pos = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/steer/command/position", 10);
    ros::Publisher fr_pos = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/steer/command/position", 10);
    ros::Publisher bl_pos = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/steer/command/position", 10);
    ros::Publisher br_pos = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/steer/command/position", 10);

    float wheel_effort = 0;
    bool has_picked = false;
    std_msgs::Float64 speed_right;
    std_msgs::Float64 speed_left;

    float pose_right_data = 0;
    float pose_left_data = 0;
    std_msgs::Float64 pose_right;
    std_msgs::Float64 pose_left;

    while(ros::ok())
    {
        //Publish a constant wheel speed to each wheel.

        int choice = 0;

        if(!has_picked){
            do {
                std::cout << "Press:" <<std::endl << "(1) to drive " << robot_name << " FORWARDS" << std::endl << "(2) to drive " << robot_name << " BACKWARDS" << std::endl << "(3) to spin " << robot_name << " IN PLACE" << std::endl;
                std::cin >> choice;
            }while(choice < 1 || choice > 3);


            std::cout << "Input wheel effort: "<< std::endl;
            std::cin >> wheel_effort;

            std::cout << "Wheel effort set to " << wheel_effort << " by parameter for " + robot_name << std::endl;

            switch(choice){
                case 1:
                    speed_right.data = wheel_effort;
                    speed_left.data = wheel_effort;
                    pose_right_data = 0;
                    pose_left_data = 0;
                    pose_right.data = pose_right_data;
                    pose_left.data = pose_left_data;
                    break;
                case 2:
                    speed_right.data = -wheel_effort;
                    speed_left.data = -wheel_effort;
                    pose_right_data = 0;
                    pose_left_data = 0;
                    pose_right.data = pose_right_data;
                    pose_left.data = pose_left_data;
                    break;
                case 3:
                    speed_right.data = -wheel_effort;
                    speed_left.data = wheel_effort;
                    pose_right_data = -M_PI/4;
                    pose_left_data = M_PI/4;
                    pose_right.data = pose_right_data;
                    pose_left.data = pose_left_data;
                    break;
                default:
                    speed_right.data = speed_right.data;
                    speed_left.data = speed_left.data;
                    pose_right.data = pose_right.data;
                    pose_left.data = pose_left.data;
            }

            has_picked = true;

        }


        fl_speed.publish(speed_left);
        fr_speed.publish(speed_right);
        bl_speed.publish(speed_left);
        br_speed.publish(speed_right);
        fl_pos.publish(pose_right);
        fr_pos.publish(pose_left);
        bl_pos.publish(pose_left);
        br_pos.publish(pose_right);


        std::cout <<"Publishing..."<<std::endl;
        

        ros::spinOnce();

        //Rate limit the loop to UPDATE_HZ.
        update_rate.sleep();
    }
  
}
