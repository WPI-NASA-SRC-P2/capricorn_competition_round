#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <operations/navigation_algorithm.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 10

std::string robot_name;

// float angle_input =  0.785;
// float velocity_input = 0.5;


ros::Publisher fl_steer;
ros::Publisher fr_steer;
ros::Publisher bl_steer;
ros::Publisher br_steer;

ros::Publisher fl_drive;
ros::Publisher fr_drive;
ros::Publisher bl_drive;
ros::Publisher br_drive;


// void publishMessage(ros::Publisher &publisher, float data)
// {
//   std_msgs::Float64 pub_data;
//   pub_data.data = data;
//   ROS_INFO_STREAM(pub_data);

//   publisher.publish(pub_data);
// }

void publishMessage(float angle, float velocity)
{
  std_msgs::Float64 angle_msg;
  std_msgs::Float64 velocity_msg;
  angle_msg.data = angle;
  velocity_msg.data = velocity;

  fl_steer.publish(angle_msg);
  fr_steer.publish(angle_msg);
  bl_steer.publish(angle_msg);
  br_steer.publish(angle_msg);

  fl_drive.publish(velocity_msg);
  fr_drive.publish(velocity_msg);
  bl_drive.publish(velocity_msg);
  br_drive.publish(velocity_msg);
}

int main(int argc, char *argv[])
{
  // Check if the node is being run through roslauch, and have one parameter of RobotName_Number
    //Get robot name from parameters, and store it globally.
    std::string name(argv[1]);
    robot_name = name;

    //Initialize the node, and create a private NodeHandle.
    ros::init(argc, argv, "radial_turn_tester");
    ros::NodeHandle nh;

    //Create the rate limiter for the loop.
    ros::Rate update_rate(UPDATE_HZ);

    //Initialize subscribers. This example subscribes to this robot's imu topic.

    //Initialize publishers. This example create publishers to command each wheel on this robot.
    fl_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/steer/command/position", 10);
    fr_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/steer/command/position", 10);
    bl_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/steer/command/position", 10);
    br_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/steer/command/position", 10);


    fl_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/drive/command/velocity", 10);
    fr_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/drive/command/velocity", 10);
    bl_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/drive/command/velocity", 10);
    br_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/drive/command/velocity", 10);

    float angle = std::atof(argv[2]);
    float angle_rads = (angle * 3.1415) / 180;
    float velocity = std::atof(argv[3]);

    while (ros::ok())
    {
        publishMessage(angle_rads, velocity);
        ros::spinOnce();
        update_rate.sleep();
    }    

    ROS_INFO("done");
}
