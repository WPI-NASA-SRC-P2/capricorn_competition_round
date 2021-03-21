#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <operations/navigation_algorithm.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 10

std::string robot_name;

void publishMessage(ros::Publisher &publisher, float data)
{
  std_msgs::Float64 pub_data;
  pub_data.data = data;
  ROS_INFO_STREAM(pub_data);

  publisher.publish(pub_data);
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
    ros::Publisher fl_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/steer/command/position", 10);
    ros::Publisher fr_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/steer/command/position", 10);
    ros::Publisher bl_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/steer/command/position", 10);
    ros::Publisher br_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/steer/command/position", 10);


    ros::Publisher fl_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/drive/command/velocity", 10);
    ros::Publisher fr_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/drive/command/velocity", 10);
    ros::Publisher bl_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/drive/command/velocity", 10);
    ros::Publisher br_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/drive/command/velocity", 10);

    geometry_msgs::Point point;
    point.x = std::atof(argv[2]);
    point.y = std::atof(argv[3]);

    std::vector<float> angles = NavigationAlgo::getSteeringAnglesRadialTurn(point);
    std::vector<float> velocities = NavigationAlgo::getDrivingVelocitiessRadialTurn(point, 2);
    ros::Duration(0.1).sleep();

    ros::spinOnce();

    publishMessage(fl_steer, angles.at(0));
    ros::Duration(0.5).sleep();
    publishMessage(fr_steer, angles.at(1));
    ros::Duration(0.5).sleep();
    publishMessage(br_steer, angles.at(2));
    ros::Duration(0.5).sleep();
    publishMessage(bl_steer, angles.at(3));
    ros::Duration(0.5).sleep();

    //Rate limit the loop to UPDATE_HZ.
    // update_rate.sleep();

    publishMessage(fl_drive, velocities.at(0));
    ros::Duration(0.5).sleep();
    publishMessage(fr_drive, velocities.at(1));
    ros::Duration(0.5).sleep();
    publishMessage(br_drive, velocities.at(2));
    ros::Duration(0.5).sleep();
    publishMessage(bl_drive, velocities.at(3));

    ros::Duration(10).sleep();

    publishMessage(fl_drive, 0);
    publishMessage(fr_drive, 0);
    publishMessage(br_drive, 0);
    publishMessage(bl_drive, 0);

    ros::Duration(1).sleep();

}
