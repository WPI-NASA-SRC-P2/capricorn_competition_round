#include <operations/ExcavatorAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <string.h>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <utils/common_names.h>

typedef actionlib::SimpleActionServer<operations::ExcavatorAction> Server;
ros::Publisher excavator_shoulder_yaw_publisher_;
ros::Publisher excavator_shoulder_pitch_publisher_;
ros::Publisher excavator_elbow_pitch_publisher_;
ros::Publisher excavator_wrist_pitch_publisher_;
using namespace COMMON_NAMES;

float BIN_RESET = 0.0; // This returns the bin to initial position 
float BIN_EMPTY = 3.0; // This is the angle to empty the bin
float SLEEP_DURATION = 5.0; // The sleep duration

/**
 * @brief Initializing the publisher here
 * 
 * @param nh nodeHandle
 * @param robot_name Passed in the terminal/launch file to target a particular rover
 */
void initExcavatorPublisher(ros::NodeHandle &nh, const std::string &robot_name)
{
  excavator_shoulder_yaw_publisher_ = nh.advertise<std_msgs::Float64>(robot_name + SET_SHOULDER_YAW_POSITION, 1000);
  excavator_shoulder_pitch_publisher_ = nh.advertise<std_msgs::Float64>(robot_name + SET_SHOULDER_PITCH_POSITION, 1000);
  excavator_elbow_pitch_publisher_ = nh.advertise<std_msgs::Float64>(robot_name + SET_ELBOW_PITCH_POSITION, 1000);
  excavator_wrist_pitch_publisher_ = nh.advertise<std_msgs::Float64>(robot_name + SET_WRIST_PITCH_POSITION, 1000); 
}

/**
 * @brief publishes the bin angle to the rostopic small_hauler_1/bin/command/position
 * 
 * @param data the bin angle
 */
void publishExcavatorMessage(int data)
{
  // std_msgs::Float64 joint_value;
  // joint_value.data = 2;
  // std_msgs::Float64 
  std_msgs::Float64 dig[4];
  std_msgs::Float64 undig[4];
  dig[0].data = 0;
  dig[1].data = 2;
  dig[2].data = 2;
  dig[3].data = 2;
  undig[0].data = 0;
  undig[1].data = -2;
  undig[2].data = -2;
  undig[3].data = -2;
  ROS_INFO_STREAM("The data value in publish method: " + std::to_string(data));
  // std_msgs::Float64MultiArray dig; //declare Atest 
  // dig.data.resize(5); //resize the array to assign to existent values 
  // dig.data[0] = 0; //put the desired value 
  // dig.data[1] = 2;
  // dig.data[2] = 2;
  // dig.data[3] = 2;
  // std_msgs::Float64MultiArray undig; //declare Atest 
  // undig.data.resize(5); //resize the array to assign to existent values 
  // undig.data[0] = 0; //put the desired value 
  // undig.data[1] = -2;
  // undig.data[2] = -2;
  // undig.data[3] = -2;
  if(data == 1) // digging angles
  {
    excavator_shoulder_yaw_publisher_.publish(dig[0]);
    ROS_INFO_STREAM("The if inside publish method joint value: " + std::to_string(dig[0].data));
    excavator_shoulder_pitch_publisher_.publish(dig[1]);
    excavator_elbow_pitch_publisher_.publish(dig[2]);
    excavator_wrist_pitch_publisher_.publish(dig[3]);
  }
  else if(data == 0) // undigging angles
  {
    excavator_shoulder_yaw_publisher_.publish(undig[0]);
    excavator_shoulder_pitch_publisher_.publish(undig[1]);
    excavator_elbow_pitch_publisher_.publish(undig[2]);
    excavator_wrist_pitch_publisher_.publish(undig[3]);
  }
  else
  {
    // call function here for random joint values
  }
  
}

/**
 * @brief This is where the action is executed if bin_angle is satisfied
 * 
 * @param goal The desired bin angle
 * @param action_server The server object for hauler action
 */
void execute(const operations::ExcavatorGoalConstPtr& goal, Server* action_server)
{
  ROS_INFO_STREAM("The first joint value in execute method: " + std::to_string(goal->wrist_pitch_angle));
  if (goal->wrist_pitch_angle == 1) // BIN_RESET = 0
  {
    ROS_INFO_STREAM("The first joint value in if of execute: ");
    publishExcavatorMessage(1);
    // action_server->working(); // might use for feedback
    ros::Duration(SLEEP_DURATION).sleep();
    // publishExcavatorMessage(0);
    action_server->setSucceeded();
  }
}

/**
 * @brief The main method for hauler server
 * 
 * @param argc The number of arguments passed
 * @param argv The array of arguments passed
 * @return int 
 */
int main(int argc, char** argv)
{
  ROS_INFO_STREAM(std::to_string(argc) + "\n");
  // Check if the node is being run through roslauch, and have one parameter of RobotName_Number
  if (argc != 2)
  {
      // Displaying an error message for correct usage of the script, and returning error.
      ROS_ERROR_STREAM("This Node must be launched via 'roslaunch' and needs an argument as <RobotName_Number>");
      return -1;
  }
  else
  {
    std::string robot_name = (std::string)argv[1];
    ROS_INFO_STREAM(robot_name + "\n");
    std::string node_name = robot_name + "_excavator_action_server";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    // Initialise the publishers for steering and wheel velocites
    initExcavatorPublisher(nh, robot_name);
    // Action server 
    Server server(nh, EXCAVATOR_ACTIONLIB, boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();

    return 0;
  }
}