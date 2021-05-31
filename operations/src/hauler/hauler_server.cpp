#include <operations/HaulerAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <string.h>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <utils/common_names.h>

typedef actionlib::SimpleActionServer<operations::HaulerAction> Server;
ros::Publisher hauler_bin_publisher_;
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
void initHaulerBinPublisher(ros::NodeHandle &nh, const std::string &robot_name)
{
  hauler_bin_publisher_ = nh.advertise<std_msgs::Float64>("/" + robot_name + SET_BIN_POSITION, 1000);  
}

/**
 * @brief publishes the bin angle to the rostopic small_hauler_1/bin/command/position
 * 
 * @param data the bin angle
 */
void publishHaulerBinMessage(float data)
{
  std_msgs::Float64 pub_data;
  pub_data.data = data;
  hauler_bin_publisher_.publish(pub_data);
}

/**
 * @brief This is where the action is executed if bin_angle is satisfied
 * 
 * @param goal The desired bin angle
 * @param action_server The server object for hauler action
 */
void execute(const operations::HaulerGoalConstPtr& goal, Server* action_server)
{
  if (goal->desired_state == true) // BIN_RESET = 0
  {
    publishHaulerBinMessage(BIN_EMPTY);
    // action_server->working(); // might use for feedback
    ros::Duration(SLEEP_DURATION).sleep();
    publishHaulerBinMessage(BIN_RESET);
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
  if(argc != 2 && argc != 4)
  {
      // Displaying an error message for correct usage of the script, and returning error.
      ROS_ERROR_STREAM("This Node needs an argument as <RobotName_Number>");
      return -1;
  }
  else
  {
    std::string robot_name = (std::string)argv[1];
    ROS_INFO_STREAM(robot_name + "\n");
    std::string node_name = robot_name + "_hauler_action_server";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    // Initialise the publishers for steering and wheel velocites
    initHaulerBinPublisher(nh, robot_name);
    // Action server 
    Server server(nh, HAULER_ACTIONLIB, boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();

    return 0;
  }
}