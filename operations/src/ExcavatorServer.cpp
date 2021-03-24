#include <operations/ExcavatorAction.h> 
#include <actionlib/server/simple_action_server.h>
#include <string.h>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <utils/common_names.h>

typedef actionlib::SimpleActionServer<operations::ExcavatorAction> Server;
ros::Publisher excavator_shoulder_yaw_publisher_;
ros::Publisher excavator_shoulder_pitch_publisher_;
ros::Publisher excavator_elbow_pitch_publisher_;
ros::Publisher excavator_wrist_pitch_publisher_;
using namespace COMMON_NAMES;

float START_DIGGING = 1.0; // This starts the digging condition
float START_UNLOADING = 2.0; // This starts the unloading condition
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
 * @brief publishes the excavator angles to the rostopics small_excavator_1/arm/*joint_name/position
 * 
 * @param task the excavator task to be accomplished
 */
void publishExcavatorMessage(int task)
{
  std_msgs::Float64 dig[4]; // This set of values moves the scoop under the surface
  std_msgs::Float64 undig[4]; // This set of values moves the scoop above the surface
  std_msgs::Float64 deposit[4]; // This set of values moves in a way to deposit volatiles in hauler
  dig[0].data = 0; 
  dig[1].data = 1;
  dig[2].data = 2;
  dig[3].data = 1;
  undig[0].data = 0;
  undig[1].data = -1.5;
  undig[2].data = 1.5;
  undig[3].data = -1;
  deposit[0].data = 0;
  deposit[1].data = -1.5;
  deposit[2].data = 1.5;
  deposit[3].data = 2;
  if(task == START_DIGGING) // digging angles
  {
    excavator_shoulder_yaw_publisher_.publish(dig[0]);
    excavator_shoulder_pitch_publisher_.publish(dig[1]);
    excavator_elbow_pitch_publisher_.publish(dig[2]);
    excavator_wrist_pitch_publisher_.publish(dig[3]);
    ros::Duration(SLEEP_DURATION).sleep();
    excavator_shoulder_yaw_publisher_.publish(undig[0]);
    excavator_shoulder_pitch_publisher_.publish(undig[1]);
    excavator_elbow_pitch_publisher_.publish(undig[2]);
    excavator_wrist_pitch_publisher_.publish(undig[3]);
  }
  else if(task == START_UNLOADING) // dumping angles
  {
    excavator_shoulder_yaw_publisher_.publish(deposit[0]);
    excavator_shoulder_pitch_publisher_.publish(deposit[1]);
    excavator_elbow_pitch_publisher_.publish(deposit[2]);
    excavator_wrist_pitch_publisher_.publish(deposit[3]);
    ros::Duration(SLEEP_DURATION).sleep();
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
 * @brief This is where the action to dig or dump are executed
 * 
 * @param goal The desired excavator task
 * @param action_server The server object for excavator action
 */
void execute(const operations::ExcavatorGoalConstPtr& goal, Server* action_server)
{
  if (goal->wrist_pitch_angle == START_DIGGING) // START_DIGGING = 1
  {
    publishExcavatorMessage(START_DIGGING);
    ros::Duration(SLEEP_DURATION).sleep();
    // action_server->working(); // might use for feedback
    action_server->setSucceeded();
  }
  else if (goal->wrist_pitch_angle == START_UNLOADING) // START_UNLOADING = 2
  {
    publishExcavatorMessage(START_UNLOADING);
    ros::Duration(SLEEP_DURATION).sleep();
    action_server->setSucceeded();
  }
}

/**
 * @brief The main method for excavator server
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
    initExcavatorPublisher(nh, robot_name);
    Server server(nh, EXCAVATOR_ACTIONLIB, boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();

    return 0;
  }
}