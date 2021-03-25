#include <operations/ExcavatorAction.h> 
#include <actionlib/server/simple_action_server.h>
#include <string.h>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <utils/common_names.h>

#include <geometry_msgs/Point.h> // To get target point in order to orient shoulder joint
//#include <tf/transform_datatypes.h> // To get shoulder joint location with respect to robot frame (http://wiki.ros.org/tf/Overview/Data%20Types)
#include <math.h> // used in findShoulderAngle() for atan2()

typedef actionlib::SimpleActionServer<operations::ExcavatorAction> Server;
ros::Publisher excavator_shoulder_yaw_publisher_;
ros::Publisher excavator_shoulder_pitch_publisher_;
ros::Publisher excavator_elbow_pitch_publisher_;
ros::Publisher excavator_wrist_pitch_publisher_;
using namespace COMMON_NAMES;

const int arraySize = 4; // Size of arrays used to change joint angles

enum Tasks{
  START_DIGGING = 1; // This starts the digging condition
  START_UNLOADING = 2; // This starts the unloading condition
  SLEEP_DURATION = 5; // The sleep duration
};

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
 * @brief Calculates the orientation of the shoulder joint based on the passed target point
 * 
 * @param target the x, y coordinates of the volatiles in the body frame of the excavator
 * @param shoulder the x, y coordinates of the shoulder joint in the body frame of excavator
 * @return float the yaw angle of the shoulder joint
 */
float findShoulderAngle(const geometry_msgs::Point &target, const geometry_msgs::Point &shoulder)
{
  return atan2((target.y - shoulder.y), (target.x - shoulder.x));
}

/**
 * @brief This function publishes the joint angles to the publishers
 * 
 * @param values array of joint angles (ordered)
 */
void publishAngles(float shoulder_yaw, float shoulder_pitch, float elbow_pitch, float wrist_pitch)
{
  excavator_shoulder_yaw_publisher_.publish(shoulder_yaw);
  excavator_shoulder_pitch_publisher_.publish(shoulder_pitch);
  excavator_elbow_pitch_publisher_.publish(elbow_pitch);
  excavator_wrist_pitch_publisher_.publish(wrist_pitch);
}

/**
 * @brief publishes the excavator angles to the rostopics small_excavator_1/arm/*joint_name/position
 * 
 * @param task the excavator task to be accomplished
 */
void publishExcavatorMessage(int task, const geometry_msgs::Point &target, const geometry_msgs::Point &shoulder)
{
  float theta = findShoulderAngle(target, shoulder);
  if(task == START_DIGGING) // digging angles
  {
    publishAngles(theta, 1, 2, 1); // This set of values moves the scoop under the surface
    ros::Duration(SLEEP_DURATION).sleep();
    publishAngles(0, -1.5, 1.5, -1); // This set of values moves the scoop above the surface and to the front center
  }
  else if(task == START_UNLOADING) // dumping angles
  {
    publishAngles(theta, -1.5, 1.5, 2); // This set of values moves in a way to deposit volatiles in hauler
    ros::Duration(SLEEP_DURATION).sleep();
    publishAngles(0, -1.5, 1.5, -1); // This set of values moves the scoop above the surface and to the front center
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
  geometry_msgs::Point shoulder;
  shoulder.x = 0.7;
  shoulder.y = 0.0;
  shoulder.z = 0.1;
  if (goal->task == START_DIGGING) // START_DIGGING = 1
  {
    publishExcavatorMessage(START_DIGGING, goal->target, shoulder);
    ros::Duration(SLEEP_DURATION).sleep();
    // action_server->working(); // might use for feedback
    action_server->setSucceeded();
  }
  else if (goal->task == START_UNLOADING) // START_UNLOADING = 2
  {
    publishExcavatorMessage(START_UNLOADING, goal->target, shoulder);
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