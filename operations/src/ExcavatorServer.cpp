#include <operations/ExcavatorAction.h> 
#include <actionlib/server/simple_action_server.h>
#include <string.h>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <utils/common_names.h>
#include <geometry_msgs/Point.h> // To get target point in order to orient shoulder joint
#include <math.h> // used in findShoulderAngle() for atan2()

typedef actionlib::SimpleActionServer<operations::ExcavatorAction> Server;
ros::Publisher excavator_shoulder_yaw_publisher_;
ros::Publisher excavator_shoulder_pitch_publisher_;
ros::Publisher excavator_elbow_pitch_publisher_;
ros::Publisher excavator_wrist_pitch_publisher_;
using namespace COMMON_NAMES;

// The global variables to save the last values passed to the joints
float curr_sh_yaw = 0;
float curr_sh_pitch = 0;
float curr_elb_pitch = 0;
float curr_wrt_pitch = 0;

// The task numbers and sleep duration
enum Tasks{
  START_DIGGING = 1, // This starts the digging condition
  START_UNLOADING = 2, // This starts the unloading condition
  SLEEP_DURATION = 2 // The sleep duration
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
 * @return atan2((target.y - shoulder.y), (target.x - shoulder.x)) is the required yaw angle of the shoulder joint
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

/**
 * @brief This function publishes the joint angles to the publishers
 * 
 * @param shoulder_yaw the desired shoulder yaw joint value
 * @param shoulder_pitch the desired shoulder pitch joint value
 * @param elbow_pitch the desired elbow pitch joint value
 * @param wrist_pitch the desired wrist pitch joint value
 * @param steps the number of simulation steps, higher value corresponds to slower movement
 * @param dig flag for dig or dump task to adjust the speed
 */
void publishAngles(float shoulder_yaw, float shoulder_pitch, float elbow_pitch, float wrist_pitch, int steps, bool dig)
{
  std_msgs::Float64 shoulder_yaw_msg;
  std_msgs::Float64 shoulder_pitch_msg;
  std_msgs::Float64 elbow_pitch_msg;
  std_msgs::Float64 wrist_pitch_msg;
  
  for(int i=0; i<steps; i++)
  {
    shoulder_yaw_msg.data = curr_sh_yaw + i*(shoulder_yaw-curr_sh_yaw)/steps;
    shoulder_pitch_msg.data = curr_sh_pitch + i*(shoulder_pitch-curr_sh_pitch)/steps;
    elbow_pitch_msg.data = curr_elb_pitch + i*(elbow_pitch-curr_elb_pitch)/steps;
    if(dig)
      wrist_pitch_msg.data = -(shoulder_pitch_msg.data + elbow_pitch_msg.data);
    else
      wrist_pitch_msg.data = curr_wrt_pitch + i*(wrist_pitch-curr_wrt_pitch)/steps;

    ros::Duration(0.2).sleep();

    excavator_shoulder_yaw_publisher_.publish(shoulder_yaw_msg);
    excavator_shoulder_pitch_publisher_.publish(shoulder_pitch_msg);
    excavator_elbow_pitch_publisher_.publish(elbow_pitch_msg);
    excavator_wrist_pitch_publisher_.publish(wrist_pitch_msg);
  }
  curr_sh_yaw = shoulder_yaw_msg.data;
  curr_sh_pitch = shoulder_pitch_msg.data;
  curr_elb_pitch = elbow_pitch_msg.data;
  curr_wrt_pitch = wrist_pitch_msg.data;
}

/**
 * @brief publishes the excavator angles to the rostopics small_excavator_1/arm/*joint_name/position
 * 
 * @param task the excavator task to be accomplished
 * @param target the target x, y coordinates in terms of the body frame
 * @param shoulder the fixed coordinates of the shoulder joint in body frame
 */
void publishExcavatorMessage(int task, const geometry_msgs::Point &target, const geometry_msgs::Point &shoulder)
{
  int FAST_STEPS = 10;
  int SLOW_STEPS = 30;
  float theta = findShoulderAngle(target, shoulder);
  if(task == START_DIGGING) // digging angles
  {
    publishAngles(theta, 0, 0, 0, FAST_STEPS, 1); // Additional step for safe trajectory to not bump into camera
    publishAngles(theta, 1, 2, -0.75, FAST_STEPS, 1); // This set of values moves the scoop under the surface
    ros::Duration(SLEEP_DURATION).sleep();
    publishAngles(theta, -1.5, 1.5, -0.2, FAST_STEPS, 1); // This set of values moves the scoop over the surface
  }
  else if(task == START_UNLOADING) // dumping angles
  {
    publishAngles(theta, -1.5, 1.5, -0.2, SLOW_STEPS, 1); // This set of values moves the scoop towards the hauler
    publishAngles(theta, -1.5, 1.5, 2, FAST_STEPS, 0); // This set of values moves the scoop to deposit volatiles in the hauler bin
    ros::Duration(SLEEP_DURATION).sleep();
    publishAngles(0, -1.5, 1.5, -1, FAST_STEPS, 1); // This set of values moves the scoop to the front center
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