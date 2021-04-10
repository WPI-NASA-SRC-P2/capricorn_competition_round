#include <operations/HaulerAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <utils/common_names.h>
#include <string>
#include <srcp2_msgs/PidTuningSrv.h>

using namespace COMMON_NAMES;

typedef actionlib::SimpleActionClient<operations::HaulerAction> Client;

/**
 * @brief Main PID function for the node
 * 
 * @param argc No arguments passed from command line
 * @param argv No arguments passed from command line
 * @return 0 for success and 1 for failed
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "excavator_pid_client");
  ros::NodeHandle nh;
  srcp2_msgs::PidTuningSrv excavator_pid;
  
  // Initializing joint names
  std::string shoulder_yaw_joint = "shoulder_yaw_joint";
  std::string shoulder_pitch_joint = "shoulder_pitch_joint";
  std::string elbow_pitch_joint = "elbow_pitch_joint";
  std::string wrist_pitch_joint = "wrist_pitch_joint";
  
  // Initializing joint values
  float shoulder_yaw_kp = 30;
  float shoulder_yaw_ki = 2;
  float shoulder_yaw_kd = 10;
  float shoulder_pitch_kp = 8;
  float shoulder_pitch_ki = 2;
  float shoulder_pitch_kd = 10;
  float elbow_pitch_kp = 25;
  float elbow_pitch_ki = 6;
  float elbow_pitch_kd = 3;
  float wrist_pitch_kp = 75;
  float wrist_pitch_ki = 5;
  float wrist_pitch_kd = 5;

  // String variable to store PID setting result
  std::string joint_pid_set;

  ros::ServiceClient client = nh.serviceClient<srcp2_msgs::PidTuningSrv>(EXCAVATOR_1 + "/pid_tuning");

  excavator_pid.request.joint = shoulder_yaw_joint;
  excavator_pid.request.Kp = shoulder_yaw_kp;
  excavator_pid.request.Ki = shoulder_yaw_ki;
  excavator_pid.request.Kd = shoulder_yaw_kd;

  if (client.call(excavator_pid))
  {
    joint_pid_set = excavator_pid.response.finished ? "True" : "False";
    ROS_INFO("The PID for shoulder yaw joint set: %s", joint_pid_set.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service for shoulder yaw joint");
    return 1;
  }

  excavator_pid.request.joint = shoulder_pitch_joint;
  excavator_pid.request.Kp = shoulder_pitch_kp;
  excavator_pid.request.Ki = shoulder_pitch_ki;
  excavator_pid.request.Kd = shoulder_pitch_kd;

  if (client.call(excavator_pid))
  {
    joint_pid_set = excavator_pid.response.finished ? "True" : "False";
    ROS_INFO("The PID for shoulder pitch joint set: %s", joint_pid_set.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service for shoulder pitch joint");
    return 1;
  }

  excavator_pid.request.joint = elbow_pitch_joint;
  excavator_pid.request.Kp = elbow_pitch_kp;
  excavator_pid.request.Ki = elbow_pitch_ki;
  excavator_pid.request.Kd = elbow_pitch_kd;

  if (client.call(excavator_pid))
  {
    joint_pid_set = excavator_pid.response.finished ? "True" : "False";
    ROS_INFO("The PID for elbow pitch joint set: %s", joint_pid_set.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service for elbow pitch joint");
    return 1;
  }

  excavator_pid.request.joint = wrist_pitch_joint;
  excavator_pid.request.Kp = wrist_pitch_kp;
  excavator_pid.request.Ki = wrist_pitch_ki;
  excavator_pid.request.Kd = wrist_pitch_kd;

  if (client.call(excavator_pid))
  {
    joint_pid_set = excavator_pid.response.finished ? "True" : "False";
    ROS_INFO("The PID for wrist pitch joint set: %s", joint_pid_set.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service for wrist pitch joint");
    return 1;
  }

  return 0;
}