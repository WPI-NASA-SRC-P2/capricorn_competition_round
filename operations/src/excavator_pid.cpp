#include <operations/HaulerAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <utils/common_names.h>
#include <string>
#include <srcp2_msgs/PidTuningSrv.h>

using namespace COMMON_NAMES;

typedef actionlib::SimpleActionClient<operations::HaulerAction> Client;

ros::ServiceClient client;

/**
 * @brief This function is called to set the PID values of the excavator arm joints
 * 
 * @param joint_name name of the joint whose PID values need to be set
 * @param kp The proporttional gain
 * @param ki The integral gain
 * @param kd The derivative gain
 */
void setPid(std::string& joint_name, const float& kp, const float& ki, const float& kd)
{
  // String variable to store PID setting result
  std::string joint_pid_set;

  srcp2_msgs::PidTuningSrv excavator_pid;

  excavator_pid.request.joint = joint_name;
  excavator_pid.request.Kp = kp;
  excavator_pid.request.Ki = ki;
  excavator_pid.request.Kd = kd;

  if (client.call(excavator_pid))
  {
    joint_pid_set = excavator_pid.response.finished ? "True" : "False";
    ROS_INFO("The PID for %s set: %s", joint_name.c_str(), joint_pid_set.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service for %s", joint_name.c_str());
  }
}

/**
 * @brief Main PID function for the node
 * 
 * @param argc No arguments passed from command line
 * @param argv No arguments passed from command line
 * @return 0 for success and 1 for failed
 */

int main(int argc, char** argv)
{
  // Check if the node is being run through roslauch, and have one parameter of RobotName_Number
  if (argc != 2 && argc != 4)
  {
    // Displaying an error message for correct usage of the script, and returning error.
    ROS_ERROR_STREAM("This Node needs an argument as <RobotName_Number>");
    return -1;
  }
  else
  {
    std::string robot_name = (std::string)argv[1];
    ROS_INFO_STREAM(robot_name + "\n");
    std::string node_name = robot_name + "_excavator_pid_client";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

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

    client = nh.serviceClient<srcp2_msgs::PidTuningSrv>("/" + robot_name + "/pid_tuning"); // robot_name might require a preceding / to work correctly

    setPid(shoulder_yaw_joint, shoulder_yaw_kp, shoulder_yaw_ki, shoulder_yaw_kd);
    setPid(shoulder_pitch_joint, shoulder_pitch_kp, shoulder_pitch_ki, shoulder_pitch_kd);
    setPid(elbow_pitch_joint, elbow_pitch_kp, elbow_pitch_ki, elbow_pitch_kd);
    setPid(wrist_pitch_joint, wrist_pitch_kp, wrist_pitch_ki, wrist_pitch_kd);

    return 0;
  }
}