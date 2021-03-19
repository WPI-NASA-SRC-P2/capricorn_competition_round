#include <operations/HaulerAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <string.h>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <utils/common_names.h>

typedef actionlib::SimpleActionServer<operations::HaulerAction> Server;
ros::Publisher hauler_bin_publisher_;
using namespace COMMON_NAMES;

void publishHaulerBinMessage(float data)
{
  std_msgs::Float64 pub_data;
  pub_data.data = data;
  hauler_bin_publisher_.publish(pub_data);
}

void execute(const operations::HaulerGoalConstPtr& goal, Server* action_server)  // Note: "Action" is not appended to DoDishes here
{
  // Do lots of awesome groundbreaking robot stuff here
  if (goal->desired_pos > 0) //1 = bin empty //desired_pos.request.name earlier
  {
    publishHaulerBinMessage(3.0);
    //action_server->working();
    ros::Duration(5.0).sleep();
    publishHaulerBinMessage(0.0);
    action_server->setSucceeded();
  }
  //as->setSucceeded();
}

void initHaulerBinPublisher(ros::NodeHandle &nh, const std::string &robot_name)
{
  hauler_bin_publisher_ = nh.advertise<std_msgs::Float64>(robot_name + SET_BIN_POSITION, 1000);  
}

int main(int argc, char** argv)
{
  std::cout << argc << std::endl;
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
    std::cout << robot_name << std::endl;
    std::string node_name = robot_name + "_hauler_action_server";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    // Initialise the publishers for steering and wheel velocites
    initHaulerBinPublisher(nh, robot_name);
    // Action server 
    Server server(nh, "hauler_bin", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();

    return 0;
  }
}