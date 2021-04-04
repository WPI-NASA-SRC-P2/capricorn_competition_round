#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <operations/ResourceLocaliserAction.h>
#include <actionlib/server/simple_action_server.h>

#include <srcp2_msgs/VolSensorMsg.h>
#include "std_msgs/String.h"


// // typedef for the Action Server
typedef actionlib::SimpleActionServer<operations::ResourceLocaliserAction> ResourceLocaliserServer;


using namespace COMMON_NAMES;


bool near_volatile_ = false;
float volatile_distance_;

geometry_msgs::PoseStamped robot_pose;

void localiseResource(const operations::ResourceLocaliserGoalConstPtr& localiser_goal, ResourceLocaliserServer* server)
{
    ROS_INFO("Starting locating volatile sequence");
    // if (near_volatile_)
    // {
    //     ROS_INFO("");
    // }
    
    // float volatile_approx_orient_ = robot_pose.pose.pose.orientation;
    
    // Fake delay to simulate a locate action
    ros::Duration(5).sleep();

    server->setSucceeded();
}

void updateSensorData(const srcp2_msgs::VolSensorMsg::ConstPtr& msg)
{
    if(msg->distance_to != -1)
        near_volatile_ = false;
    else
    {
        volatile_distance_ = msg->distance_to;
    }
}

/**
 * @brief Subscribes to an odometry topic, and updates the global robot_pose
 * 
 * @param msg The odometry message to process
 */
void updateRobotPose(const nav_msgs::Odometry::ConstPtr &msg)
{
	robot_pose.header = msg->header;
	robot_pose.pose = msg->pose.pose;
}

int main(int argc, char** argv)
{
  // Ensure the robot name is passed in
  if (argc != 2 && argc != 4)
  {
      // Displaying an error message for correct usage of the script, and returning error.
      ROS_ERROR_STREAM("Not enough arguments! Please pass in robot name with number.");
      return -1;
  }
  else
  {
    // Robot Name from argument
    std::string robot_name(argv[1]);
    std::string node_name = robot_name + "_resource_localiser_action_server";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    
    ros::Subscriber subscriber = nh.subscribe(robot_name + VOLATILE_SENSOR_TOPIC, 1000, updateSensorData);
    ros::Subscriber update_current_robot_pose = nh.subscribe(CAPRICORN_TOPIC + robot_name + CHEAT_ODOM_TOPIC, 1000, updateRobotPose);

    ResourceLocaliserServer resource_localiser_server(nh, RESOURCE_LOCALISER_ACTIONLIB, boost::bind(&localiseResource, _1, &resource_localiser_server), false);
    resource_localiser_server.start();
    ros::spin();
    
    return 0;
  }
}
