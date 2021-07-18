#include "path_planner_server.h"
#include <cspace.h>
#include <astar.h>
#include "planning/TrajectoryWithVelocities.h"
#include <geometry_msgs/Point.h>
#include "perception/ObjectArray.h"
#include "perception/Object.h"
#include "dyn_planning_2.h"
#include "std_msgs/Bool.h"
#include <ros/ros.h>
#include<nav_msgs/Odometry.h>

//Setting the node's update rate
#define UPDATE_HZ 0.5

#define DEBUG_INSTRUMENTATION

ros::Subscriber pathSubscriber;
ros::Subscriber ObjectDetectionSubscriber; ///
ros::Subscriber robotPose_subscriber;

ros::Publisher replan_trigger;
tf2_ros::Buffer buffer_;
tf2_ros::TransformListener *listener_path_;



std::string robot_name_ = "";
geometry_msgs::PoseStamped robot_pose_;
bool path_received = false;
nav_msgs::Path global_path_1;
std_msgs::Bool result;
perception::ObjectArray global_Obstacles_; 
bool freePath = false;
std_msgs::Bool pathstatus;

using namespace COMMON_NAMES;


void DetectionCB(perception::ObjectArray obstacles)
{
  ROS_INFO(" obstacles Received");
  global_Obstacles_ = obstacles; 
}

void pathCB(nav_msgs::Path path)
{
  ROS_INFO(" Path Received");
  
  for(int i = 0; i < path.poses.size(); i++)
  {
    DynamicPlanning2::transformPose(path.poses.at(i), MAP, buffer_);
  }

  global_path_1 = path;
  path_received = true;
 
}

void poseCallback(nav_msgs::Odometry odom)
{
  robot_pose_.header = odom.header;
  robot_pose_.pose = odom.pose.pose;
  DynamicPlanning2::transformPose(robot_pose_, MAP, buffer_);
  
}



int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "dynamic_planner");

  std::string robot_name(argv[1]);
  robot_name_ = robot_name;

  //ROS Topic names
  std::string ObjectDetection_topic_ = CAPRICORN_TOPIC+robot_name_+OBJECT_DETECTION_OBJECTS_TOPIC;
  std::string path_topic_ = CAPRICORN_TOPIC+robot_name_+"/debug_path";
  std::string robotPose_topic_ = "/" + robot_name_+"/camera/odom";

 
  //create a nodehandle
  ros::NodeHandle nh;

  ObjectDetectionSubscriber = nh.subscribe(ObjectDetection_topic_, 1000, DetectionCB);
  robotPose_subscriber = nh.subscribe(robotPose_topic_, 1000, poseCallback);

  listener_path_ = new tf2_ros::TransformListener(buffer_);
  pathSubscriber = nh.subscribe(path_topic_, 1000, pathCB);

  // #ifdef DEBUG_INSTRUMENTATION
  // #endif
  replan_trigger = nh.advertise<std_msgs::Bool>(CAPRICORN_TOPIC + robot_name_ + "/"  + NAVIGATION_ACTIONLIB + REPLAN_TRAJECTORY, 1000);
  
  while(ros::ok())
  {
      if(path_received)
      {

        freePath = DynamicPlanning2::checkAllObstacles2(global_Obstacles_, global_path_1, robot_name_, robot_pose_, buffer_);
    
        if(freePath)
        {
          pathstatus.data = true;
          replan_trigger.publish(pathstatus);
        }
        else 
        {
          pathstatus.data = false;
          replan_trigger.publish(pathstatus);
        }
          
      }
     
    // run at 10hz
    ros::Rate update_rate(UPDATE_HZ); // it slowed down pretty much, wont recommend havung it.
    update_rate.sleep();
    ros::spinOnce();
  }

  delete listener_path_;
  return 0;
}
