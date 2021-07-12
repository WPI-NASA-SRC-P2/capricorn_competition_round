#include "path_planner_server.h"
#include <cspace.h>
#include <astar.h>
#include "planning/TrajectoryWithVelocities.h"
#include <geometry_msgs/Point.h>
#include "perception/ObjectArray.h"
#include "perception/Object.h"
#include "dyn_planning_2.h"
#include "std_msgs/Bool.h"


//Setting the node's update rate
#define UPDATE_HZ 10

#define DEBUG_INSTRUMENTATION

ros::Subscriber pathSubscriber;
ros::Subscriber ObjectDetectionSubscriber; ///
ros::Publisher replan_trigger;
tf2_ros::Buffer buffer_;
tf2_ros::TransformListener *listener_path_;



std::string robot_name_ = "";
bool path_received;
nav_msgs::Path global_path_1;
std_msgs::Bool result;
perception::ObjectArray global_Obstacles_; 
bool freePath;
std_msgs::Bool pathstatus;


void DetectionCB(perception::ObjectArray obstacles)
{
  ROS_INFO(" obstacles Received");
  global_Obstacles_ = obstacles; 
}

void pathCB(nav_msgs::Path path)
{
  ROS_INFO(" Path Received");
  global_path_1 = path;
  path_received = true;
  
  for(int i = 0; i < path.poses.size(); i++)
  {
    DynamicPlanning2::transformPose(path.poses[i],COMMON_NAMES::MAP, buffer_);
  }

 
}



int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "dynamic_planner");

  std::string robot_name(argv[1]);
  robot_name_ = robot_name;

  //ROS Topic names
  std::string ObjectDetection_topic_ = "/capricorn/"+robot_name_+"/object_detection/objects";
  std::string path_topic_ = "/galaga/debug_path";
 
  //create a nodehandle
  ros::NodeHandle nh;

  ROS_INFO("Begin Subscribing");

  ObjectDetectionSubscriber = nh.subscribe(ObjectDetection_topic_, 1000, DetectionCB);
  ROS_INFO("Begin Subscribing-2");
  listener_path_ = new tf2_ros::TransformListener(buffer_);
  pathSubscriber = nh.subscribe(path_topic_, 1000, pathCB);

  ROS_INFO("Done Subscribing");
  #ifdef DEBUG_INSTRUMENTATION
  replan_trigger = nh.advertise<std_msgs::Bool>("/capricorn/"+ robot_name_ + "/trigger", 1000);
  #endif
  ROS_INFO("Done Publishing");
  
  while(ros::ok())
  {
    ROS_INFO("While loop - 1");
      if(path_received)
      {

        freePath = DynamicPlanning2::checkAllObstacles(global_Obstacles_, global_path_1, robot_name_);
        ROS_INFO("While loop - 2");
        
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
        ROS_INFO("While loop - 3");
          
      }
    ros::spinOnce();
  }
  return 0;
}
