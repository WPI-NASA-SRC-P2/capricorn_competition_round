#include "path_planner_server.h"
#include <cspace.h>
#include <astar.h>
#include "planning/TrajectoryWithVelocities.h"
#include <geometry_msgs/Point.h>
#include "perception/ObjectArray.h"
#include "dyn_planning.h"
#include "std_msgs/Bool.h"


//Setting the node's update rate
#define UPDATE_HZ 10

#define DEBUG_INSTRUMENTATION

ros::Subscriber pathSubscriber;
ros::Subscriber ObjectDetectionSubscriber; ///
ros::Publisher replan_trigger;


std::string robot_name_ = "";
bool path_received;
// bool robot_reached = false;
nav_msgs::Path global_path_1;
std_msgs::Bool result;

nav_msgs::OccupancyGrid global_oGrid_;
// geometry_msgs::PoseStamped robot_pose;

perception::ObjectArray GlobalObstacle; 

void ObjectDetectionCB(perception::ObjectArray obstacles)
{
    GlobalObstacle = obstacles; 

}

void pathCB(nav_msgs::Path path)
{
  ROS_INFO(" Path Received");
  
  path_received = true;
}



int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "dynamic_planner");

  std::string robot_name(argv[1]);
  robot_name_ = robot_name;

  //ROS Topic names
  std::string ObjectDetection_topic_ = "/capricorn/"+robot_name_+"/object_detection_map";
  std::string path_topic_ = "/galaga/debug_path";
 

  //create a nodehandle
  ros::NodeHandle nh;

  //initializing a planner server

  ObjectDetectionSubscriber = nh.subscribe(ObjectDetection_topic_, 1000, ObjectDetectionCB);
  pathSubscriber = nh.subscribe(path_topic_, 1000, pathCB);

  #ifdef DEBUG_INSTRUMENTATION
  replan_trigger = nh.advertise<std_msgs::Bool>("/capricorn/"+ robot_name_ + "/trigger", 1000);
  #endif
perception::ObjectArray obstacles;
  while(ros::ok())
  {
    
    // else if (robot_reached)
    //   continue;
    // else
    // {

      // if(!path_received)
      //   continue;
      // else
      // {
    if(path_received == true)
    {
      ROS_INFO("Path Received");

    }
   
    
    ros::spinOnce();
  }
  return 0;
}
