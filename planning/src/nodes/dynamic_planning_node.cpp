#include "path_planner_server.h"

#include <cspace.h>
#include <astar.h>
#include "planning/TrajectoryWithVelocities.h"
#include <geometry_msgs/Point.h>
#include "dyn_planning.h"
#include "std_msgs/Bool.h"

//Setting the node's update rate
#define UPDATE_HZ 10

#define DEBUG_INSTRUMENTATION

ros::Subscriber pathSubscriber;
ros::Subscriber oGridSubscriber;
ros::Publisher replan_trigger;


std::string robot_name_ = "";

nav_msgs::OccupancyGrid global_oGrid_;


void oGridCB(nav_msgs::OccupancyGrid oGrid)
{
  global_oGrid_ = oGrid;
}

void pathCB(nav_msgs::Path path)
{
  std_msgs::Bool result;
  if(DynamicPlanning::checkForObstacles(path, global_oGrid_))
  {
    result.data = true;
    replan_trigger.publish(result);
  }
  else{
    
    result.data = false;
    replan_trigger.publish(result);

  }
}


int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "dynamic_planner");

  std::string robot_name(argv[1]);
  robot_name_ = robot_name;

  //ROS Topic names
  std::string oGrid_topic_ = "/capricorn/"+robot_name_+"/object_detection_map";
  std::string path_topic_ = "/galaga/debug_path";

  //create a nodehandle
  ros::NodeHandle nh;

  //initializing a planner server
  DynamicPlanning dp;

  oGridSubscriber = nh.subscribe(oGrid_topic_, 1000, oGridCB);
  pathSubscriber = nh.subscribe(path_topic_, 1000, pathCB);

  #ifdef DEBUG_INSTRUMENTATION
  replan_trigger = nh.advertise<std_msgs::Bool>("/capricorn/"+ robot_name_ + "/trigger", 1000);
  #endif

  ros::spin();

  return 0;
}
