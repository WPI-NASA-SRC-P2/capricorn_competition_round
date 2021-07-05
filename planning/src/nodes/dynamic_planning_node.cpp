#include "path_planner_server.h"

#include <cspace.h>
#include <astar.h>
#include "planning/TrajectoryWithVelocities.h"
#include <geometry_msgs/Point.h>
#include "dyn_planning.h"

//Setting the node's update rate
#define UPDATE_HZ 10

ros::Subscriber pathSubscriber;
ros::Subscriber OccupancyGrid;
ros::Publisher replan_trigger;


std::string robot_name_ = "";


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

  server.oGrid_subscriber = nh.subscribe(oGrid_topic_, 1000, &DynamicPlanning::oGridCallback, &server);
  server.pathSubscriber = nh.subscribe(path_topic_, 1000, &DynamicPlanning::checkForObstacles, &server);

  #ifdef DEBUG_INSTRUMENTATION
  debug_oGridPublisher = nh.advertise<nav_msgs::OccupancyGrid>("/galaga/debug_oGrid", 1000);
  debug_pathPublisher = nh.advertise<nav_msgs::Path>("/galaga/debug_path", 1000);
  #endif

  while(!map_received)
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  //Instantiating ROS server for generating trajectory
  ros::ServiceServer service = nh.advertiseService("trajectoryGenerator", &PathServer::trajectoryGeneration, &server);

  ros::spin();

  return 0;
}
