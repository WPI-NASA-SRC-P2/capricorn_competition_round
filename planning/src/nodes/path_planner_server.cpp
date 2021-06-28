#include "path_planner_server.h"

#include <cspace.h>
#include <astar.h>
#include "planning/TrajectoryWithVelocities.h"
#include <geometry_msgs/Point.h>


//Setting the node's update rate
#define UPDATE_HZ 10

#define DEBUG_INSTRUMENTATION

#ifdef DEBUG_INSTRUMENTATION
ros::Publisher debug_oGridPublisher;
ros::Publisher debug_pathPublisher;
#endif

std::string robot_name_ = "";
bool map_received_ = false;

bool PathServer::trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res)
{ 
  ROS_INFO("generator started - 1");
  std::unique_lock<std::mutex> locationLock(oGrid_mutex_);
  // nav_msgs::OccupancyGrid global_oGrid_CPY = global_oGrid_;
  ROS_INFO("occgrid assigned - 2");
  locationLock.unlock();

  ROS_INFO("map mutex unlocked - 3");

  CSpace::getCSpace(global_oGrid_, 50, 5); 
  while(ros::ok())
  {
    ROS_INFO("something");
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("padded map generated - 4");

  #ifdef DEBUG_INSTRUMENTATION
  ROS_INFO("before");
  debug_oGridPublisher.publish(global_oGrid_);
  ROS_INFO("after");
  #endif
  ROS_INFO("padded ogrid published - 5");

  ROS_INFO("Path is being calculated .... - 6");
  nav_msgs::Path path = AStar::findPathOccGrid(global_oGrid_, req.targetPose.pose.position, 50, robot_name_);
  ROS_INFO("Path is calculated - 7");

  #ifdef DEBUG_INSTRUMENTATION
  debug_pathPublisher.publish(path);
  #endif
  ROS_INFO("Path is published - 8");

  if(path.poses.size() > 0) {
    planning::TrajectoryWithVelocities trajectory;

    trajectory.waypoints = path.poses;

    res.trajectory = trajectory;

    return true;
  } else {
    ROS_WARN("[planning | path_planner_server | %s]: No Poses Set.", robot_name_);
    return false;
  }

  return false;
 
}

void PathServer::oGridCallback(const nav_msgs::OccupancyGrid& oGrid)
{
  std::lock_guard<std::mutex> lock(oGrid_mutex_);
  global_oGrid_ = oGrid;
  ROS_INFO(" new map received");
  map_received_ = true;
}

int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "path_planner_server");

  std::string robot_name(argv[1]);

  robot_name_ = robot_name;

  //ROS Topic names
  std::string oGrid_topic_ = "/capricorn/"+ robot_name_ +"/object_detection_map";

  //create a nodehandle
  ros::NodeHandle nh;

  PathServer server;

  server.oGrid_subscriber = nh.subscribe(oGrid_topic_, 1000, &PathServer::oGridCallback, &server);

  #ifdef DEBUG_INSTRUMENTATION
  debug_oGridPublisher = nh.advertise<nav_msgs::OccupancyGrid>("/galaga/debug_oGrid", 1000);
  debug_pathPublisher = nh.advertise<nav_msgs::Path>("/galaga/debug_path", 1000);
  #endif

  while(!map_received_ && ros::ok())
  {
    ROS_INFO("Map Not Received");
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  //Instantiating ROS server for generating trajectory
  ros::ServiceServer service = nh.advertiseService("trajectoryGenerator", &PathServer::trajectoryGeneration, &server);

  ros::spin();

  return 0;
}
