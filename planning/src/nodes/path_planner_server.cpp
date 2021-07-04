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
bool map_received = false;

bool PathServer::trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res)
{
  std::unique_lock<std::mutex> locationLock(oGrid_mutex_);
  auto global_oGrid_CPY = global_oGrid_;
  locationLock.unlock();

  auto paddedGrid = CSpace::getCSpace(global_oGrid_CPY, 50, 6);

  #ifdef DEBUG_INSTRUMENTATION
  debug_oGridPublisher.publish(paddedGrid);
  #endif

  auto path = AStar::findPathOccGrid(paddedGrid, req.targetPose.pose.position, 50, robot_name_);

  
  #ifdef DEBUG_INSTRUMENTATION
  debug_pathPublisher.publish(path);
  #endif

  if(path.poses.size() > 0) {
    planning::TrajectoryWithVelocities trajectory;

    trajectory.waypoints = path.poses;

    res.trajectory = trajectory;
  } else {
    ROS_WARN("No Poses Set.");
  }

  return true;
}

void PathServer::oGridCallback(nav_msgs::OccupancyGrid oGrid)
{
  std::lock_guard<std::mutex> lock(oGrid_mutex_);
  global_oGrid_ = oGrid;
  map_received = true;
}

int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "path_planner_server");

  std::string robot_name(argv[1]);
  robot_name_ = robot_name;

  //ROS Topic names
  std::string oGrid_topic_ = "/capricorn/"+robot_name_+"/object_detection_map";

  //create a nodehandle
  ros::NodeHandle nh;

  PathServer server;

  server.oGrid_subscriber = nh.subscribe(oGrid_topic_, 1000, &PathServer::oGridCallback, &server);

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
