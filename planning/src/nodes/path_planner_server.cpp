#include "path_planner_server.h"

#include <cspace.h>
#include <astar.h>
#include "planning/TrajectoryWithVelocities.h"
#include <geometry_msgs/Point.h>

//Setting the node's update rate
#define UPDATE_HZ 10

bool PathServer::trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res)
{
  std::unique_lock<std::mutex> oGridLock(oGrid_mutex_);
  auto global_location_CPY = global_location_;
  oGridLock.unlock();

  std::unique_lock<std::mutex> locationLock(location_mutex_);
  auto global_oGrid_CPY = global_oGrid_;
  locationLock.unlock();

  auto paddedGrid = CSpace::getCSpace(global_oGrid_CPY, 50, 3);
  auto path = AStar::findPathOccGrid(paddedGrid, req.targetPose.pose.position, global_location_CPY.pose.position);

  planning::TrajectoryWithVelocities trajectory;

  trajectory.waypoints = path.poses;
  // trajectory.velocities = std::vector<std_msgs::Float64>(trajectory.poses.size(), (std_msgs::Float64)2.0);

  res.trajectory = trajectory;

  return true;
}

void PathServer::oGridCallback(nav_msgs::OccupancyGrid oGrid)
{
  std::lock_guard<std::mutex> lock(oGrid_mutex_);
  global_oGrid_ = oGrid;
}

void PathServer::locationCallback(geometry_msgs::PoseStamped location)
{
  std::lock_guard<std::mutex> lock(location_mutex_);
  global_location_ = location;
}

int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "path_planner_server");


  std::string robot_name(argv[1]);

  //ROS Topic names
  std::string oGrid_topic_ =  "/" + robot_name +  "/camera/grid_map";
  std::string location_topic_ = "/" + robot_name + "/camera/odom";

  //create a nodehandle
  ros::NodeHandle nh;

  PathServer server;

  server.oGrid_subscriber = nh.subscribe(oGrid_topic_, 1000, &PathServer::oGridCallback, &server);
  server.location_subscriber = nh.subscribe(location_topic_, 1000, &PathServer::locationCallback, &server);

  //Instantiating ROS server for generating trajectory
  ros::ServiceServer service = nh.advertiseService("trajectoryGenerator", &PathServer::trajectoryGeneration, &server);

  ros::spin();

  return 0;
}
