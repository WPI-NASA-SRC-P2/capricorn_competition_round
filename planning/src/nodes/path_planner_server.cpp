#include <path_planner_server.h>
#include <cspace.h>
#include <astar.h>

const std::string oGrid_topic = "";
const std::string location_topic = "";

//Setting the node's update rate
#define UPDATE_HZ 10

bool PathServer::trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res)
{
  std::unique_lock<std::mutex> oGridLock(oGrid_mutex);
  auto paddedGrid = CSpace::getCSpace(global_oGrid, 50, 3);
  oGridLock.unlock();

  std::unique_lock<std::mutex> locationLock(location_mutex);
  auto path = AStar::findPathOccGrid(paddedGrid, req.targetPose.pose.position, global_location.pose.position);
  locationLock.unlock();

  planning::TrajectoryWithVelocities trajectory;

  trajectory.poses = path.poses;
  // trajectory.velocities = std::vector<std_msgs::Float64>(trajectory.poses.size(), (std_msgs::Float64)2.0);

  res.trajectory = trajectory;

  return true;
}

void PathServer::oGridCallback(nav_msgs::OccupancyGrid oGrid) {
  std::lock_guard<std::mutex> lock(oGrid_mutex);
  global_oGrid = oGrid;
}

void PathServer::locationCallback(geometry_msgs::PoseStamped location) {
  std::lock_guard<std::mutex> lock(location_mutex);
  global_location = location;
}

int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "path_planner_server");

  //create a nodehandle
  ros::NodeHandle nh;

  PathServer::oGrid_subscriber = nh.subscribe(oGrid_topic, 1000, PathServer::oGridCallback);
  PathServer::location_subscriber = nh.subscribe(location_topic, 1000, PathServer::locationCallback);

  //Instantiating ROS server for generating trajectory
  ros::ServiceServer service = nh.advertiseService("trajectoryGenerator", &PathServer::trajectoryGeneration);

  ros::spin();
  ros::Duration(10).sleep();

  return 0;
}
