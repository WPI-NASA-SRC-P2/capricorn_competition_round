#include "path_planner_server.h"

#include <cspace.h>
#include <astar.h>
#include "planning/TrajectoryWithVelocities.h"
#include <geometry_msgs/Point.h>
#include "dyn_planning_2.h"
#include "planning/GetCSpace.h"


//Setting the node's update rate
#define UPDATE_HZ 10

#define DEBUG_INSTRUMENTATION

#ifdef DEBUG_INSTRUMENTATION
ros::Publisher debug_oGridPublisher;
ros::Publisher debug_pathPublisher;
#endif

std::string robot_name_ = "";
geometry_msgs::PoseStamped robot_pose_;
bool map_received = false;
int padding = 2;
ros::ServiceClient client;

bool PathServer::trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res)
{
  std::unique_lock<std::mutex> locationLock(oGrid_mutex_);
  auto global_oGrid_CPY = global_oGrid_;
  locationLock.unlock();


  //calling GetCSpace Service for padded grid
  planning::GetCSpace cspace_srv;
  cspace_srv.request.cspace_length = padding;
  cspace_srv.request.map = global_oGrid_CPY;

  if (client.call(cspace_srv))
  {
    ROS_INFO("Calculating CSpace");
  }
  else
  {
    ROS_ERROR("Failed to Get CSpace service");
    return false;
  }
    auto paddedGrid =  cspace_srv.response.map;

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
    ROS_WARN("[planning | astar | %s ]: No Poses Set.", robot_name_.c_str());
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

  //initializing a planner server
  PathServer server;

  server.oGrid_subscriber = nh.subscribe(oGrid_topic_, 1000, &PathServer::oGridCallback, &server);

  #ifdef DEBUG_INSTRUMENTATION
  debug_oGridPublisher = nh.advertise<nav_msgs::OccupancyGrid>("/capricorn/" + robot_name_ + "/debug_oGrid", 1000);
  debug_pathPublisher = nh.advertise<nav_msgs::Path>("/capricorn/"+ robot_name_ + "/debug_path", 1000);
  #endif


  while(!map_received && ros::ok())
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  } 
  client = nh.serviceClient<planning::GetCSpace>("calc_cspace");
  client.waitForExistence();
  
  //Instantiating ROS server for generating trajectory
  ros::ServiceServer service = nh.advertiseService("trajectoryGenerator", &PathServer::trajectoryGeneration, &server);


  ros::spin();

  return 0;
}
