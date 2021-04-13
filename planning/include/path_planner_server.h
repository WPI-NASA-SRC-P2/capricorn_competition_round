#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include "planning/trajectory.h"
#include <mutex>

class PathServer {
public:
  geometry_msgs::PoseStamped global_location;
  nav_msgs::OccupancyGrid global_oGrid;

  std::mutex oGrid_mutex;
  std::mutex location_mutex;

  ros::Subscriber oGrid_subscriber;
  ros::Subscriber location_subscriber;
  void oGridCallback(nav_msgs::OccupancyGrid oGrid);
  void locationCallback(geometry_msgs::PoseStamped location);
  bool trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res);
};