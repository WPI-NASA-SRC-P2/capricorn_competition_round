#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include "planning/trajectory.h"
#include <mutex>
#include <nav_msgs/Odometry.h>

class PathServer
{
private:

  std::string robot_name_ = "";
  
  nav_msgs::OccupancyGrid global_oGrid_;

  std::mutex oGrid_mutex_;

public:
  ros::Subscriber oGrid_subscriber;

  void oGridCallback(nav_msgs::OccupancyGrid oGrid);
  void locationCallback(nav_msgs::Odometry location);
  bool trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res);
};