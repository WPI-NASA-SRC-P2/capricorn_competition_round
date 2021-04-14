#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include "planning/trajectory.h"
#include <mutex>

class PathServer {
private:
  geometry_msgs::PoseStamped global_location_;
  nav_msgs::OccupancyGrid global_oGrid_;

  std::mutex oGrid_mutex_;
  std::mutex location_mutex_;

public:
  ros::Subscriber oGrid_subscriber;
  ros::Subscriber location_subscriber;

  const std::string oGrid_topic_ = "";
  const std::string location_topic_ = "";

  void oGridCallback(nav_msgs::OccupancyGrid oGrid);
  void locationCallback(geometry_msgs::PoseStamped location);
  bool trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res);
};