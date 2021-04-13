#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "planning/trajectory.h"
#include "planning/TrajectoryWithVelocities.h"
#include "trajectory_methods.h"
#include <mutex>

class PathServer {
public:
  static geometry_msgs::PoseStamped global_location;
  static nav_msgs::OccupancyGrid global_oGrid;

  static std::mutex oGrid_mutex;
  static std::mutex location_mutex;

  static ros::Subscriber oGrid_subscriber;
  static ros::Subscriber location_subscriber;
  static void oGridCallback(nav_msgs::OccupancyGrid oGrid);
  static void locationCallback(geometry_msgs::PoseStamped location);
  static bool trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res);
};