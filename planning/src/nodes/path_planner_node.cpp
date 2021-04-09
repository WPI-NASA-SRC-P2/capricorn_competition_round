#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "path_planner.h"
#include "planning/trajectory.h"
#include "planning/TrajectoryWithVelocities.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <astar.h>
#include <cspace.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 10

ros::Publisher pathPublisher;
ros::Publisher cSpacePublisher;

ros::Publisher originPublisher;
ros::Publisher targetPublisher;
ros::Publisher markerArrayPublisher;

using geometry_msgs::Point;
using geometry_msgs::PoseStamped;

void callback(const nav_msgs::OccupancyGrid oGrid)
{
  ROS_INFO("reached callback");

  Point origin;

  origin.x = 300;
  origin.y = 300;
  ROS_INFO("STARTED NEW CSPACE");
  auto CSpace = CSpace::GetCSpace(oGrid, 50, 3);

  auto path = AStar::FindPathOccGrid(CSpace, target, origin);

  pathPublisher.publish(path);
  cSpacePublisher.publish(CSpace);

  visualization_msgs::MarkerArray ma;

  for (int i = 1; i < path.poses.size() - 1; i++)
  {
    auto wp = path.poses[i];
    visualization_msgs::Marker mkr;

    mkr.header.frame_id = oGrid.header.frame_id;
    mkr.header.stamp = ros::Time();
    mkr.ns = "path_planner";
    mkr.id = i;
    mkr.type = visualization_msgs::Marker::SPHERE;
    mkr.action = visualization_msgs::Marker::ADD;

    mkr.pose.orientation.x = 0.0;
    mkr.pose.orientation.y = 0.0;
    mkr.pose.orientation.z = 0.0;
    mkr.pose.orientation.w = 1.0;
    mkr.scale.x = .2;
    mkr.scale.y = .2;
    mkr.scale.z = .2;
    mkr.color.a = 1.0; // Don't forget to set the alpha!
    mkr.color.r = 1.0;
    mkr.color.g = 1.0;
    mkr.color.b = 0.0;

    mkr.pose.position.x = wp.pose.position.x;
    mkr.pose.position.y = wp.pose.position.y;

    ma.markers.push_back(mkr);
  }

  markerArrayPublisher.publish(ma);

  visualization_msgs::Marker p1;

  p1.header.frame_id = oGrid.header.frame_id;
  p1.header.stamp = ros::Time();
  p1.ns = "path_planner";
  p1.id = 0;
  p1.type = visualization_msgs::Marker::SPHERE;
  p1.action = visualization_msgs::Marker::ADD;

  p1.pose.orientation.x = 0.0;
  p1.pose.orientation.y = 0.0;
  p1.pose.orientation.z = 0.0;
  p1.pose.orientation.w = 1.0;
  p1.scale.x = 1;
  p1.scale.y = 1;
  p1.scale.z = 1;
  p1.color.a = 1.0; // Don't forget to set the alpha!
  p1.color.r = 1.0;
  p1.color.g = 0.0;
  p1.color.b = 0.0;

  p1.pose.position.x = origin.x / 20;
  p1.pose.position.y = origin.y / 20;

  originPublisher.publish(p1);

  visualization_msgs::Marker p2;

  p2.header.frame_id = oGrid.header.frame_id;
  p2.header.stamp = ros::Time();
  p2.ns = "path_planner";
  p2.id = 0;
  p2.type = visualization_msgs::Marker::SPHERE;
  p2.action = visualization_msgs::Marker::ADD;

  p2.pose.orientation.x = 0.0;
  p2.pose.orientation.y = 0.0;
  p2.pose.orientation.z = 0.0;
  p2.pose.orientation.w = 1.0;
  p2.scale.x = 1;
  p2.scale.y = 1;
  p2.scale.z = 1;
  p2.color.a = 1.0; // Don't forget to set the alpha!
  p2.color.r = 0.0;
  p2.color.g = 1.0;
  p2.color.b = 0.0;

  p2.pose.position.x = target.x / 20;
  p2.pose.position.y = target.y / 20;

  targetPublisher.publish(p2);
}

bool trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res)
{
  //res.trajectory = trajectoryGenerator(req.targetPose);
  ROS_INFO("Entered trajectoryGeneration");
  planning::TrajectoryWithVelocities trajectory;
  res.trajectory = trajectory;
  return true;
}

int main(int argc, char *argv[])
{
  //if(argc != 3) printf("Wrong arg count: %d\n", argc);

  // target.x = atoi(argv[1]);
  // target.y = atoi(argv[2]);

  target.x = 10;
  target.y = 10;

  // ROS initialization
  ros::init(argc, argv, "path_planner");
  ros::NodeHandle nh;

  pathPublisher = nh.advertise<nav_msgs::Path>("/capricorn/navigation_path", 1000);
  cSpacePublisher = nh.advertise<nav_msgs::OccupancyGrid>("/capricorn/cspace", 1000);

  originPublisher = nh.advertise<visualization_msgs::Marker>("/capricorn/origin", 1000);
  targetPublisher = nh.advertise<visualization_msgs::Marker>("/capricorn/target", 1000);

  markerArrayPublisher = nh.advertise<visualization_msgs::MarkerArray>("/capricorn/marker_array", 1000);

  //Subscribe to the node publishing map values
  ros::Subscriber sub = nh.subscribe("/capricorn/Ground_Truth_Map", 1000, callback);
  // ros::Subscriber sub2 = nh.subscribe("/capricorn/target_point", 1000, target_point_callback);

  //Subscribe to the node publishing map values
  ros::Subscriber indexValues = nh.subscribe("/capricorn/Ground_Truth_Map", 1000, callback);

  //creates a service
  ros::ServiceServer service = nh.advertiseService("trajectoryGenerator", trajectoryGeneration);
  ros::spin();

  ros::Duration(10).sleep();

  return 0;
}