#include <visualization_msgs/Marker.h>
#include <operations/navigation_algorithm.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spiral_points_publisher");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  visualization_msgs::Marker marker;
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  // marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 2;
  marker.scale.y = 2;
  marker.scale.z = 2;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 0.5; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;



  visualization_msgs::Marker marker_circle;
  ros::Publisher vis_circ_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker_circle", 0);

  marker_circle.header.frame_id = "map";
  marker_circle.header.stamp = ros::Time();
  //_circle marker.ns = "my_namespace";
  marker_circle.id = 0;
  marker_circle.type = visualization_msgs::Marker::CYLINDER;
  marker_circle.action = visualization_msgs::Marker::ADD;
  //_circle marker.scale.x = 2;
  //_circle marker.scale.y = 2;
  //_circle marker.scale.z = 2;
  marker_circle.pose.orientation.x = 0.0;
  marker_circle.pose.orientation.y = 0.0;
  marker_circle.pose.orientation.z = 0.0;
  marker_circle.pose.orientation.w = 1.0;
  marker_circle.color.a = 0.5; // Don't forget to set the alpha!
  marker_circle.color.r = 1.0;
  marker_circle.color.g = 0.0;
  marker_circle.color.b = 0.0;



  ros::Duration(0.1).sleep();
  geometry_msgs::PointStamped zero_point;
  zero_point.header.frame_id = "map";
  std::vector<geometry_msgs::PointStamped> spiral_points = NavigationAlgo::getNArchimedeasSpiralPoints(zero_point, 300, 1);

  geometry_msgs::PointStamped center = NavigationAlgo::getCenterOfThreePointsCircle(spiral_points);
  double radius = NavigationAlgo::getRadiusOfThreePointsCircle(spiral_points);
  ROS_INFO_STREAM(center);
  ROS_INFO_STREAM(radius);
  marker_circle.pose.position.x = center.point.x;
  marker_circle.pose.position.y = center.point.y;
  marker_circle.pose.position.z = 0;
  marker_circle.scale.x = 2*radius;
  marker_circle.scale.y = 2*radius;
  marker_circle.scale.z = 0.5;

  marker.points.resize(spiral_points.size());

  for (int i = 0; i < spiral_points.size(); i++)
    marker.points.at(i) = spiral_points.at(i).point;

  while (ros::ok())
  {
    vis_pub.publish(marker);
    ros::Duration(1).sleep();
    vis_circ_pub.publish(marker_circle);
    /* code */
  }

  spinner.stop();
  return 0;
}