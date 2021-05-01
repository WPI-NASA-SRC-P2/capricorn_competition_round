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

  ros::Duration(0.1).sleep();
  geometry_msgs::Point zero_point;
  std::vector<geometry_msgs::Point> spiral_points = NavigationAlgo::getNArchimedeasSpiralPoints(zero_point, 10000, 195);

  marker.points.resize(spiral_points.size());

  for (int i = 0; i < spiral_points.size(); i++)
    marker.points.at(i) = spiral_points.at(i);

  while (ros::ok())
  {
    vis_pub.publish(marker);
    ros::Duration(1).sleep();
    /* code */
  }

  spinner.stop();
  return 0;
}