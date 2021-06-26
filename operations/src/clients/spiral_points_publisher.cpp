#include <visualization_msgs/Marker.h>
#include <operations/navigation_algorithm.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

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
  std_msgs::ColorRGBA green_color, red_color;
  green_color.a = 0.5; // Don't forget to set the alpha!
  green_color.r = 0.0;
  green_color.g = 1.0;
  green_color.b = 0.0;

  red_color.a = 0.5; // Don't forget to set the alpha!
  red_color.r = 1.0;
  red_color.g = 0.0;
  red_color.b = 0.0;

  ros::Duration(0.1).sleep();
  geometry_msgs::PointStamped zero_point;
  zero_point.header.frame_id = "map";
  int number_of_points = 120;
  std::vector<geometry_msgs::PointStamped> spiral_points_1 = NavigationAlgo::getNArchimedeasSpiralPoints(1);
  // std::vector<geometry_msgs::PointStamped> spiral_points_2 = NavigationAlgo::getNArchimedeasSpiralPoints(2);

  marker.points.resize(number_of_points);
  marker.colors.resize(number_of_points);

  for (int i = 0; i < number_of_points; i++)
  {
    marker.points.at(i) = spiral_points_1.at(i).point;
    marker.colors.at(i) = green_color;
    // marker.points.at(number_of_points + i) = spiral_points_2.at(i).point;
    // marker.colors.at(number_of_points + i) = red_color;
  }

  while (ros::ok())
  {
    vis_pub.publish(marker);
    ros::Duration(1).sleep();
    /* code */
  }

  spinner.stop();
  return 0;
}