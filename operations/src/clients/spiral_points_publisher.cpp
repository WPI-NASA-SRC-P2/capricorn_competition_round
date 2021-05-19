#include <visualization_msgs/Marker.h>
#include <operations/navigation_algorithm.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spiral_points_publisher");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int x = std::stoi(argv[1]);
  int y = std::stoi(argv[2]);

  visualization_msgs::Marker marker;
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  // marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 4;
  marker.scale.y = 4;
  marker.scale.z = 4;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;


  ros::Duration(0.1).sleep();
  geometry_msgs::PointStamped zero_point;
  zero_point.header.frame_id = "map";
  zero_point.point.x = x;
  zero_point.point.y = y;
  geometry_msgs::PointStamped closer_point = NavigationAlgo::getPointCloserToOrigin(zero_point, 10);

  marker.points.resize(2);
  marker.points.at(0) = zero_point.point;
  marker.points.at(1) = closer_point.point;
  
  marker.colors.resize(2);
  marker.colors.at(0) = (marker.color);
  marker.colors.at(1) = (marker.color);
  marker.colors.at(1).r = 1.0;
  marker.colors.at(1).g = 0.0;


  while (ros::ok())
  {
    vis_pub.publish(marker);
    ros::Duration(1).sleep();
    /* code */
  }

  spinner.stop();
  return 0;
}