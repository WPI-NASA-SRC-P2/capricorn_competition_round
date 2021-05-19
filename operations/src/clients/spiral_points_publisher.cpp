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

  int s_x = std::stoi(argv[3]);
  int s_y = std::stoi(argv[4]);

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


  visualization_msgs::Marker marker_arrow;
  ros::Publisher vis_pub_arrow = nh.advertise<visualization_msgs::Marker>("visualization_marker_arrow", 0);

  marker_arrow.header.frame_id = "map";
  marker_arrow.header.stamp = ros::Time();
  // marker_arrow.ns = "my_namespace";
  marker_arrow.id = 0;
  marker_arrow.type = visualization_msgs::Marker::ARROW;
  marker_arrow.action = visualization_msgs::Marker::ADD;
  marker_arrow.scale.x = 5;
  marker_arrow.scale.y = 2;
  marker_arrow.scale.z = 1;
  marker_arrow.color.a = 1; // Don't forget to set the alpha!
  marker_arrow.color.r = 0.0;
  marker_arrow.color.g = 0.0;
  marker_arrow.color.b = 1.0;


  ros::Duration(0.1).sleep();
  geometry_msgs::Pose zero_point, one_point;
  zero_point.position.x = x;
  zero_point.position.y = y;
  one_point.position.x = s_x;
  one_point.position.y = s_y;

  marker_arrow.pose = NavigationAlgo::getPointCloserToOrigin(zero_point, one_point, 5);

  marker.points.resize(3);
  marker.points.at(0) = zero_point.position;
  marker.points.at(1) = one_point.position;
  marker.points.at(2) = marker_arrow.pose.position;
  
  marker.colors.resize(3);
  marker.colors.at(0) = (marker.color);
  marker.colors.at(1) = (marker.color);
  marker.colors.at(2) = (marker.color);
  marker.colors.at(1).r = 1.0;
  marker.colors.at(1).g = 0.0;
  marker.colors.at(2).b = 1.0;
  marker.colors.at(2).g = 0.0;

  ROS_INFO_STREAM(marker_arrow.pose);
  // marker_arrow.pose.orientation = temp_pose.orientation;
  while (ros::ok())
  {
    vis_pub.publish(marker);
    ros::Duration(0.5).sleep();
    vis_pub_arrow.publish(marker_arrow);
    ros::Duration(0.5).sleep();
    /* code */
  }

  spinner.stop();
  return 0;
}