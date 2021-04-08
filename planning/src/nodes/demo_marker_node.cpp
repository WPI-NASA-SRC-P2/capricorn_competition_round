#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

ros::Publisher demoMarkerPublisher;
ros::Publisher locationPublisher;
ros::Publisher targetPublisher;

visualization_msgs::Marker marker;
double lastTime;

using geometry_msgs::Point;

void callback(nav_msgs::Path path) {
    marker.header.stamp = ros::Time();

    auto dt = ros::Time().now().toSec() - lastTime;

    marker.pose.position.x = (marker.pose.position.x - path.poses.at(0).pose.position.x) * dt;
    marker.pose.position.y = (marker.pose.position.y - path.poses.at(0).pose.position.y) * dt;

    demoMarkerPublisher.publish(marker);

    Point origin;
    origin.x = 200;
    origin.y = 200;

    Point target;
    target.x = 0;
    target.y = 0;

    locationPublisher.publish(origin);
    targetPublisher.publish(target);
    
    lastTime = ros::Time().now().toSec();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_marker_node");

    ros::NodeHandle nh;
    
    demoMarkerPublisher = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    locationPublisher = nh.advertise<geometry_msgs::Point>("target_point", 0);
    locationPublisher = nh.advertise<geometry_msgs::Point>("origin_point", 0);

    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.header.frame_id = "map";

    lastTime = ros::Time().now().toSec();

    nh.subscribe("/capricorn/navigation_path", 1000, callback);
    
    ros::spin();

    return 0;
}