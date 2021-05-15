#include <ros/ros.h>
#include <utils/common_names.h>
#include <geometry_msgs/PoseStamped.h>

using namespace COMMON_NAMES;

ros::Publisher vol_location_excavator_pub, vol_location_hauler_pub;

/**
 * @brief Listens to the volatile location topic, and republishes received poses to both the hauler and excavator.
 * 
 * @param vol_loc 
 */
void volLocationCB(const geometry_msgs::PoseStamped::ConstPtr& vol_loc)
{
    ROS_INFO("Received volatile location at [%f, %f]. Sending to hauler and excavator.\n", vol_loc->pose.position.x, vol_loc->pose.position.y);
    vol_location_excavator_pub.publish(vol_loc);
    ros::Duration(10).sleep();
    vol_location_hauler_pub.publish(vol_loc);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "scheduler"); 
    ros::NodeHandle nh;

    // Listens to any messages published about detecting new volatiles.
    ros::Subscriber volatile_location_sub = nh.subscribe("/" + CAPRICORN_TOPIC + SCHEDULER_TOPIC + VOLATILE_LOCATION_TOPIC, 1000, volLocationCB);

    // Publishers to redistribute any messages on the volatile location topic to the hauler and excavator.
    // In the future, we will select which hauler and excavator to send off (or wait for one to be ready), but for now we just send the volatile off to the first of each.
    vol_location_excavator_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + CAPRICORN_TOPIC + "/" + EXCAVATOR_1 + VOLATILE_LOCATION_TOPIC, 1000);
    vol_location_hauler_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + CAPRICORN_TOPIC + "/" + HAULER_1 + VOLATILE_LOCATION_TOPIC, 1000);

    ros::Publisher hauler_lookout_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + CAPRICORN_TOPIC + "/" + HAULER_1 + LOOKOUT_LOCATION_TOPIC, 1000);
    ros::Publisher excavtor_lookout_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + CAPRICORN_TOPIC + "/" + EXCAVATOR_1 + LOOKOUT_LOCATION_TOPIC, 1000);

    ROS_INFO("Scheduler started, listening for volatiles.\n");

    ros::Duration(5).sleep();
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 25;
    pose.pose.position.y = 2;
    excavtor_lookout_pub.publish(pose);

    ros::Duration(10).sleep();
    pose.pose.position.x = 20;
    pose.pose.position.y = 5;
    hauler_lookout_pub.publish(pose);

    ros::spin();

    ROS_WARN("Scheduler died!\n");

    return 0;
}
