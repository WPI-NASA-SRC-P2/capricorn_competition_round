/*
Copied and modified from qualification phase: https://github.com/WPI-NASA-SRC-P2/TeamCapricorn/blob/master/capricorn_examples/src/publish_cheat_odom.cpp
MODIFIED BY: Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_broadcaster.h>
#include <utils/common_names.h>

#define UPDATE_HZ 200

/**
 * @brief This node is used to publish the ground truth position of the model given in the argument.
 * It is just for testing and debugging should not be used in the finals.
 * 
 * It takes the ground truth pose from gazebo for a particular model and publishes it on a topic "/capricorn/model_name/cheat_odom", 
 * message type is nav_msgs/Odometry. The messages are published at 10 Hz frequency.
 * 
 * All the positions are relative to the "heightmap" entity in the simulation which is assumed to be the origin (also appears to be the origin as well).
 * Changes the reference frame id to "odom"
 *  
 * @param argc 
 * @param argv : Model name (Required, eg. small_scout_1, small_excavator_2 etc.)
 * @return int 
 */
int main(int argc, char **argv)
{
    std::string model_name = std::string(argv[1]);
    ros::init(argc, argv, model_name + COMMON_NAMES::CHEAT_ODOM_PUB_NODE_NAME);
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);

    std::string topic_name = COMMON_NAMES::CAPRICORN_TOPIC + model_name + COMMON_NAMES::CHEAT_ODOM_TOPIC;
    std::string robot_frame_name = model_name + COMMON_NAMES::ROBOT_BASE;

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>(COMMON_NAMES::MODEL_STATE_QUERY);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(topic_name, 1, true);

    static tf::TransformBroadcaster br;
    tf::StampedTransform transform;

    gazebo_msgs::GetModelState req;
    req.request.model_name = model_name;

    // HEIGHTMAP is considered as origin of the simulation (as per the observation it is)
    req.request.relative_entity_name = COMMON_NAMES::HEIGHTMAP;
    nav_msgs::Odometry odom_msg;
    tf::Quaternion quat;
    odom_msg.header.frame_id = COMMON_NAMES::ODOM;
    odom_msg.child_frame_id = robot_frame_name;

    ros::Rate update_rate(UPDATE_HZ);

    while (ros::ok())
    {
      if (client.call(req))
      {
        transform.setOrigin(tf::Vector3(req.response.pose.position.x, req.response.pose.position.y, req.response.pose.position.z));
        tf::quaternionMsgToTF(req.response.pose.orientation, quat);
        transform.setRotation(quat);
        transform.frame_id_ = COMMON_NAMES::MAP;
        transform.child_frame_id_ = model_name + "_" + COMMON_NAMES::ROBOT_BASE;
        transform.stamp_ = req.response.header.stamp;

        br.sendTransform(transform);

        odom_msg.pose.pose = req.response.pose;
        odom_msg.twist.twist = req.response.twist;
        odom_msg.header = req.response.header;
        odom_msg.header.frame_id = COMMON_NAMES::MAP;
        odom_pub.publish(odom_msg);
      }
      else
      {
        ROS_ERROR("Something went wrong in getting odom from gazebo.");
      }
      update_rate.sleep();
    }
    return 0;
}
