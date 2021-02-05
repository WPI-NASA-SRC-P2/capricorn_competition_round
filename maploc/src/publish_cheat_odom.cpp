#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_broadcaster.h>

#define UPDATE_HZ 10

/**
 * @brief This node is used to publish the ground truth position of the model given in the argument.
 * It is just for testing and debugging should not be used in the finals.
 * 
 * It takes the ground truth pose from gazebo for a particular model and publishes it on a topic "/capricorn/model_name/cheat_odom", 
 * message type is nav_msgs/Odometry. The messages are published at 10 Hz frequency.
 * 
 * All the positions are relative to the "heightmap" entity in the simulation which is assumed to be the origin (also appears to be the origin as well)
 *  
 * @param argc 
 * @param argv : Model name (Required, eg. small_scout_1, small_excavator_2 etc.)
 * @return int 
 */
int main(int argc, char **argv)
{
    std::string node_name = std::string(argv[1]) + "_cheat_odom_publisher";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);

    std::string model_name = std::string(argv[1]);
    std::string topic_name = "/capricorn/" + model_name + "/cheat_odom";
    // std::string topic_name = "/" + model_name + "_loc/filtered_odometry";
    std::string robot_frame_name = model_name + "_base_footprint";

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(topic_name, 1, true);

    static tf::TransformBroadcaster br;
    tf::Transform transform;

    gazebo_msgs::GetModelState req;
    req.request.model_name = model_name;
    req.request.relative_entity_name = "heightmap";
    nav_msgs::Odometry odom_msg;
    tf::Quaternion quat;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = robot_frame_name;

    ros::Rate update_rate(UPDATE_HZ);

    while (ros::ok())
    {
      if (client.call(req))
      {
        transform.setOrigin(tf::Vector3(req.response.pose.position.x, req.response.pose.position.y, req.response.pose.position.z));
        tf::quaternionMsgToTF(req.response.pose.orientation, quat);
        transform.setRotation(quat);
        odom_msg.pose.pose = req.response.pose;
        odom_msg.twist.twist = req.response.twist;
        odom_msg.header = req.response.header;
        odom_pub.publish(odom_msg);
      }
      else
      {
        std::string filename = std::string(argv[0]);
        int index = filename.find_last_of('/');
        std::string input_trace_filename = filename.substr(index + 1);
        ROS_ERROR("Something went wrong in file %s getting odom from gazebo.", input_trace_filename);
      }
      update_rate.sleep();
    }
    return 0;
}