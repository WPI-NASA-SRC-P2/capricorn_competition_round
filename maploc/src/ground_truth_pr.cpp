/*
Copied and modified from qualification phase: https://github.com/WPI-NASA-SRC-P2/TeamCapricorn/blob/master/capricorn_examples/src/publish_cheat_odom.cpp
MODIFIED BY: Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <maploc/PosePR.h>
#include <utils/common_names.h>
#include <math.h>

#define UPDATE_HZ 10

ros::ServiceClient client;
gazebo_msgs::GetModelState req;
std::string model_name;
ros::Publisher pr_pub;

geometry_msgs::Pose ground_truth_3d() {
    req.request.model_name = model_name;
    req.request.relative_entity_name = COMMON_NAMES::PROCESSING_PLANT_GAZEBO;
    if (client.call(req))
    {
        return req.response.pose;
    }
    else
    {
        ROS_ERROR("Something went wrong in getting robot pose from gazebo.");
    }
}

void ground_truth_2d() {
    req.request.model_name = COMMON_NAMES::REPAIR_STATION_GAZEBO;
    req.request.relative_entity_name = COMMON_NAMES::PROCESSING_PLANT_GAZEBO;
    if (client.call(req))
    {
        geometry_msgs::Pose pose_rs = req.response.pose, pose_robot = ground_truth_3d();

        float radius = sqrt(pow(pose_robot.position.x, 2) +  pow(pose_robot.position.y, 2) * 1.0);

        float dot = pose_rs.position.x * pose_robot.position.x + pose_rs.position.y * pose_robot.position.y;
        float det = pose_rs.position.x * pose_robot.position.y - pose_rs.position.y * pose_robot.position.x;
        float angle = atan2(det, dot);

        if(angle < 0) {
            angle = 2 * 3.1459 + angle;
        }
        
        maploc::PosePR pr_msg;
        pr_msg.angle = angle;
        pr_msg.radius = radius;
        pr_msg.robot_name = model_name;
        
        pr_pub.publish(pr_msg);
    }
    else
    {
        ROS_ERROR("Something went wrong in getting repair station from gazebo.");
    }
}

/**
 * DISCLAIMER: "SHOULD NOT BE USED IN SUBMISSION, just for testing and debugging" 
 */ 
int main(int argc, char **argv)
{
    model_name = std::string(argv[1]);
    ros::init(argc, argv, model_name + COMMON_NAMES::GROUND_TRUTH_PR_NODE_NAME);
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);

    client = nh.serviceClient<gazebo_msgs::GetModelState>(COMMON_NAMES::MODEL_STATE_QUERY);

    std::string gt_topic_name = COMMON_NAMES::CAPRICORN_TOPIC + model_name + COMMON_NAMES::GROUND_TRUTH_TOPIC;
    std::string pr_topic_name = COMMON_NAMES::CAPRICORN_TOPIC + model_name + COMMON_NAMES::PR_GROUND_TRUTH_TOPIC;

    ros::Publisher gt_pub = nh.advertise<geometry_msgs::PoseStamped>(gt_topic_name, 1, true);
    pr_pub = nh.advertise<maploc::PosePR>(pr_topic_name, 1, true);

    ros::Rate update_rate(UPDATE_HZ);
    geometry_msgs::PoseStamped gt_msg;
    gt_msg.header.frame_id = "map";

    while (ros::ok())
    {
        ground_truth_2d();
        gt_msg.pose = ground_truth_3d();
        gt_pub.publish(gt_msg);
        update_rate.sleep();
    }
    return 0;
}