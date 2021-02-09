/*
Copied and modified from qualification phase: https://github.com/WPI-NASA-SRC-P2/TeamCapricorn/blob/master/capricorn_odom/src/capricorn_odom_error.cpp
MODIFIED BY: Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <maploc/OdomError.h>

#include <gazebo_msgs/GetModelState.h>
#include <math.h>

#include <utils/common_names.h>

#define UPDATE_HZ 5

geometry_msgs::PoseWithCovarianceStamped last_gazebo_pose;
maploc::OdomError error;

std::string robot_name;

/*
 * Calculates the difference between the filtered (robot_localization) pose and actual (gazebo/get_model_state) pose.
 * Determines the distance and angle from the estimated pose to the actual pose in 2D as 2D coordinates are of our interest.
*/
void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    float dy = last_gazebo_pose.pose.pose.position.y - odom->pose.pose.position.y;
    float dx = last_gazebo_pose.pose.pose.position.x - odom->pose.pose.position.x;

    float dtheta = atan2(dy, dx);

    error.dist_error = sqrt(pow(dy, 2) + pow(dx, 2));
    error.theta_error = (360 * dtheta) / (2 * M_PI);
}

int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        ROS_INFO("Arguments not correct");
        ROS_INFO("Usage rosrun maploc odom_error robot_model_name odom_topic_to_be_compared");
        return -1;
    }

    std::string name(argv[1]);
    std::string robot_name = name;

    std::string odom_topic(argv[2]);
    
    //Setup ROS
    ros::init(argc, argv, COMMON_NAMES::ODOM_ERROR_NODE_NAME);

    ros::NodeHandle nh;
    ros::Rate update_rate(UPDATE_HZ);

    //Subscribe to our robot's filtered odometry. Since we are running in the /capricorn/robot/odom ns, we need no prefix
    ros::Subscriber localization_out = nh.subscribe(odom_topic, 10, odom_callback);

    ros::Publisher error_pub = nh.advertise<maploc::OdomError>(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::POSE_ERROR_TOPIC, 1, true);

    //Setup Gazebo get_model_state service
    ros::ServiceClient gazebo_client = nh.serviceClient<gazebo_msgs::GetModelState>(COMMON_NAMES::MODEL_STATE_QUERY);

    gazebo_msgs::GetModelState state;
    state.request.model_name = robot_name;
    state.request.relative_entity_name = COMMON_NAMES::HEIGHTMAP;

    while(ros::ok())
    {
        //Get the gazebo pose (relative to lunar_terrain) and update the global pose, to be used in the callback
        if(gazebo_client.call(state))
        {
            last_gazebo_pose.pose.pose = state.response.pose;
            last_gazebo_pose.header = state.response.header;
        }

        //Publish the OdomError to /robot/pose_error
        error_pub.publish(error);

        ros::spinOnce();
        update_rate.sleep();
    }
}