#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/GetModelState.h>

#include <rtabmap_ros/ResetPose.h>

#include <utils/common_names.h>

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char *argv[])
{
    //Setup robot_name from passed in arguments
    std::string robot_name(argv[1]);

    //Startup ROS
    ros::init(argc, argv, "odom_initialize_"+robot_name);

    ros::NodeHandle nh;

    geometry_msgs::PoseWithCovarianceStamped true_pose;

    ros::ServiceClient gazebo_client;
    ros::ServiceClient nasa_client;
    
    ROS_INFO("Initializing odom with gazebo");

    gazebo_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    gazebo_msgs::GetModelState state;
    state.request.model_name = robot_name;
    state.request.relative_entity_name = "heightmap";

    if(gazebo_client.call(state))
    {
        true_pose.pose.pose = state.response.pose;
        true_pose.header    = state.response.header;
    }
    else
    {
        ROS_ERROR("Gazebo client call on odom initialization failed.");
        return -1;
    }
        
    true_pose.header.seq = 0;
    true_pose.header.frame_id = robot_name + "_odom";

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;

    geometry_msgs::PoseStamped stampedPose;
    stampedPose.pose = true_pose.pose.pose;
    stampedPose.header = true_pose.header;

    tf2::Quaternion q(stampedPose.pose.orientation.x,
                        stampedPose.pose.orientation.y,
                        stampedPose.pose.orientation.z,
                        stampedPose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);

    ros::ServiceClient rtabmap_client = nh.serviceClient<rtabmap_ros::ResetPose>("/" + robot_name + "/camera/reset_odom_to_pose");
    rtabmap_ros::ResetPose pose;
    pose.request.x = stampedPose.pose.position.x;
    pose.request.y = stampedPose.pose.position.y;
    pose.request.z = stampedPose.pose.position.z;

    pose.request.roll = r;
    pose.request.pitch = p;
    pose.request.yaw = y;

    std::cout << "Waiting for rtabmap client" << std::endl;
    
    rtabmap_client.waitForExistence();

    std::cout << "Rtabmap client loaded" << std::endl;

    if(rtabmap_client.call(pose))
    {
        std::cout << "Pose initialized for rtabmap" << std::endl;
    }
    else
    {
        std::cout << "RTabMap initialize pose failed." << std::endl;
    }
}