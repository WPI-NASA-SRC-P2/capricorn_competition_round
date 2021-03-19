#include <maploc/ground_truth_pr.h>

/**
 * DISCLAIMER: "SHOULD NOT BE USED IN SUBMISSION, just for testing and debugging" 
 */ 
int main(int argc, char **argv)
{
    robot_name = std::string(argv[1]);
    ros::init(argc, argv, robot_name + COMMON_NAMES::GROUND_TRUTH_PR_NODE_NAME);
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);

    client = nh.serviceClient<gazebo_msgs::GetLinkState>(COMMON_NAMES::LINK_STATE_QUERY);

    std::string gt_topic_name = COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::GROUND_TRUTH_TOPIC;
    std::string pr_topic_name = COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::PR_GROUND_TRUTH_TOPIC;

    ros::Publisher gt_pub = nh.advertise<geometry_msgs::PoseStamped>(gt_topic_name, 1, true);
    ros::Publisher pr_pub;
    pr_pub = nh.advertise<maploc::PosePR>(pr_topic_name, 1, true);

    ros::Rate update_rate(UPDATE_HZ);
    geometry_msgs::PoseStamped gt_msg;
    maploc::PosePR pr_msg;
    gt_msg.header.frame_id = "map";

    while (ros::ok())
    {
        gt_msg.pose = ground_truth_3d(robot_name);
        pr_msg = ground_truth_2d(robot_name, gt_msg.pose);
        
        pr_pub.publish(pr_msg);
        gt_pub.publish(gt_msg);

        update_rate.sleep();
    }
    return 0;
}