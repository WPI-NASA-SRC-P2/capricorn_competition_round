#include <ros/ros.h>
#include <utils/common_names.h>
#include <geometry_msgs/PoseStamped.h>

using namespace COMMON_NAMES;

void doSomething(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("Sending robot task & volatile location [%f]\n", msg->pose.position.x);

    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "scheduler"); 
    ros::NodeHandle nh;

    /**
    *   Put publishers here
    *   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    */

    // /capricorn/scheduler/volatile_location
    std::string vol_location_topic = CAPRICORN_TOPIC + SCHEDULER_TOPIC + VOLATILE_LOCATION_TOPIC;

    ros::Subscriber sub = nh.subscribe(vol_location_topic, 1000, doSomething);
    

    ros::Rate loop_rate(10);
    
    //int count = 0;
    while(ros::ok())
    {
        //ROS_INFO("test\n");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
