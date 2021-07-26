#include<team_level/detected_volatile_register.h>

using namespace COMMON_NAMES;

DetectedVolatileRegister *volatile_register, *volatile_register_1;

/**
 * @brief Callback for the navigation testing topic
 * 
 * @param twist geometry_msgs::Point for the goal 
 */
void checkCB(geometry_msgs::PoseStamped goal_point)
{
  ROS_WARN_STREAM("Checking Volatile"<<(volatile_register_1->isNewVolatile(SCOUT_1, goal_point)));
}

/**
 * @brief Callback for the navigation testing topic
 * 
 * @param twist geometry_msgs::PoseStamped for the goal 
 */
void registerCB(geometry_msgs::PoseStamped goal_point)
{
  volatile_register->registerNewVolatile(SCOUT_1, goal_point);
  ROS_WARN_STREAM("Registering Volatile");
}

int main(int argc, char **argv)
{
    // Robot Name from argument
    ros::init(argc, argv, "volatile_register_tester");
    ros::NodeHandle nh;

    // Subscribing to teleop topic
    ros::Subscriber navigation_sub = nh.subscribe("/register", 1000, registerCB);
    ros::Subscriber check_sub = nh.subscribe("/check", 1000, checkCB);

    volatile_register = new DetectedVolatileRegister(nh);
    volatile_register_1 = new DetectedVolatileRegister(nh);

    ros::spin();
    
    return 0;
}