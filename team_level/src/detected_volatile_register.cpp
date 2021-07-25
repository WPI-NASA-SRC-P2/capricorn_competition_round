#include <team_level/detected_volatile_register.h>

DetectedVolatileRegister::DetectedVolatileRegister(ros::NodeHandle nh)
{
   scout_1_subscriber = nh.subscribe(SCOUT_1_NAME + VOLATILE_SENSOR_TOPIC, 1000, &DetectedVolatileRegister::scout1VolatileCB, this);

   scout_2_subscriber = nh.subscribe(SCOUT_2_NAME + VOLATILE_SENSOR_TOPIC, 1000, &DetectedVolatileRegister::scout2VolatileCB, this);
}

void DetectedVolatileRegister::scout1VolatileCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg)
{
    scout_1_current_volatile = msg->vol_type;
}

void DetectedVolatileRegister::scout2VolatileCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg)
{
    scout_2_current_volatile = msg->vol_type;
}

bool DetectedVolatileRegister::isCloseEnough(const geometry_msgs::Point& pose_1, const geometry_msgs::Point& pose_2)
{
    float x = pose_1.x - pose_2.x;
    float y = pose_1.y - pose_2.y;

    float dist = std::hypot(x,y);
    // ROS_INFO_STREAM(dist);
    return dist<DISTANCE_THRESHOLD;
}

bool DetectedVolatileRegister::isNewVolatile(ROBOTS_ENUM robot, const geometry_msgs::PoseStamped& robot_pose)
{
    std::string volatile_name = robot == SCOUT_1 ? scout_1_current_volatile : scout_2_current_volatile;
    if (volatile_name == "none")
        return false;

    bool same_volatile = false;
    for(auto each_volatile = std::begin(DetectedVolatileRegister::detected_volatile_list); each_volatile < std::end(DetectedVolatileRegister::detected_volatile_list); each_volatile++)
    {
        if(each_volatile->first == volatile_name)
        {
            if(isCloseEnough(each_volatile->second, robot_pose.pose.position))
            {
                same_volatile = true;
                break;
            }
        }
    }
    for(auto each_volatile = std::begin(DetectedVolatileRegister::detected_volatile_list); each_volatile < std::end(DetectedVolatileRegister::detected_volatile_list); each_volatile++)
        // ROS_INFO_STREAM(each_volatile->first<<"  "<<each_volatile->second);
    return !same_volatile;
}

void DetectedVolatileRegister::registerNewVolatile(ROBOTS_ENUM robot, geometry_msgs::PoseStamped robot_pose)
{
    std::string volatile_name = robot == SCOUT_1 ? scout_1_current_volatile : scout_2_current_volatile;
    if(isNewVolatile(robot, robot_pose))
        DetectedVolatileRegister::detected_volatile_list.push_back(std::make_pair(volatile_name, robot_pose.pose.position));
}