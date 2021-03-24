#include <ros/ros.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>
#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

float height = 480;
float width = 640;

std::string robot_name;

#define UPDATE_HZ 20

void objects_callback(const perception::ObjectArray& objects) 
{  

}

int main(int argc, char** argv)
{
    std::string name(argv[1]);
    robot_name = name;

    ros::init(argc, argv, robot_name + COMMON_NAMES::NAVIGATION_VISION_NODE_NAME);
    ros::NodeHandle nh;
    
    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objects_callback);

    ros::spin();
    return 0;
}