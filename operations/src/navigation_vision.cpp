/*
Author: Mahimana Bhatt
Email: mbhatt@wpi.edu
TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include <perception/Find_PP_RSAction.h>
#include <actionlib/client/simple_action_client.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>
#include <utils/common_names.h>

std::string robot_name;

void bb_and_objects_callback(const perception::ObjectArray& objects) 
{

    ros::shutdown();
}

int main(int argc, char** argv)
{
    std::string name(argv[1]);
    robot_name = name;
    ros::init(argc, argv, robot_name + COMMON_NAMES::PR_DATASET_NODE_NAME);

    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    
    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &bb_and_objects_callback);

    ros::spin(); 
    
    return 0;
}