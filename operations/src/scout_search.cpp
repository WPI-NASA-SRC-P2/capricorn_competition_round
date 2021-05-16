/*
Author BY: Ashay Aswale, Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

Command Line Arguments Required:
1. robot_name: eg. small_scout_1, small_excavator_2
*/
#include <mutex>
#include <operations/NavigationAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <operations/obstacle_avoidance.h>

#define UPDATE_HZ 10

typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;

Client* g_client;

operations::NavigationGoal g_nav_goal;
perception::ObjectArray g_objects;

const float INIT_VALUE = -100.00, FORWARD_VELOCITY = 0.8;
std::mutex g_objects_mutex;
int g_lost_detection_times = 0, g_true_detection_times = 0, g_revolve_direction = -1;

/**
 * @brief Callback function which subscriber to Objects message published from object detection
 * 
 * @param objs 
 */
void objectsCallback(const perception::ObjectArray& objs) 
{
    const std::lock_guard<std::mutex> lock(g_objects_mutex); 
    g_objects = objs;
}

/**
 * @brief Function for navigating a robot near to an object detection based class
 * 
 */
void spiralSearch()
{
    const std::lock_guard<std::mutex> lock(g_objects_mutex); 
    perception::ObjectArray objects = g_objects;

    std::vector<perception::Object> obstacles;

    for(int i = 0; i < objects.number_of_objects; i++) 
        obstacles.push_back(objects.obj.at(i));

    float direction = checkObstacle(obstacles);

    if(abs(direction) > 0.0)
    {
        g_nav_goal.forward_velocity = FORWARD_VELOCITY;
        g_nav_goal.direction = direction;
        g_nav_goal.angular_velocity = 0;
        ROS_INFO("Avoiding Obstacle");
        return;
    }
    else
    {
        g_nav_goal.forward_velocity = FORWARD_VELOCITY;
        g_nav_goal.direction = 0;
        ROS_INFO("Driving Straight");
    }
}

/**
 * @brief Function which gets at the start
 */
void execute()
{
    ros::Rate update_rate(UPDATE_HZ);
    
    while (ros::ok())
    {
        spiralSearch();
        update_rate.sleep();
        g_client->sendGoal(g_nav_goal);
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    if(argc != 2 && argc != 4)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");
        return -1;
    }

    std::string robot_name(argv[1]);
    ros::init(argc, argv, robot_name + COMMON_NAMES::SCOUT_SEARCH_NODE_NAME);
    ros::NodeHandle nh;

    g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
    g_client = new Client(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);

    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objectsCallback);

    ROS_INFO_STREAM("Starting Searching - "<<robot_name);
    execute();
    return 0;
}
