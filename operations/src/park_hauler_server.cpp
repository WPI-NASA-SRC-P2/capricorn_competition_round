/*
Author BY: Mahimana Bhatt, Anirban Mukherjee, Kishor Sabarish G
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

This is an actionlib server for parking hauler with hopper or excavator

Command Line Arguments Required:
1. robot_name: eg. small_scout_1, small_excavator_2
*/

#include <string>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <operations/ParkRobotAction.h>
#include <operations/NavigationAction.h> 
#include <utils/common_names.h>
#include <perception/ObjectArray.h>
#include <operations/NavigationVisionAction.h>

#define UPDATE_HZ 10

typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> VisionClient;
typedef actionlib::SimpleActionServer<operations::ParkRobotAction> Server;
typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;

enum OBJECT_PARKER
{
    HOPPER = true,
    EXCAVATOR = false
};

Client* g_client;
operations::NavigationGoal g_nav_goal;
const float FORWARD_VELOCITY = 0.4;
std::mutex g_hauler_objects_mutex, g_excavator_objects_mutex;

// global variables for park excavator
const int MIN_DIFF_THRESH = 150, DIFF_CHANGE_THRESH = 20, ROBOT_ANTENNA_HEIGHT_THRESH = 150, ANGLE_THRESHOLD_NARROW = 20, ANGLE_THRESHOLD_WIDE = 80;
const float DEFAULT_RADIUS = 5, ROBOT_RADIUS = 1, WIDTH_IMAGE = 640.0;
bool g_parked = false, g_found_orientation = false;
float g_max_diff = -1;
int g_times_excavator = 0;

// global variables for park hopper
const int TIMES_REACHED_THRESHOLD = 10, DIFF_HOPPER_X_THRESH = 50, DIFF_FURNACE_X_THRESH = 500, HOPPER_ORBIT_THRESH = 320;
const float PROCESSING_PLANT_RADIUS = 1.8, INIT_VALUE = -100.00;
int g_hopper_x, g_processing_plant_z, g_furnace_x, g_hopper_height, g_times_reached = 0, g_center_image_x = 320, g_target_height = 385;
double g_hopper_z;

bool g_execute_called = false;
perception::ObjectArray g_hauler_objects, g_excavator_objects;
std::string robot_name;

/**
 * @brief Callback function which subscriber to Objects message published from hauler's object detection
 * 
 * @param objs 
 */
void haulerObjectsCallback(const perception::ObjectArray& objs) 
{
    const std::lock_guard<std::mutex> lock(g_hauler_objects_mutex); 
    g_hauler_objects = objs;
}

/**
 * @brief Callback function which subscriber to Objects message published from excavator's object detection
 * 
 * @param objs 
 */
void excavatorObjectsCallback(const perception::ObjectArray& objs) 
{
    const std::lock_guard<std::mutex> lock(g_excavator_objects_mutex); 
    g_excavator_objects = objs;
}

/**
 * @brief Function for parking a robot with respect to hopper
 * 
 * Steps:
 * 1. Rotate around hopper assuming robot is already near the processingPlant and has processingPlant in the center of the image
 * 2. Stop rotation when you reach desired orientation i.e. hopper and furnace are center in the image
 * 3. Drive forward until you reach a required hopper bounding box height
 * 
 */
void parkWrtHopper()
{
    //parsing the objects message to extract the necessary information of required object
    const std::lock_guard<std::mutex> lock(g_hauler_objects_mutex);     
    perception::ObjectArray objects = g_hauler_objects;

    int n = objects.number_of_objects; 

    for(int i = 0; i<n; i++)
    {
        perception::Object object = objects.obj.at(i);

        if(object.label == COMMON_NAMES::OBJECT_DETECTION_HOPPER_CLASS)
        {
            //If hopper is detected store g_hopper_x and g_hopper_z 
            g_hopper_x = object.center.x; // in pixels
            g_hopper_z = object.point.pose.position.z;
            g_hopper_height = object.size_y;
            ROS_INFO_STREAM("Detected Hopper");
        }

        if(object.label == COMMON_NAMES::OBJECT_DETECTION_PROCESSING_PLANT_CLASS)
        {
            //If processingPlant is detected store g_processing_plant_z 
            g_processing_plant_z = object.point.pose.position.z; // in meters
        }
        if(object.label == COMMON_NAMES::OBJECT_DETECTION_FURNACE_CLASS)
        {
            //If furnace is detected store g_furnace_x
            g_furnace_x = object.center.x; // in meters
        }
    }

    if ((abs(g_hopper_x - g_center_image_x) < DIFF_HOPPER_X_THRESH) && (abs(g_furnace_x - g_center_image_x) < DIFF_FURNACE_X_THRESH)) //check to see if hopper is close to center and if furnace is detected
    {
        if(g_hopper_height >= g_target_height) //if hopper is too close, stop the robot
        {
            ROS_INFO_STREAM("Stopping Hauler");
            g_times_reached++;

            if(g_times_reached == TIMES_REACHED_THRESHOLD)
            {
                g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
                g_nav_goal.forward_velocity = 0;
                g_nav_goal.angular_velocity = 0;
                g_parked = true;
            }
        }
        else //otherwise drive towards hopper
        {
            ROS_INFO_STREAM("Driving To Hopper");
            g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
            g_nav_goal.forward_velocity = FORWARD_VELOCITY;
            g_nav_goal.angular_velocity = 0;
        }
    }
    else
    {
        double radius = g_processing_plant_z + PROCESSING_PLANT_RADIUS; //set radius of orbit, 1.8 is the depth of the object 
        geometry_msgs::PointStamped pt; 
        pt.point.x = radius; 
        pt.header.frame_id = robot_name + COMMON_NAMES::ROBOT_CHASSIS;
        g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::REVOLVE; //all nav goals will be using the revolve drive
        g_nav_goal.point = pt;
        
        if (g_hopper_x > HOPPER_ORBIT_THRESH) // if the hopper is on the right side of the screen, orbit right 
        {
            g_nav_goal.forward_velocity = FORWARD_VELOCITY;
        }
        else //otherwise orbit left 
        {
            g_nav_goal.forward_velocity = -FORWARD_VELOCITY;
        }   
    }
}

/**
 * @brief Compute properties such as size of bounding box, area etc.
 * 
 * @param object Perception object message
 * @param found boolean value if the object is found or not
 * @param size_x width of bounding box
 * @param size_y height of bounding box
 * @param area area of bounding box
 * @param center center of bounding box x
 */
void computeProperties(perception::Object& object, bool& found, float& size_x, float& size_y, float& center_x)
{
    found = true;
    size_x = object.size_x;
    size_y = object.size_y;
    center_x = object.center.x;
}

/**
 * @brief find the excavator using navigation vision
 * 
 */
void findExcavator()
{
    VisionClient client(robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
    client.waitForServer();
    operations::NavigationVisionGoal goal; 
    // desired target object, any object detection class
    goal.desired_object_label = COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS; 
    goal.mode = COMMON_NAMES::NAV_VISION_TYPE::V_REACH;
    client.sendGoal(goal);    
    client.waitForResult();    
    return;  
}

/**
 * @brief Copies the global hauler's and excavator's object into local objects and frees the lock immediately
 * 
 * @param hauler_objects 
 * @param exc_objects 
 */
void getObjects(perception::ObjectArray& hauler_objects, perception::ObjectArray& exc_objects)
{
    const std::lock_guard<std::mutex> lock_hauler(g_hauler_objects_mutex);
    const std::lock_guard<std::mutex> lock_excavator(g_excavator_objects_mutex);   

    hauler_objects = g_hauler_objects;
    exc_objects = g_excavator_objects;
}

/**
 * @brief Function to park robot wrt excavator
 * 
 * Steps:
 * 1. Rotate around excavator assuming robot is already near the excavator and has excavator in the center of the image
 * 2. Stop rotation when you reach desired orientation i.e. excavator arm on right of robot antenna and having maximum distance between their center
 *    This is done assuming that when camera looks exactly in between robot antenna and excavator arm, their difference between centers is maximum, thats our interest point
 * 3. Drive forward until you reach a required distance from robot antenna
 */
void parkWrtExcavator()
{

    perception::ObjectArray hauler_objects;
    perception::ObjectArray exc_objects;

    getObjects(hauler_objects, exc_objects);

    float center_exc = INIT_VALUE, height_exc = INIT_VALUE, z_exc = INIT_VALUE;

    static bool prev_centered = false;
        
    //parsing the objects message to extract the necessary information of required object
    for(int i = 0; i < hauler_objects.number_of_objects; i++) 
    {   
        perception::Object object = hauler_objects.obj.at(i);
        if(object.label == COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS) 
        {
            center_exc = object.center.x;
            height_exc = object.size_y;
            z_exc = object.point.pose.position.z;
        }
    }

    //parsing the objects message to extract the necessary information of required object
    for(int i = 0; i < exc_objects.number_of_objects; i++) 
    {   
        perception::Object object = exc_objects.obj.at(i);
        bool is_hauler = (object.label == COMMON_NAMES::OBJECT_DETECTION_HAULER_CLASS), is_scout = (object.label == COMMON_NAMES::OBJECT_DETECTION_SCOUT_CLASS), is_big_enough = (object.size_y > 180);
        if((is_hauler || is_scout) && is_big_enough) 
        {
            float center_img = (WIDTH_IMAGE / 2.0);
            float error_angle = center_img - object.center.x;   

            if(abs(error_angle) < 20)
            {
                ROS_INFO("HAULER's ORIENTATION CORRECT");
                g_found_orientation = true;
            }         
        }
    }

    if(g_found_orientation)
    {
        // Stopping and exiting once correct orientation wrt excavator is found and robot antenna's distance is less than threshold
        g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
        g_nav_goal.forward_velocity = 0;
        g_nav_goal.angular_velocity = 0;
        g_parked = true;
        return;
    }

    g_times_excavator = (center_exc == INIT_VALUE) ? g_times_excavator + 1 : 0;
    float center_img = (WIDTH_IMAGE / 2.0);
    float error_angle = center_img - center_exc;

    if(!g_found_orientation && height_exc > 0 && height_exc > 300)
    {
        g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
        g_nav_goal.forward_velocity = -0.2;
        g_nav_goal.angular_velocity = 0;
        return;
    }
    
    if((!g_found_orientation && center_exc != INIT_VALUE && abs(error_angle) > 100) || g_times_excavator > 10)
    {
        findExcavator();
    }

    // rotate robot around excavator
    geometry_msgs::PointStamped pt;
    pt.point.x = (center_exc > 0) ? ROBOT_RADIUS + z_exc : DEFAULT_RADIUS;
    pt.header.frame_id = robot_name + COMMON_NAMES::ROBOT_CHASSIS;
    g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::REVOLVE;
    g_nav_goal.point = pt;
    g_nav_goal.forward_velocity = 0.3;
}

/**
 * @brief Function which gets executed when any goal is received to actionlib
 * 
 * @param goal for action lib
 * @param as variable to send feedback
 */
void execute(const operations::ParkRobotGoalConstPtr& goal, Server* as)
{
    // initializing global variables
    g_execute_called = true; 
    g_parked = false;
    ros::Rate update_rate(UPDATE_HZ);
    bool park_mode = OBJECT_PARKER::EXCAVATOR;
    
    ROS_INFO("Got the parking goal");

    ros::NodeHandle nh;
    ros::Subscriber excavator_objects_sub;

    if(goal->hopper_or_excavator == COMMON_NAMES::OBJECT_DETECTION_HOPPER_CLASS)
    {
        // check the mode, park with hopper
        // Assumes the robot has reached near processing plant 
        ROS_INFO("Parking To Hopper");

        // initialize all the necessary variables
        park_mode = OBJECT_PARKER::HOPPER;
        g_times_reached = 0;
        g_hopper_x = -1;
        g_processing_plant_z = -1;
        g_furnace_x = -1;
        g_hopper_height = -1;
        g_center_image_x = 320;
        g_target_height = 385;
        double g_hopper_z;
    }
    else
    {
        // park with excavator
        // assumes robot has reached near excavator
        ROS_INFO("Parking To Excavator");      
        std::string excavator_name = COMMON_NAMES::EXCAVATOR_1;
        if(goal->hopper_or_excavator == COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS)
        {
            ROS_INFO_STREAM("USING AS "<<COMMON_NAMES::EXCAVATOR_1<<" EXCAVATOR DEFAULT");
        }
        else
        {
            excavator_name = goal->hopper_or_excavator;
        }

        excavator_objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + excavator_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &excavatorObjectsCallback);
        // initialize all the necessary variables
        g_times_excavator = 0;
        g_found_orientation = false;
        g_max_diff = -1;
    }

    while (ros::ok() && !g_parked)
    {    
        ros::spinOnce();
        if(park_mode == OBJECT_PARKER::HOPPER)
            parkWrtHopper();
        else
            parkWrtExcavator();

        g_client->sendGoal(g_nav_goal);    
        update_rate.sleep();
    }

    ROS_INFO("Parked Robot");
    
    as->setSucceeded();
    g_execute_called = false;

    g_hauler_objects = perception::ObjectArray();
    g_excavator_objects = perception::ObjectArray();
}

int main(int argc, char *argv[])
{
    if(argc < 2)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname and target passed as a command line argument!");
        return -1;
    }

    //take in robot name as arg1, usually small_hauler_1
    robot_name = argv[1];

    //initialize node and node handler
    ros::init(argc, argv, robot_name + COMMON_NAMES::PARK_HAULER_HOPPER_SERVER_NODE_NAME);
    ros::NodeHandle nh;
    
    //subscriber for object detection
    ros::Subscriber hauler_objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &haulerObjectsCallback);
    
    g_client = new Client(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);

    Server server(nh, robot_name + COMMON_NAMES::PARK_HAULER_ACTIONLIB, boost::bind(&execute, _1, &server), false);
    server.start();
    ROS_INFO("Starting Park Hauler Server");
    ros::spin();
    
    ROS_INFO("Exiting");    
    return 0;
}