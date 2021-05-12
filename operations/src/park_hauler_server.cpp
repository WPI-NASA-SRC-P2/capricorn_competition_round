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
std::mutex g_objects_mutex;

// global variables for park excavator
const int MIN_DIFF_THRESH = 150, DIFF_CHANGE_THRESH = 20, ROBOT_ANTENNA_HEIGHT_THRESH = 120, ANGLE_THRESHOLD_NARROW = 20, ANGLE_THRESHOLD_WIDE = 80;
const float DEFAULT_RADIUS = 3.5, WIDTH_IMAGE = 640.0;
bool g_parked = false, g_found_orientation = false;
float g_max_diff = -1;

// global variables for park hopper
const int TIMES_REACHED_THRESHOLD = 10, DIFF_HOPPER_X_THRESH = 50, DIFF_FURNACE_X_THRESH = 500, HOPPER_ORBIT_THRESH = 320;
const float PROCESSING_PLANT_RADIUS = 1.8, INIT_VALUE = -100.00;
int g_hopper_x, g_processing_plant_z, g_furnace_x, g_hopper_height, g_times_reached = 0, g_center_image_x = 320, g_target_height = 385;
double g_hopper_z;

bool g_execute_called = false;
perception::ObjectArray g_objects;
std::string robot_name;

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
    const std::lock_guard<std::mutex> lock(g_objects_mutex);    
    perception::ObjectArray objects = g_objects;

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
    const std::lock_guard<std::mutex> lock(g_objects_mutex);
    perception::ObjectArray objects = g_objects;

    // bool if excavator arm and robot antenna are found previously
    bool found_ea = false, found_ra = false;
    // float values for center_x (in pixels), size_x (width of bounding box in pixels), size_y (height of bounding box in pixels) of robot antenna and excavator arm
    // distance from robot antenna (z_ra in m)
    float center_x_ea = INIT_VALUE, center_x_ra = INIT_VALUE, size_x_ea = INIT_VALUE, size_y_ea = INIT_VALUE, size_x_ra = INIT_VALUE, size_y_ra= INIT_VALUE;
    float center_exc = INIT_VALUE, height_exc = INIT_VALUE;

    static bool prev_centered = false;
        
    //parsing the objects message to extract the necessary information of required object
    for(int i = 0; i < objects.number_of_objects; i++) 
    {   
        perception::Object object = objects.obj.at(i);
        if(object.label == COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS) 
        {
            center_exc = object.center.x;
            height_exc = object.size_y;
        }
        else if(object.label == COMMON_NAMES::OBJECT_DETECTION_ROBOT_ANTENNA_CLASS) 
        {
            //found robot antenna object
            if(found_ra)
            {
                // if the robot antenna is already found, false detection, abort further calculation
                return;
            }            
            computeProperties(object, found_ra, size_x_ra, size_y_ra, center_x_ra);
        }
        else if(object.label == COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_ARM_CLASS) 
        {
            //found excavator arm object
            if(found_ea)
            {
                // if the excavator arm is already found, false detection, abort further calculation
                return;
            }
            computeProperties(object, found_ea, size_x_ea, size_y_ea, center_x_ea);
        }
    }

    if(!g_found_orientation && center_exc != INIT_VALUE)
    {
        float center_img = (WIDTH_IMAGE / 2.0);
        float error_angle = center_img - center_exc;
        if(abs(error_angle) > 80)
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
    }

    if(g_found_orientation)
    {
        if(found_ra && size_y_ra < ROBOT_ANTENNA_HEIGHT_THRESH)
        {
            g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
            g_nav_goal.forward_velocity = FORWARD_VELOCITY;
            g_nav_goal.angular_velocity = 0;
            return;
        }
        else
        {
            // Stopping and exiting once correct orientation wrt excavator is found and robot antenna's distance is less than threshold
            g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
            g_nav_goal.forward_velocity = 0;
            g_nav_goal.angular_velocity = 0;
            g_parked = true;
            return;
        }
    }

    if(found_ea && found_ra)
    {
        // if both excavator and robot antenna are found
        float ratio = size_x_ea / size_y_ea;
        if(center_x_ea < center_x_ra)
        {
            // check if the excavator arm is in right of robot antenna
            // calculating difference between the center of robot antenna and excavator arm
            float diff = center_x_ra - center_x_ea;
            // keeping track if the current difference value is the maximum difference value
            // the maximum difference will be when the robot is looking exactly in center of robot antenna and excavator arm
            g_max_diff = std::max(g_max_diff, diff);

            if(diff > MIN_DIFF_THRESH && diff < g_max_diff - DIFF_CHANGE_THRESH)
            {
                // check if the current diff value start decreasing, this means we have reached necessary orientation
                ROS_INFO("Stop Revolution");
                g_found_orientation = true;
            }
        }
    }

    // rotate robot around excavator
    geometry_msgs::PointStamped pt;
    pt.point.x = DEFAULT_RADIUS;
    pt.header.frame_id = robot_name + COMMON_NAMES::ROBOT_CHASSIS;
    g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::REVOLVE;
    g_nav_goal.point = pt;
    g_nav_goal.forward_velocity = -0.3;
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

        // initialize all the necessary variables
        g_found_orientation = false;
        g_max_diff = -1;
    }

    while (ros::ok() && !g_parked)
    {    
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
    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objectsCallback);
    g_client = new Client(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);

    Server server(nh, robot_name + COMMON_NAMES::PARK_HAULER_ACTIONLIB, boost::bind(&execute, _1, &server), false);
    server.start();
    ROS_INFO("Starting Park Hauler Server");
    ros::spin();
    
    ROS_INFO("Exiting");    
    return 0;
}