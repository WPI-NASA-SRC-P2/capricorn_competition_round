/*
Author BY: Mahimana Bhatt, Anirban Mukherjee, Kishor Sabarish G
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

This is an actionlib server for parking hauler with hopper or excavator

Command Line Arguments Required:
1. g_robot_name: eg. small_scout_1, small_excavator_2
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

#define UPDATE_HZ 8

typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> VisionClient;
typedef actionlib::SimpleActionServer<operations::ParkRobotAction> Server;
typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;

enum OBJECT_PARKER
{
    HOPPER = true,
    EXCAVATOR = false
};

Client *g_nav_client;
VisionClient *g_navigation_vision_client;
operations::NavigationGoal g_nav_goal;

const float HOPPER_FORWARD_VELOCITY = 0.3, EXC_FORWARD_VELOCITY = 0.2;
std::mutex g_hauler_objects_mutex, g_excavator_objects_mutex, g_cancel_goal_mutex;

bool g_hauler_message_received = false, g_excavator_message_received = false;

// global variables for park excavator
const int HAULER_HEIGHT_THRESH = 130, ANGLE_THRESHOLD_NARROW = 20, ANGLE_THRESH_WIDE = 100, EXCAVATOR_TIMES_DETECT_TIMES = 10, EXCAVATOR_HEIGHT_THRESH = 300;
const float DEFAULT_RADIUS = 5, ROBOT_RADIUS = 1, WIDTH_IMAGE = 640.0, ROBOT_ANTENNA_DEPTH_THRESH = 2.0;
bool g_parked = false, g_found_orientation = false, g_cancel_called = false, g_revolve_direction_set = false;
float g_revolve_direction = EXC_FORWARD_VELOCITY;
float HAULER_EXCAVATOR_PARKING_FORWARD_TIME = 25, HAULER_EXCAVATOR_PARKING_BACKWARD_TIME = 4;
int ANTENNA_AVERAGE_FOR_FRAMES = 5;
int g_times_excavator = 0;

// global variables for park hopper
const int TIMES_REACHED_THRESHOLD = 10, DIFF_HOPPER_X_THRESH = 50, DIFF_FURNACE_X_THRESH = 500, HOPPER_ORBIT_THRESH = 320;
const float PROCESSING_PLANT_RADIUS = 1.8, INIT_VALUE = -100.00;
int g_times_reached = 0, g_center_image_x = 320, g_target_height = 385;

bool g_execute_called = false, g_failed = false;
perception::ObjectArray g_hauler_objects, g_excavator_objects;
std::string g_robot_name;

/**
 * @brief Returns the string to be logged or printed
 * 
 */
std::string getString(std::string message)
{
    return "[OPERATIONS | PARK HAULER SERVER | " + g_robot_name +  "]: " + message;
}

/**
 * @brief Callback function which subscriber to Objects message published from hauler's object detection
 * 
 * @param objs 
 */
void haulerObjectsCallback(const perception::ObjectArray &objs)
{
    const std::lock_guard<std::mutex> lock(g_hauler_objects_mutex);
    g_hauler_message_received = true;
    g_hauler_objects = objs;
}

/**
 * @brief Callback function which subscriber to Objects message published from excavator's object detection
 * 
 * @param objs 
 */
void excavatorObjectsCallback(const perception::ObjectArray &objs)
{
    const std::lock_guard<std::mutex> lock(g_excavator_objects_mutex);
    g_excavator_message_received = true;
    g_excavator_objects = objs;
}

/**
 * @brief find the excavator using navigation vision
 * 
 */
void findProcessingPlant()
{
    g_navigation_vision_client->waitForServer();
    operations::NavigationVisionGoal goal;
    // desired target object, any object detection class
    goal.desired_object_label = COMMON_NAMES::OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
    goal.mode = COMMON_NAMES::NAV_VISION_TYPE::V_REACH;
    g_navigation_vision_client->sendGoal(goal);
    g_navigation_vision_client->waitForResult();
    return;
}

/**
 * @brief find the excavator using navigation vision
 * 
 */
void centerFurnace()
{
    g_navigation_vision_client->waitForServer();
    operations::NavigationVisionGoal goal;
    // desired target object, any object detection class
    goal.desired_object_label = COMMON_NAMES::OBJECT_DETECTION_FURNACE_CLASS;
    goal.mode = COMMON_NAMES::NAV_VISION_TYPE::V_CENTER;
    g_navigation_vision_client->sendGoal(goal);
    g_navigation_vision_client->waitForResult();
    return;
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
    float processing_plant_z = INIT_VALUE, hopper_x = INIT_VALUE, hopper_z = INIT_VALUE, hopper_height = INIT_VALUE, furnace_z = INIT_VALUE, furnace_x = INIT_VALUE, furnace_size_x = INIT_VALUE;
    float furnace_center_y = INIT_VALUE, processing_plant_x = INIT_VALUE;
    static bool centering = true;

    for (int i = 0; i < n; i++)
    {
        perception::Object object = objects.obj.at(i);

        if (object.label == COMMON_NAMES::OBJECT_DETECTION_HOPPER_CLASS)
        {
            //If hopper is detected store hopper_x and hopper_z
            hopper_x = object.center.x; // in pixels
            hopper_z = object.point.pose.position.z;
            hopper_height = object.size_y;
        }

        else if (object.label == COMMON_NAMES::OBJECT_DETECTION_PROCESSING_PLANT_CLASS)
        {
            //If processingPlant is detected store processing_plant_z
            processing_plant_z = object.point.pose.position.z; // in meters
            processing_plant_x = object.center.x;
        }
        else if (object.label == COMMON_NAMES::OBJECT_DETECTION_FURNACE_CLASS)
        {
            //If furnace is detected store furnace_z
            furnace_x = object.center.x;
            furnace_z = object.point.pose.position.z; // in meters
            furnace_size_x = object.size_x;           // in meters
            furnace_center_y = object.center.y;
        }
    }

    bool processing_plant_detected = (processing_plant_z != INIT_VALUE), furnace_detected = (furnace_z != INIT_VALUE), hopper_detected = (hopper_x != INIT_VALUE);

    if (furnace_detected && furnace_z < 3)
    {
        // ROS_INFO_STREAM(getString("REACHED HOPPER"));
        for (int i = 0; i < 20; i++)
        {
            g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
            g_nav_goal.forward_velocity = HOPPER_FORWARD_VELOCITY;
            g_nav_client->sendGoal(g_nav_goal);
            ros::Duration(0.2).sleep();
        }
        g_nav_goal.forward_velocity = 0;
        g_parked = true;
        centering = true;
        return;
    }

    if (hopper_detected && furnace_detected && furnace_size_x > 70 && abs(hopper_x - furnace_x) < 30)
    {
        if (centering && furnace_x > 90)
        {
            centerFurnace();
            centering = false;
        }
        g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
        g_nav_goal.forward_velocity = HOPPER_FORWARD_VELOCITY;
        return;
    }

    double radius = -1;
    centering = true;

    if (processing_plant_detected && hopper_detected)
        g_nav_goal.forward_velocity = std::copysign(HOPPER_FORWARD_VELOCITY, hopper_x - processing_plant_x);

    if (furnace_detected)
    {
        if (abs(furnace_x - g_center_image_x) > 80 && furnace_size_x > 100)
            centerFurnace();

        if (hopper_detected)
        {
            g_nav_goal.forward_velocity = std::copysign(HOPPER_FORWARD_VELOCITY, hopper_x - furnace_x);
        }
        radius = furnace_z + PROCESSING_PLANT_RADIUS; //set radius of orbit, 1.8 is the depth of the object
    }
    else if (processing_plant_detected)
    {
        radius = processing_plant_z + PROCESSING_PLANT_RADIUS; //set radius of orbit, 1.8 is the depth of the object
    }

    if (processing_plant_z > 10)
        findProcessingPlant();

    if (radius > -1)
    {
        geometry_msgs::PointStamped pt;
        pt.point.x = radius;
        pt.header.frame_id = g_robot_name + COMMON_NAMES::ROBOT_CHASSIS;
        g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::REVOLVE; //all nav goals will be using the revolve drive
        g_nav_goal.point = pt;
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
void computeProperties(perception::Object &object, bool &found, float &size_x, float &size_y, float &center_x)
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
bool findExcavator()
{
    g_navigation_vision_client->waitForServer();
    operations::NavigationVisionGoal goal;
    // desired target object, any object detection class
    goal.desired_object_label = COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS;
    goal.mode = COMMON_NAMES::NAV_VISION_TYPE::V_REACH;
    g_navigation_vision_client->sendGoal(goal);
    g_navigation_vision_client->waitForResult();
    if(g_navigation_vision_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED && g_navigation_vision_client->getResult()->result == COMMON_NAMES::COMMON_RESULT::FAILED)
    {
        return false;
    }
    return true;
}

/**
 * @brief Copies the global hauler's and excavator's object into local objects and frees the lock immediately
 * 
 * @param hauler_objects 
 * @param exc_objects 
 */
void getObjects(perception::ObjectArray &hauler_objects, perception::ObjectArray &exc_objects)
{
    const std::lock_guard<std::mutex> lock_hauler(g_hauler_objects_mutex);
    const std::lock_guard<std::mutex> lock_excavator(g_excavator_objects_mutex);

    hauler_objects = g_hauler_objects;
    exc_objects = g_excavator_objects;
}

/**
 * @brief Sets the revolve direction for parking of excavator to greedily reach the required orientation quickly
 * 
 * @param hauler_center : hauler bounding box's center (in pixels)
 * @param exc_center : excavator bounding box's center (in pixels)
 * @param exc_arm_center : excavator arm bounding box's center (in pixels)
 */
void setRevolveDirection(float hauler_center, float exc_center, float exc_arm_center)
{
    if (hauler_center > -1)
    {
        // if hauler is detection, use the hauler location only to decide the revolve direction
        float center_img = (WIDTH_IMAGE / 2.0);
        g_revolve_direction = (hauler_center > center_img) ? EXC_FORWARD_VELOCITY : -EXC_FORWARD_VELOCITY;
        g_revolve_direction_set = true;
    }
    else if (!g_revolve_direction_set)
    {
        // this would only run once when the hauler reaches the excavator, not to change the revolve direction when excavator arm's direction changes wrt to excavator
        if (exc_center > -1 && exc_arm_center > -1)
        {
            g_revolve_direction = (exc_arm_center > exc_center) ? EXC_FORWARD_VELOCITY : -EXC_FORWARD_VELOCITY;
            g_revolve_direction_set = true;
        }
    }
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
void parkWrtExcavator(std::vector<float>& last_n_depths_for_antenna)
{
    perception::ObjectArray hauler_objects;
    perception::ObjectArray exc_objects;

    getObjects(hauler_objects, exc_objects);

    float center_exc = INIT_VALUE, height_exc = INIT_VALUE, z_exc = INIT_VALUE, depth_hauler_ra = INIT_VALUE, center_exc_arm = INIT_VALUE, center_hauler = INIT_VALUE;

    static bool prev_centered = false;

    //parsing the objects message to extract the necessary information of required hauler's objects
    for (int i = 0; i < hauler_objects.number_of_objects; i++)
    {
        perception::Object object = hauler_objects.obj.at(i);
        if (object.label == COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS)
        {
            center_exc = object.center.x;
            height_exc = object.size_y;
            z_exc = object.point.pose.position.z;
        }
        else if (object.label == COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_ARM_CLASS)
        {
            center_exc_arm = object.center.x;
        }
    }

    //parsing the objects message to extract the necessary information of required excavator's objects
    for (int i = 0; i < exc_objects.number_of_objects; i++)
    {
        perception::Object object = exc_objects.obj.at(i);
        bool is_hauler = (object.label == COMMON_NAMES::OBJECT_DETECTION_HAULER_CLASS), is_scout = (object.label == COMMON_NAMES::OBJECT_DETECTION_SCOUT_CLASS), is_hauler_big_enough = (object.size_y > HAULER_HEIGHT_THRESH);
        if ((is_hauler || is_scout) && is_hauler_big_enough)
        {
            center_hauler = object.center.x;
            float center_img = (WIDTH_IMAGE / 2.0);
            float error_angle = center_img - center_hauler;

            if (error_angle > 0 && error_angle < ANGLE_THRESHOLD_NARROW)
            {
                // ROS_INFO_STREAM(getString("HAULER's ORIENTATION CORRECT"));
                g_found_orientation = true;
            }
        }
        // ROS_INFO_STREAM("Antenna Finding");
        bool is_robot_antenna = (object.label == COMMON_NAMES::OBJECT_DETECTION_ROBOT_ANTENNA_CLASS);
        if (is_robot_antenna)
        {
            last_n_depths_for_antenna.push_back(object.point.pose.position.z);
            if(last_n_depths_for_antenna.size()>ANTENNA_AVERAGE_FOR_FRAMES)
                last_n_depths_for_antenna.erase(last_n_depths_for_antenna.begin());
            float sum_of_datas = 0.0;
            for(int i = 0; i<last_n_depths_for_antenna.size(); i++)
                sum_of_datas += last_n_depths_for_antenna.at(i);
                
            depth_hauler_ra = sum_of_datas/last_n_depths_for_antenna.size();
            // ROS_INFO_STREAM("Antenna depth calculated: "<<depth_hauler_ra);
        }
    }

    setRevolveDirection(center_hauler, center_exc, center_exc_arm);

    if (g_found_orientation)
    {

        // if the required orientation is found, drive forward till hauler's robot antenna's height (in pixels) exceeds a minimum threshold
        static int times_depth_crossed = 0, g_lost = 0;
        static float last_depth = INIT_VALUE;

        g_lost = (depth_hauler_ra == INIT_VALUE) ? g_lost + 1 : 0;
        // ROS_INFO("##### Using the new value #####");
        if (depth_hauler_ra < ROBOT_ANTENNA_DEPTH_THRESH && depth_hauler_ra != INIT_VALUE)
        {
            if (last_depth != depth_hauler_ra)
            {
                times_depth_crossed++;
            }
        }
        else
        {
            times_depth_crossed = 0;
        }

        if (times_depth_crossed > 3)
        {
            g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
            g_nav_goal.forward_velocity = 0;
            g_nav_goal.angular_velocity = 0;
            times_depth_crossed = 0;
            last_depth = INIT_VALUE;
            g_parked = true;
            return;
        }
        else if(g_lost > 5)
        {
            g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
            g_nav_goal.forward_velocity = 0.15;
            g_nav_goal.direction = 0;
            g_nav_goal.angular_velocity = 0;
            
            g_nav_client->sendGoal(g_nav_goal);
            g_nav_client->sendGoal(g_nav_goal);

            ROS_WARN("New Attacking the Excavator");
            ros::Duration(HAULER_EXCAVATOR_PARKING_FORWARD_TIME).sleep();

            g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
            g_nav_goal.forward_velocity = -0.15;
            g_nav_goal.direction = 0;
            g_nav_goal.angular_velocity = 0;
            
            g_nav_client->sendGoal(g_nav_goal);
            g_nav_client->sendGoal(g_nav_goal);

            ROS_WARN("New Backing off from the Excavator");
            ros::Duration(HAULER_EXCAVATOR_PARKING_BACKWARD_TIME).sleep();


            g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
            g_nav_goal.forward_velocity = 0;
            g_nav_goal.direction = 0;
            g_nav_goal.angular_velocity = 0;
            
            g_nav_client->sendGoal(g_nav_goal);
            g_nav_client->sendGoal(g_nav_goal);

            g_parked = true;
            return;
        }

        last_depth = depth_hauler_ra;

        float center_img = (WIDTH_IMAGE / 2.0);
        float error_angle = center_img - center_exc;
        if (center_exc != INIT_VALUE && abs(error_angle) > 10 && depth_hauler_ra > 3)
            findExcavator();

        g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
        g_nav_goal.forward_velocity = 0.1;
        g_nav_goal.angular_velocity = 0;
        // Stopping and exiting once correct orientation wrt excavator is found and robot antenna's distance is less than threshold
        return;
    }

    g_times_excavator = (center_exc == INIT_VALUE) ? g_times_excavator + 1 : 0;
    float center_img = (WIDTH_IMAGE / 2.0);
    float error_angle = center_img - center_exc;

    if (!g_found_orientation && height_exc > 0 && height_exc > EXCAVATOR_HEIGHT_THRESH)
    {
        // if the excavator is bigger than required (when revolving), means hauler is very close to excavator, drive backward
        g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
        g_nav_goal.forward_velocity = -EXC_FORWARD_VELOCITY;
        g_nav_goal.angular_velocity = 0;
        return;
    }

    if ((!g_found_orientation && center_exc != INIT_VALUE && abs(error_angle) > ANGLE_THRESH_WIDE) || g_times_excavator > EXCAVATOR_TIMES_DETECT_TIMES)
    {
        // if excavator is lost or excavator is not in the center of the image, call the navigation vision server to center or find the excavator
        if(!findExcavator())
        {
            g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
            g_nav_goal.forward_velocity = 0;
            g_nav_goal.direction = 0;
            g_nav_goal.angular_velocity = 0;
            
            g_nav_client->sendGoal(g_nav_goal);
            g_nav_client->sendGoal(g_nav_goal);
            g_failed = true; 
            return;
        }
    }

    // rotate robot around excavator
    geometry_msgs::PointStamped pt;
    pt.point.x = (center_exc > 0) ? ROBOT_RADIUS + z_exc : DEFAULT_RADIUS;
    pt.header.frame_id = g_robot_name + COMMON_NAMES::ROBOT_CHASSIS;
    g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::REVOLVE;
    g_nav_goal.point = pt;
    g_nav_goal.forward_velocity = g_revolve_direction;
}

/**
 * @brief Function called when the goal is cancelled
 * 
 */
void cancelGoal()
{
    const std::lock_guard<std::mutex> lock(g_cancel_goal_mutex);
    g_cancel_called = true;

    bool navigation_vision_pending = (g_navigation_vision_client->getState() == actionlib::SimpleClientGoalState::PENDING);
    bool navigation_vision_active = (g_navigation_vision_client->getState() == actionlib::SimpleClientGoalState::ACTIVE);
    if (navigation_vision_active || navigation_vision_pending)
    {
        g_navigation_vision_client->cancelGoal();
    }

    g_nav_goal.forward_velocity = 0;
    g_nav_goal.angular_velocity = 0;
    g_nav_client->sendGoal(g_nav_goal);

    // ROS_INFO_STREAM(getString("Park Hauler : Cancelled Goal"));
}

/**
 * @brief Function which gets executed when any goal is received to actionlib
 * 
 * @param goal for action lib
 * @param as variable to send feedback
 */
void execute(const operations::ParkRobotGoalConstPtr &goal, Server *as)
{
    operations::ParkRobotResult result;

    // initializing global variables
    g_execute_called = true;
    g_cancel_called = false;
    g_failed = false;
    g_parked = false;
    ros::Rate update_rate(UPDATE_HZ);
    bool park_mode = OBJECT_PARKER::EXCAVATOR;

    // ROS_INFO_STREAM(getString("Got the parking goal"));

    ros::NodeHandle nh;
    ros::Subscriber excavator_objects_sub;

    if (goal->hopper_or_excavator == COMMON_NAMES::OBJECT_DETECTION_HOPPER_CLASS)
    {
        // check the mode, park with hopper
        // Assumes the robot has reached near processing plant
        g_nav_goal.forward_velocity = HOPPER_FORWARD_VELOCITY;
        // ROS_INFO_STREAM(getString("Parking To Hopper"));
        findProcessingPlant();
        // initialize all the necessary variables
        park_mode = OBJECT_PARKER::HOPPER;
        g_times_reached = 0;
        g_center_image_x = 320;
        g_target_height = 385;
        double hopper_z;
    }
    else
    {
        // park with excavator
        // ROS_INFO_STREAM(getString("Parking To Excavator"));
        std::string excavator_name = COMMON_NAMES::EXCAVATOR_1_NAME;

        if (goal->hopper_or_excavator == COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS)
        {
            // uses "small_excavator_1" by default
            // ROS_INFO_STREAM(getString("USING AS ") << COMMON_NAMES::EXCAVATOR_1_NAME << " EXCAVATOR DEFAULT");
        }
        else
        {
            // ROS_INFO_STREAM(getString("USING AS ") << goal->hopper_or_excavator << " FOR EXCAVATOR PARKING");
            // use given excavator name in the actionlib goal, this excavator name is used for subscribing to it's object detection topic
            excavator_name = goal->hopper_or_excavator;
        }

        excavator_objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + excavator_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &excavatorObjectsCallback);

        // initialize all the necessary variables
        g_times_excavator = 0;
        g_found_orientation = false;
        g_revolve_direction_set = false;
    }

    std::vector<float> last_n_depths_for_antenna;

    while (ros::ok() && !g_parked && !g_cancel_called && !g_failed)
    {
        ros::spinOnce();
        if (park_mode == OBJECT_PARKER::HOPPER && g_hauler_message_received)
            parkWrtHopper();
        else if (g_hauler_message_received && g_excavator_message_received)
            parkWrtExcavator(last_n_depths_for_antenna);

        g_nav_client->sendGoal(g_nav_goal);
        update_rate.sleep();
        const std::lock_guard<std::mutex> lock(g_cancel_goal_mutex);
    }

    if (g_cancel_called)
    {
        g_cancel_called = false;
        result.result = COMMON_NAMES::COMMON_RESULT::INTERRUPTED;
        as->setSucceeded(result, "Cancelled Goal");
        return;
    }

    else if (g_failed)
    {
        // ROS_INFO_STREAM(getString("Parking Failed"));
        g_failed = false;
        result.result = COMMON_NAMES::COMMON_RESULT::FAILED;
        as->setSucceeded(result, "Failed Goal");
        return;
    }

    // ROS_INFO_STREAM(getString("Parked Robot"));

    result.result = COMMON_NAMES::COMMON_RESULT::SUCCESS;
    as->setSucceeded(result);
    g_execute_called = false;

    g_hauler_objects = perception::ObjectArray();
    g_excavator_objects = perception::ObjectArray();
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname and target passed as a command line argument!");
        return -1;
    }

    //take in robot name as arg1, usually small_hauler_1
    g_robot_name = argv[1];

    //initialize node and node handler
    ros::init(argc, argv, g_robot_name + COMMON_NAMES::PARK_HAULER_HOPPER_SERVER_NODE_NAME);
    ros::NodeHandle nh;
    // ROS_INFO_STREAM(getString("Starting Park Hauler Server"));
    //subscriber for object detection
    ros::Subscriber hauler_objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + g_robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &haulerObjectsCallback);

    g_nav_client = new Client(COMMON_NAMES::CAPRICORN_TOPIC + g_robot_name + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);
    g_navigation_vision_client = new VisionClient(g_robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);

    Server server(nh, g_robot_name + COMMON_NAMES::PARK_HAULER_ACTIONLIB, boost::bind(&execute, _1, &server), false);
    server.registerPreemptCallback(&cancelGoal);
    server.start();
    // ROS_INFO_STREAM(getString("Starting Park Hauler Server"));
    ros::spin();

    // ROS_INFO_STREAM(getString("Exiting"));

    delete g_nav_client;
    delete g_navigation_vision_client;
    
    return 0;
}