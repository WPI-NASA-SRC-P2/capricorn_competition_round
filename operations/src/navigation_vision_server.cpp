/*
Author BY: Mahimana Bhatt, Chris DeMaio
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

This is an actionlib server for parking hauler with hopper or excavator

Command Line Arguments Required:
1. robot_name: eg. small_scout_1, small_excavator_2
*/
#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <operations/NavigationVisionAction.h>
#include <operations/obstacle_avoidance.h>

#define UPDATE_HZ 10

typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;
typedef actionlib::SimpleActionServer<operations::NavigationVisionAction> Server;

Client* g_client;

operations::NavigationGoal g_nav_goal;
perception::ObjectArray g_objects;

const int ANGLE_THRESHOLD_NARROW = 20, ANGLE_THRESHOLD_WIDE = 80, HEIGHT_IMAGE = 480, FOUND_FRAME_THRESHOLD = 3, LOST_FRAME_THRESHOLD = 5;
const float PROPORTIONAL_ANGLE = 0.0010, ANGULAR_VELOCITY = 0.35, INIT_VALUE = -100.00, FORWARD_VELOCITY = 0.8;
std::mutex g_objects_mutex, g_cancel_goal_mutex;
std::string g_desired_label;
bool g_centered = false, g_execute_called = false, g_cancel_called = false;
int g_height_threshold = 400, g_lost_detection_times = 0, g_true_detection_times = 0, g_revolve_direction = -1;

enum HEIGHT_THRESHOLD
{
    HOPPER = 250,
    EXCAVATOR = 240,
    OTHER = 400,
};

/**
 * @brief Set the Desired Label Bounding BoxHeight Threshold object
 * 
 */
void setDesiredLabelHeightThreshold()
{
    if(g_desired_label == COMMON_NAMES::OBJECT_DETECTION_HOPPER_CLASS)
    {
        g_height_threshold = HEIGHT_THRESHOLD::HOPPER;
    }
    if(g_desired_label == COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS)
    {
        g_height_threshold = HEIGHT_THRESHOLD::EXCAVATOR;
    }
    else
    {
        g_height_threshold = HEIGHT_THRESHOLD::OTHER;
    }    
}

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
 * Steps:
 * 1. Rotate robot util the desired object detection class has its bounding box in the center of the frame
 * 2. Drive forward until you reach the desired class bounding box's minimum height
 * 3. Avoid obstacle using object detection, if the obstacle is in right half of the image, it means that the obstacle will be in robot's path, so robot crab drives until
 *    there is no obstacle in right half of the image, an object will be considered an obstacle iff it is not target label and is greater than a height threshold (currently rocks are only considered as obstacles)
 * If the object is lost while the above process, the process will be started again * 
 * Two thresholds are used for centering the object in the image, narrow threshold for initial centering and wide threshold if the object was centered
 * but looses the center afterwards
 */
void visionNavigation()
{
    const std::lock_guard<std::mutex> lock(g_objects_mutex);   
    perception::ObjectArray objects = g_objects;
    // Initialize location and size variables
    float center_obj = INIT_VALUE, height_obj = INIT_VALUE;

    // Initialize error, P Control, and necessary thresholds 
    float error_angle = WIDTH_IMAGE, error_height = HEIGHT_IMAGE;

    static float prev_angular_velocity;
    static bool prev_centered;

    std::vector<perception::Object> obstacles;
    float err_obstacle = 0;

    // Find the desired object
    for(int i = 0; i < objects.number_of_objects; i++) 
    {   
        perception::Object object = objects.obj.at(i);
        if(object.label == g_desired_label) 
        {
            // Store the object's center and height
            center_obj = object.center.x;
            height_obj = object.size_y;
        }
        else
            obstacles.push_back(object);
    }

    if(center_obj < -1)
    {
        // object not detected, rotate on robot's axis to find the object
        g_lost_detection_times++;
        g_true_detection_times = 0;
        if(g_lost_detection_times > LOST_FRAME_THRESHOLD)
        {
            g_nav_goal.angular_velocity = g_revolve_direction * ANGULAR_VELOCITY;
            g_nav_goal.forward_velocity = 0;
        }
        return;
    }
    else
    {
        // get the direction of crab walk needed to avoid obstacle
        float direction = checkObstacle(obstacles);

        if(abs(direction) > 0.0)
        {
            // if there is an obstacle to avoid, crab walk
            g_nav_goal.forward_velocity = FORWARD_VELOCITY;
            g_nav_goal.direction = direction;
            g_nav_goal.angular_velocity = 0;
            ROS_INFO("Avoid Obstacle Mode");
            return;
        }

        g_nav_goal.direction = 0;

        g_lost_detection_times = 0;
        g_true_detection_times++;
        // object found, compute the error in angle i.e. the error between the center of image and center of bounding box
        float center_img = (WIDTH_IMAGE / 2.0) + err_obstacle;
        error_angle = center_img - center_obj;
        // compute error in height, desired height minus current height of bounding box
        error_height = g_height_threshold - height_obj;

        if(error_angle < 0)
        {
            g_revolve_direction = -1;
        }
        else
        {
            g_revolve_direction = 1;
        }
    
        if (abs(error_angle) > ANGLE_THRESHOLD_WIDE)
        {
            // if the bounding box is not in the center of the image
            g_centered = false;
        }
      
        if(g_centered || abs(error_angle) < ANGLE_THRESHOLD_NARROW)
        {
            // if the bounding box is in the center of image following the narrow angle
            g_centered = true;
            g_nav_goal.angular_velocity = 0;
            if(error_height <= 0 && g_true_detection_times > FOUND_FRAME_THRESHOLD)
            {
                // If the object is big enough, stop the robot
                g_nav_goal.forward_velocity = 0;
                g_execute_called = false;
                g_centered = false;
                ROS_INFO_STREAM("Reached Goal - "<<g_desired_label);
                return;
            }
            else
            {
                // Keep driving forward
                g_nav_goal.forward_velocity = FORWARD_VELOCITY;
            }
        }
        else
        {
            // proportional controller for calculating angular velocity needed to center the object in the image
            g_nav_goal.angular_velocity = error_angle * PROPORTIONAL_ANGLE;
            g_nav_goal.forward_velocity = 0;

            if(g_nav_goal.angular_velocity < prev_angular_velocity - 0.05)
            {
                g_nav_goal.angular_velocity = prev_angular_velocity - 0.05;
            }
            if(g_nav_goal.angular_velocity > prev_angular_velocity + 0.05)
            {
                g_nav_goal.angular_velocity = prev_angular_velocity + 0.05;
            }
        }
    }

    if(prev_centered && !g_centered)
    {
        g_nav_goal.angular_velocity = 0;
        g_nav_goal.forward_velocity = 0;
    }

    // maintaing previous values
    prev_angular_velocity = g_nav_goal.angular_velocity;
    prev_centered = g_centered;
}

/**
 * @brief Checks whether the desire target label is a valid object detection class or not
 * 
 * @return true - if the class is valid
 * @return false - if the class is invalid
 */
bool check_class()
{
    if(g_desired_label == COMMON_NAMES::OBJECT_DETECTION_PROCESSING_PLANT_CLASS ||
       g_desired_label ==  COMMON_NAMES::OBJECT_DETECTION_REPAIR_STATION_CLASS ||
       g_desired_label ==  COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS ||
       g_desired_label ==  COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_ARM_CLASS ||
       g_desired_label ==  COMMON_NAMES::OBJECT_DETECTION_SCOUT_CLASS ||
       g_desired_label ==  COMMON_NAMES::OBJECT_DETECTION_HAULER_CLASS ||
       g_desired_label ==  COMMON_NAMES::OBJECT_DETECTION_FURNACE_CLASS ||
       g_desired_label ==  COMMON_NAMES::OBJECT_DETECTION_HOPPER_CLASS ||
       g_desired_label ==  COMMON_NAMES::OBJECT_DETECTION_ROBOT_ANTENNA_CLASS ||
       g_desired_label ==  COMMON_NAMES::OBJECT_DETECTION_PP_SMALL_THRUSTER_CLASS ||
       g_desired_label ==  COMMON_NAMES::OBJECT_DETECTION_ROCK_CLASS)
        return true;
    return false;
}

/**
 * @brief Function which gets executed when any goal is received to actionlib
 * 
 * @param goal for action lib
 * @param as variable to send feedback
 */
void execute(const operations::NavigationVisionGoalConstPtr& goal, Server* as)
{
    operations::NavigationVisionResult result;
    g_execute_called = true;
    g_desired_label = goal->desired_object_label;
    ROS_INFO_STREAM("Goal Received - "<<g_desired_label);


    if(!check_class())
    {
        // the class is not valid, send the appropriate result
        result.result = COMMON_NAMES::NAV_VISION_RESULT::V_INVALID_CLASS;
        as->setAborted(result, "Wrong Object Detection Class Name");
        ROS_INFO("Invalid Class");
        return;
    }

    // Set the desired bounding box target height according to the desired target class
    setDesiredLabelHeightThreshold();
    ros::Rate update_rate(UPDATE_HZ);
    
    do
    {
        while (ros::ok() && g_execute_called && !g_cancel_called)
        {
            visionNavigation();
            update_rate.sleep();
            g_client->sendGoal(g_nav_goal);
            const std::lock_guard<std::mutex> lock(g_cancel_goal_mutex); 
        }

        if(g_cancel_called)
        {
            g_cancel_called = false;
            result.result = COMMON_NAMES::NAV_VISION_RESULT::V_INTERRUPTED;
            as->setSucceeded(result, "Cancelled Goal");
            return;
        }

        g_execute_called = true;
    }
    while(goal->mode == COMMON_NAMES::NAV_VISION_TYPE::V_FOLLOW);

    result.result = COMMON_NAMES::NAV_VISION_RESULT::V_SUCCESS;
    as->setSucceeded(result);
}

/**
 * @brief Function called when the goal is cancelled
 * 
 */
void cancelGoal()
{
    ROS_INFO("Cancelled Vision Goal");
    const std::lock_guard<std::mutex> lock(g_cancel_goal_mutex); 
    g_cancel_called = true;
    ROS_INFO("Done Cancelling");
    g_nav_goal.forward_velocity = 0;
    g_nav_goal.angular_velocity = 0;
    g_client->sendGoal(g_nav_goal);
}

int main(int argc, char** argv)
{
    if(argc != 2 && argc != 4)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");
        return -1;
    }

    std::string robot_name(argv[1]);
    ros::init(argc, argv, robot_name + COMMON_NAMES::NAVIGATION_VISION_SERVER_NODE_NAME);
    ros::NodeHandle nh;

    g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;
    g_client = new Client(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);

    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objectsCallback);

    Server server(nh, robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, boost::bind(&execute, _1, &server), false);
	server.registerPreemptCallback(&cancelGoal);
    server.start();
    ROS_INFO("Starting Navigation Vision Server");
    ros::spin();
    return 0;
}
