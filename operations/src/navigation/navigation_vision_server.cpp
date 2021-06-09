/**
 * @file navigation_vision_server.cpp
 * @author Mahimana Bhatt, Chris DeMaio
 * @brief This is an actionlib server for navigation any robot using vision
 * Command Line Arguments Required:
 * 1. robot_name: eg. small_scout_1, small_excavator_2
 * 
 * @version 0.1
 * @date 2021-05-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <operations/NavigationVisionAction.h>
#include <operations/obstacle_avoidance.h>
#include <operations/navigation_algorithm.h>
#include <nav_msgs/Odometry.h>

#define UPDATE_HZ 10

typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;
typedef actionlib::SimpleActionServer<operations::NavigationVisionAction> Server;

using namespace COMMON_NAMES;

Client *g_client;

operations::NavigationGoal g_nav_goal;
perception::ObjectArray g_objects;

std::string g_robot_name;
geometry_msgs::PoseStamped g_robot_pose;

const int ANGLE_THRESHOLD_NARROW = 10, ANGLE_THRESHOLD_WIDE = 80, HEIGHT_IMAGE = 480, FOUND_FRAME_THRESHOLD = 3, LOST_FRAME_THRESHOLD = 5;
const float PROPORTIONAL_ANGLE = 0.0010, ANGULAR_VELOCITY = 0.35, INIT_VALUE = -100.00, FORWARD_VELOCITY = 0.8, g_angular_vel_step_size = 0.05;
const double NOT_AVOID_OBSTACLE_THRESHOLD = 5.0;
std::mutex g_objects_mutex, g_cancel_goal_mutex, g_odom_mutex;
std::string g_desired_label;
bool g_reached_goal = false, g_cancel_called = false, g_send_nav_goal = false, g_previous_state_is_go_to = false, g_message_received = false;
int g_height_threshold = 400;

enum HEIGHT_THRESHOLD
{
    EXCAVATOR = 170,
    SCOUT = 200,
    HAULER = 200,
    PROCESSING_PLANT = 340,
    REPAIR_STATION = 340,
    FURNACE = 150,
    OTHER = 50,
    MINIMUM_THRESH = -1,
};

enum REVOLVE_DIRECTION
{
    CLOCK = -1,
    COUNTER_CLOCK = 1,
};

/**
 * @brief Set the Desired Label Bounding BoxHeight Threshold object
 * 
 */
void setDesiredLabelHeightThreshold()
{
    if (g_desired_label == OBJECT_DETECTION_EXCAVATOR_CLASS)
    {
        g_height_threshold = HEIGHT_THRESHOLD::EXCAVATOR;
    }
    else if (g_desired_label == OBJECT_DETECTION_SCOUT_CLASS)
    {
        g_height_threshold = HEIGHT_THRESHOLD::SCOUT;
    }
    else if (g_desired_label == OBJECT_DETECTION_HAULER_CLASS)
    {
        g_height_threshold = HEIGHT_THRESHOLD::HAULER;
    }
    else if (g_desired_label == OBJECT_DETECTION_PROCESSING_PLANT_CLASS)
    {
        g_height_threshold = HEIGHT_THRESHOLD::PROCESSING_PLANT;
    }
    else if (g_desired_label == OBJECT_DETECTION_REPAIR_STATION_CLASS)
    {
        g_height_threshold = HEIGHT_THRESHOLD::REPAIR_STATION;
    }
    else
    {
        g_height_threshold = HEIGHT_THRESHOLD::OTHER;
    }
}

/**
 * @brief Checks whether the desire target label is a valid object detection class or not
 * 
 * @return true - if the class is valid
 * @return false - if the class is invalid
 */
bool check_class()
{
    if (g_desired_label == OBJECT_DETECTION_PROCESSING_PLANT_CLASS ||
        g_desired_label == OBJECT_DETECTION_REPAIR_STATION_CLASS ||
        g_desired_label == OBJECT_DETECTION_EXCAVATOR_CLASS ||
        g_desired_label == OBJECT_DETECTION_SCOUT_CLASS ||
        g_desired_label == OBJECT_DETECTION_FURNACE_CLASS ||
        g_desired_label == OBJECT_DETECTION_HAULER_CLASS)
        return true;
    return false;
}

/**
 * @brief Callback function which subscriber to Objects message published from object detection
 * 
 * @param objs 
 */
void objectsCallback(const perception::ObjectArray &objs)
{
    const std::lock_guard<std::mutex> lock(g_objects_mutex);
    g_message_received = true;
    g_objects = objs;
}

/**
 * @brief Function for centering robot wrt object
 * 
 * Steps:
 * 1. Rotate robot util the desired object detection class has its bounding box in the center of the frame
 */
bool center()
{
    const std::lock_guard<std::mutex> lock(g_objects_mutex);
    perception::ObjectArray objects = g_objects;
    // Initialize location and size variables
    float center_obj = INIT_VALUE, error_angle = WIDTH_IMAGE;

    static float prev_angular_velocity;
    static int lost_detection_times = 0, true_detection_times = 0, revolve_direction = -1, centered_times = 0;

    if (centered_times > 10)
    {
        g_nav_goal.angular_velocity = 0;
        centered_times = 0;
        ROS_INFO_STREAM(g_robot_name << " NAV VISION : Center finished to " << g_desired_label);
        return true;
    }

    // Find the desired objects
    for (int i = 0; i < objects.number_of_objects; i++)
    {
        perception::Object object = objects.obj.at(i);
        if (object.label == g_desired_label)
        {
            // Store the object's center
            center_obj = object.center.x;
        }
    }

    if (center_obj < HEIGHT_THRESHOLD::MINIMUM_THRESH)
    {
        // object not detected, rotate on robot's axis to find the object
        lost_detection_times++;
        true_detection_times = 0;
        if (lost_detection_times > LOST_FRAME_THRESHOLD)
        {
            g_nav_goal.angular_velocity = revolve_direction * ANGULAR_VELOCITY;
            g_nav_goal.forward_velocity = 0;
        }
        return false;
    }
    else
    {
        g_nav_goal.direction = 0;
        lost_detection_times = 0;
        true_detection_times++;

        // object found, compute the error in angle i.e. the error between the center of image and center of bounding box
        float center_img = (WIDTH_IMAGE / 2.0);
        error_angle = center_img - center_obj;

        if (error_angle < 0)
        {
            revolve_direction = REVOLVE_DIRECTION::CLOCK;
        }
        else
        {
            revolve_direction = REVOLVE_DIRECTION::COUNTER_CLOCK;
        }

        if (abs(error_angle) < ANGLE_THRESHOLD_NARROW)
        {
            // if the bounding box is in the center of image following the narrow angle
            centered_times++;

            if (centered_times > 10)
            {
                g_nav_goal.angular_velocity = 0;
            }
        }
        else
        {
            centered_times = 0;
            // proportional controller for calculating angular velocity needed to center the object in the image
            g_nav_goal.angular_velocity = error_angle * PROPORTIONAL_ANGLE;
            g_nav_goal.forward_velocity = 0;

            if (g_nav_goal.angular_velocity < prev_angular_velocity - g_angular_vel_step_size)
            {
                g_nav_goal.angular_velocity = prev_angular_velocity - g_angular_vel_step_size;
            }
            if (g_nav_goal.angular_velocity > prev_angular_velocity + g_angular_vel_step_size)
            {
                g_nav_goal.angular_velocity = prev_angular_velocity + g_angular_vel_step_size;
            }
        }
    }

    // maintaing previous values
    prev_angular_velocity = g_nav_goal.angular_velocity;
}

/**
 * @brief Function for centering mode for robot wrt object
 * 
 * Steps:
 * 1. Rotate robot util the desired object detection class has its bounding box in the center of the frame
 */
void centering()
{
    if (center())
        g_reached_goal = true;
}

/**
 * @brief Function for navigation robot away from an object
 * 
 * Steps:
 * 1. Rotate robot util the desired object detection class has its bounding box in the center of the frame
 * 2. Drive back from the object
 */
void undock()
{
    static bool centered = false;

    if (centered)
    {
        if (!g_send_nav_goal && g_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO_STREAM(g_robot_name << " NAV VISION: Undocked Successfully");
            g_reached_goal = true;
            centered = false;
            g_send_nav_goal = false;
        }
        if (g_send_nav_goal)
        {
            geometry_msgs::PoseStamped pt;
            pt.header.frame_id = g_robot_name + ROBOT_BASE;
            pt.pose.position.y = -5;
            g_nav_goal.drive_mode = NAV_TYPE::GOAL;
            g_nav_goal.pose = pt;
            g_client->sendGoal(g_nav_goal);
            g_send_nav_goal = false;
        }
    }

    if (!centered)
    {
        centered = center();
    }
}

/**
 * @brief Function for navigating a robot near to an object detection based class
 * 
 * Steps:
 * 1. Rotate robot util the desired object detection class has its bounding box in the center of the frame
 * 2. Drive forward until you reach the desired class bounding box's minimum height
 * 3. Avoid obstacle using object detection, if the obstacle is in projected path, it means that the obstacle will be in robot's path, so robot crab drives until
 *    there is no obstacle in projected path, an object will be considered an obstacle iff it is not target label and is greater than a height threshold (currently rocks are only considered as obstacles)
 * If the object is lost while the above process, the process will be started again * 
 * Two thresholds are used for centering the object in the image, narrow threshold for initial centering and wide threshold if the object was centered
 * but looses the center afterwards
 */
void visionNavigation()
{
    const std::lock_guard<std::mutex> lock(g_objects_mutex);
    perception::ObjectArray objects = g_objects;

    static float prev_angular_velocity;
    static bool prev_centered, centered = false;
    static int lost_detection_times = 0, true_detection_times = 0, revolve_direction = -1;

    // Initialize location and size variables
    float center_obj = INIT_VALUE, height_obj = INIT_VALUE;
    // Initialize error, P Control, and necessary thresholds
    float error_angle = WIDTH_IMAGE, error_height = HEIGHT_IMAGE;

    std::vector<perception::Object> obstacles;
    float err_obstacle = 0;

    bool target_processing_plant = (g_desired_label == OBJECT_DETECTION_PROCESSING_PLANT_CLASS);
    bool target_excavator = (g_desired_label == OBJECT_DETECTION_EXCAVATOR_CLASS);

    // Find the desired objects
    for (int i = 0; i < objects.number_of_objects; i++)
    {
        perception::Object object = objects.obj.at(i);
        bool object_is_furnace = (object.label == OBJECT_DETECTION_FURNACE_CLASS);
        bool object_is_excavator_arm = (object.label == OBJECT_DETECTION_EXCAVATOR_ARM_CLASS);
        if (object.label == g_desired_label)
        {
            // Store the object's center and height
            center_obj = object.center.x;
            height_obj = object.size_y;
        }
        else if (target_processing_plant && object_is_furnace)
        {
            // do not consider furnace as an obstacle when going to processing plant
            continue;
        }
        else if (target_excavator && object_is_excavator_arm)
        {
            // do not consider excavator arm as an obstacle when going to excavator
            continue;
        }
        else
            obstacles.push_back(object);
    }

    if (center_obj < HEIGHT_THRESHOLD::MINIMUM_THRESH)
    {
        // object not detected, rotate on robot's axis to find the object
        lost_detection_times++;
        true_detection_times = 0;
        if (lost_detection_times > LOST_FRAME_THRESHOLD)
        {
            g_nav_goal.angular_velocity = revolve_direction * ANGULAR_VELOCITY;
            g_nav_goal.forward_velocity = 0;
        }
        return;
    }
    else
    {
        // get the direction of crab walk needed to avoid obstacle
        float direction = checkObstacle(obstacles);

        if (abs(direction) > 0.0)
        {
            // if there is an obstacle to avoid, crab walk
            g_nav_goal.forward_velocity = FORWARD_VELOCITY;
            g_nav_goal.direction = direction;
            g_nav_goal.angular_velocity = 0;
            ROS_INFO("Avoid Obstacle Mode");
            return;
        }

        g_nav_goal.direction = 0;

        lost_detection_times = 0;
        true_detection_times++;
        // object found, compute the error in angle i.e. the error between the center of image and center of bounding box
        float center_img = (WIDTH_IMAGE / 2.0) + err_obstacle;
        error_angle = center_img - center_obj;
        // compute error in height, desired height minus current height of bounding box
        error_height = g_height_threshold - height_obj;

        if (error_angle < 0)
        {
            revolve_direction = REVOLVE_DIRECTION::CLOCK;
        }
        else
        {
            revolve_direction = REVOLVE_DIRECTION::COUNTER_CLOCK;
        }

        if (abs(error_angle) > ANGLE_THRESHOLD_WIDE)
        {
            // if the bounding box is not in the center of the image
            centered = false;
        }

        if (centered || abs(error_angle) < ANGLE_THRESHOLD_NARROW)
        {
            // if the bounding box is in the center of image following the narrow angle
            centered = true;
            g_nav_goal.angular_velocity = 0;
            if (error_height < 0 && true_detection_times > FOUND_FRAME_THRESHOLD)
            {
                // If the object is having desired height, stop the robot
                g_nav_goal.forward_velocity = 0;
                g_reached_goal = true;
                centered = false;
                ROS_INFO_STREAM(g_robot_name << " NAV VISION: Reached Goal - " << g_desired_label);
                return;
            }
            else
            {
                // Keep driving forward according to height of the object
                g_nav_goal.forward_velocity = FORWARD_VELOCITY;
            }
        }
        else
        {
            // proportional controller for calculating angular velocity needed to center the object in the image
            g_nav_goal.angular_velocity = error_angle * PROPORTIONAL_ANGLE;
            g_nav_goal.forward_velocity = 0;

            if (g_nav_goal.angular_velocity < prev_angular_velocity - g_angular_vel_step_size)
            {
                g_nav_goal.angular_velocity = prev_angular_velocity - g_angular_vel_step_size;
            }
            if (g_nav_goal.angular_velocity > prev_angular_velocity + g_angular_vel_step_size)
            {
                g_nav_goal.angular_velocity = prev_angular_velocity + g_angular_vel_step_size;
            }
        }
    }

    if (prev_centered && !centered)
    {
        g_nav_goal.angular_velocity = 0;
        g_nav_goal.forward_velocity = 0;
    }

    // maintaing previous values
    prev_angular_velocity = g_nav_goal.angular_velocity;
    prev_centered = centered;
}

/**
 * @brief Function for obstacle avoidance + go to goal
 * 
 */
void goToGoalObsAvoid(const geometry_msgs::PoseStamped &goal_loc)
{
    const std::lock_guard<std::mutex> obj_lock(g_objects_mutex);
    const std::lock_guard<std::mutex> odom_lock(g_odom_mutex);

    perception::ObjectArray objects = g_objects;

    std::vector<perception::Object> obstacles;

    for (int i = 0; i < objects.number_of_objects; i++)
        obstacles.push_back(objects.obj.at(i));

    float direction = checkObstacle(obstacles);
    double distance = NavigationAlgo::changeInPosition(g_robot_pose, goal_loc);

    if (abs(direction) > 0.0 && distance > NOT_AVOID_OBSTACLE_THRESHOLD)
    {
        g_nav_goal.drive_mode = NAV_TYPE::MANUAL;
        g_nav_goal.forward_velocity = FORWARD_VELOCITY;
        g_nav_goal.direction = direction;
        g_nav_goal.angular_velocity = 0;
        g_send_nav_goal = true;
        g_previous_state_is_go_to = false;
        ROS_INFO("Avoiding Obstacle");
    }
    else
    {
        g_nav_goal.drive_mode = NAV_TYPE::GOAL;
        g_nav_goal.pose = goal_loc;
        ROS_INFO_STREAM(g_robot_name << " Outgoing nav vision goal" << goal_loc);
        if (g_previous_state_is_go_to)
        {
            g_send_nav_goal = false;
            if (g_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO_STREAM(g_robot_name << " NAV VISION: Reached GoTo Goal");
                g_reached_goal = true;
            }
        }
        g_previous_state_is_go_to = true;
    }
}

/**
 * @brief Function handling mode: NAV_AND_NAV_VISION which does the following thing:
 * 1. Sends navigation goal for target location
 * 2. As soon as the robot is near the target location (10 meter in radius), starts looking for the target label, as soon as it finds the target label, 
 * switches to navigation vision
 * 
 * @param goal_loc 
 */
void goToLocationAndObject(const geometry_msgs::PoseStamped &goal_loc)
{

    static int object_found_frames = 0;

    if (object_found_frames > 10)
    {
        // ROS_INFO("Going to vision navigation");
        g_nav_goal.drive_mode = NAV_TYPE::MANUAL;
        g_send_nav_goal = true;
        visionNavigation();
        if (g_reached_goal)
        {
            object_found_frames = 0;
        }
        return;
    }

    if (g_send_nav_goal)
    {
        object_found_frames = 0;
        g_nav_goal.drive_mode = NAV_TYPE::GOAL;
        g_nav_goal.pose = goal_loc;
        g_client->sendGoal(g_nav_goal);
        if (g_previous_state_is_go_to)
        {
            g_send_nav_goal = false;
        }
        g_previous_state_is_go_to = true;
    }

    if (g_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        // ROS_INFO("Going to vision navigation");
        g_nav_goal.drive_mode = NAV_TYPE::MANUAL;
        g_send_nav_goal = true;
        visionNavigation();
    }

    const std::lock_guard<std::mutex> odom_lock(g_odom_mutex);
    double distance = NavigationAlgo::changeInPosition(g_robot_pose, goal_loc);

    if (distance > 20)
    {
        return;
    }

    // ROS_INFO("Looking for object");

    bool object_found = false;

    std::unique_lock<std::mutex> obj_lock(g_objects_mutex);
    perception::ObjectArray objects = g_objects;
    obj_lock.unlock();

    // Find the desired objects
    for (int i = 0; i < objects.number_of_objects; i++)
    {
        perception::Object object = objects.obj.at(i);
        if (object.label == g_desired_label)
        {
            // ROS_INFO("Found object");
            // Store the object's center
            object_found = true;
        }
    }

    if (object_found)
    {
        object_found_frames++;
    }
    else
    {
        object_found_frames = 0;
    }
}

/**
 * @brief Function called when the goal is cancelled
 * 
 */
void cancelGoal()
{
    const std::lock_guard<std::mutex> lock(g_cancel_goal_mutex);
    g_cancel_called = true;

    g_nav_goal.forward_velocity = 0;
    g_nav_goal.angular_velocity = 0;
    g_client->sendGoal(g_nav_goal);

    ROS_INFO_STREAM(g_robot_name << " NAV VISION : Cancelled Goal");
}

/**
 * @brief Function which gets executed when any goal is received to actionlib
 * 
 * @param goal for action lib
 * @param as variable to send feedback
 */
void execute(const operations::NavigationVisionGoalConstPtr &goal, Server *as)
{
    operations::NavigationVisionResult result;

    NAV_VISION_TYPE mode = (NAV_VISION_TYPE)goal->mode;
    ROS_INFO_STREAM(g_robot_name << " NAV VISION : Goal Received - " << mode);

    if (mode == NAV_VISION_TYPE::V_FOLLOW || mode == NAV_VISION_TYPE::V_REACH || mode == NAV_VISION_TYPE::V_UNDOCK || mode == NAV_VISION_TYPE::V_CENTER || mode == NAV_VISION_TYPE::V_NAV_AND_NAV_VISION)
    {
        g_nav_goal.drive_mode = NAV_TYPE::MANUAL;
        g_desired_label = goal->desired_object_label;
        ROS_INFO_STREAM("Desired label =" << g_desired_label);
        if (!check_class())
        {
            // the class is not valid, send the appropriate result
            result.result = COMMON_RESULT::INVALID_GOAL;
            as->setAborted(result, "Invalid Object Detection Class or Cannot go to the class");
            ROS_INFO("Invalid Object Detection Class or Cannot go to the class");
            return;
        }

        // Set the desired bounding box target height according to the desired target class
        setDesiredLabelHeightThreshold();
    }
    else if (mode == NAV_VISION_TYPE::V_OBS_GOTO_GOAL || mode == NAV_VISION_TYPE::V_NAV_AND_NAV_VISION)
    {
        g_previous_state_is_go_to = false;
    }

    g_send_nav_goal = true;
    g_cancel_called = false;
    g_reached_goal = false;

    ros::Rate update_rate(UPDATE_HZ);

    while (ros::ok() && !g_reached_goal && !g_cancel_called)
    {
        if (!g_message_received)
            continue;

        switch (mode)
        {
        case NAV_VISION_TYPE::V_FOLLOW:
            visionNavigation();
            g_client->sendGoal(g_nav_goal);
            g_reached_goal = false;
            break;
        case NAV_VISION_TYPE::V_REACH:
            visionNavigation();
            g_client->sendGoal(g_nav_goal);
            break;
        case NAV_VISION_TYPE::V_UNDOCK:
            undock();
            if (g_send_nav_goal)
            {
                g_client->sendGoal(g_nav_goal);
            }
            break;
        case NAV_VISION_TYPE::V_CENTER:
            centering();
            g_client->sendGoal(g_nav_goal);
            break;
        case NAV_VISION_TYPE::V_OBS_GOTO_GOAL:
            goToGoalObsAvoid(goal->goal_loc);
            if (g_send_nav_goal)
            {
                g_client->sendGoal(g_nav_goal);
            }
            break;
        case NAV_VISION_TYPE::V_NAV_AND_NAV_VISION:
            goToLocationAndObject(goal->goal_loc);
            if (g_send_nav_goal)
            {
                g_client->sendGoal(g_nav_goal);
            }
            break;
        default:
            ROS_ERROR_STREAM(g_robot_name + " NAV VISION: Encountered Unhandled State!");
            result.result = COMMON_RESULT::INVALID_GOAL;
            as->setSucceeded(result, "Cancelled Goal");
            return;
        }

        update_rate.sleep();
        const std::lock_guard<std::mutex> lock(g_cancel_goal_mutex);
    }

    if (g_cancel_called)
    {
        g_cancel_called = false;
        result.result = COMMON_RESULT::INTERRUPTED;
        as->setSucceeded(result, "Cancelled Goal");
        return;
    }

    g_reached_goal = false;
    result.result = COMMON_RESULT::SUCCESS;
    as->setSucceeded(result);
}

/**
 * @brief Callback to the robot pose topic
 * 
 * @param msg 
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    const std::lock_guard<std::mutex> lock(g_odom_mutex);
    g_robot_pose.header = msg->header;
    g_robot_pose.pose = msg->pose.pose;
}

int main(int argc, char **argv)
{
    if (argc != 2 && argc != 4)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");
        return -1;
    }

    g_robot_name = argv[1];
    ros::init(argc, argv, g_robot_name + NAVIGATION_VISION_SERVER_NODE_NAME);
    ros::NodeHandle nh;

    g_nav_goal.drive_mode = NAV_TYPE::MANUAL;
    g_client = new Client(CAPRICORN_TOPIC + g_robot_name + "/" + NAVIGATION_ACTIONLIB, true);

    ros::Subscriber objects_sub = nh.subscribe(CAPRICORN_TOPIC + g_robot_name + OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objectsCallback);

    // example of intended subscriber-> ros::Subscriber robot_odom_sub = nh.subscribe("/small_excavator_1/camera/odom", 1000, &odomCallback);
    ros::Subscriber robot_odom_sub = nh.subscribe("/" + g_robot_name + RTAB_ODOM_TOPIC, 1000, &odomCallback);

    Server server(nh, g_robot_name + NAVIGATION_VISION_ACTIONLIB, boost::bind(&execute, _1, &server), false);
    server.registerPreemptCallback(&cancelGoal);
    server.start();
    ROS_INFO("Starting Navigation Vision Server");
    ros::spin();
    return 0;
}
