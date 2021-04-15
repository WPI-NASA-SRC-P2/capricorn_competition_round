#include<operations/hauler_state_machine.h>

HaulerStateMachine::HaulerStateMachine(ros::NodeHandle nh, const std::string& robot_name):nh_(nh)
{
    robot_name_ = robot_name;
    
    // Actionlib initializations
    navigation_client_ = new NavigationClient(NAVIGATION_ACTIONLIB, true);
    hauler_client_ = new HaulerClient(HAULER_ACTIONLIB, true);
    navigation_vision_client_ = new NavigationVisionClient(robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
    park_robot_client_ = new ParkRobotClient(robot_name + COMMON_NAMES::PARK_HAULER_ACTIONLIB, true);

    // Subscriber initializations
    dig_site_location_ = nh_.subscribe(CAPRICORN_TOPIC + "/" + robot_name_ + VOLATILE_LOCATION_TOPIC, 1000, &HaulerStateMachine::digSiteLocCB, this);
    hauler_filled_sub_ = nh_.subscribe(CAPRICORN_TOPIC + "/" + robot_name_ + HAULER_FILLED, 1000, &HaulerStateMachine::haulerFilledCB, this);
}

HaulerStateMachine::~HaulerStateMachine()
{
    delete navigation_client_;
    delete hauler_client_;
    delete navigation_vision_client_;
    delete park_robot_client_;
}

void HaulerStateMachine::digSiteLocCB(const geometry_msgs::PoseStamped &msg)
{
    // TODO: lock guard
    dig_site_pose_ = msg;
    volatile_found_ = true;
}

void HaulerStateMachine::haulerFilledCB(const std_msgs::Empty &msg)
{
    // TODO: lock guard
    hauler_filled_ = true;
}

void HaulerStateMachine::startStateMachine()
{
    // Waiting for the servers to start
    navigation_client_->waitForServer();
    hauler_client_->waitForServer();
    navigation_vision_client_->waitForServer();
    park_robot_client_->waitForServer();
    ROS_WARN("All actionlib servers ready");
    
    while (ros::ok() && state_machine_continue_)
    {   
        switch (robot_state_)
        {
        case HAULER_STATES::INIT:
            initState();
            break;
        case HAULER_STATES::GO_TO_DIG_SITE:
            goToDigSite();
            break;
        case HAULER_STATES::FOLLOW_EXCAVATOR:
            followExcavator();
            break;
        case HAULER_STATES::PARK_AT_EXCAVATOR:
            parkAtExcavator();
            break;
        case HAULER_STATES::ACCEPT_VOLATILE:
            waitTillFilled();
            break;
        case HAULER_STATES::GO_TO_PROC_PLANT:
            goToProcPlant();
            break;
        case HAULER_STATES::PARK_AT_HOPPER:
            parkAtHopper();
            break;
        case HAULER_STATES::DUMP_VOLATILE:
            dumpVolatile();
            break;

        default:
            ROS_ERROR_STREAM(robot_name_ + " state machine encountered unhandled state!");
            break;
        }
        ros::Duration(SLEEP_TIME).sleep();
        ros::spinOnce();
    }
}

void HaulerStateMachine::initState()
{
    if(volatile_found_)
    {
        robot_state_ = GO_TO_DIG_SITE;
        volatile_found_ = false;
    }
}

void HaulerStateMachine::goToDigSite()
{
    if(nav_server_idle_)
    {
        ROS_INFO("Goal action requested");
        
        // This is big hack for the demo. Ideally, hauler should follow the excavator
        // But we do not have follow functionality. It can only 'reach' at a given location
        // Currently, for parking, the Hauler should be in the lower 3rd quadrant (bottom right) of the excavator
        // Hence this hack
        geometry_msgs::PoseStamped temp_location = dig_site_pose_;
        temp_location.pose.position.x -= 5;
        temp_location.pose.position.y -= 5;

        navigation_action_goal_.pose = temp_location;
        navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

        navigation_client_->sendGoal(navigation_action_goal_);
        nav_server_idle_ = false;
    }
    else if(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("GOAL FINISHED");
        robot_state_ = FOLLOW_EXCAVATOR;
        nav_server_idle_ = true;
    }
}

void HaulerStateMachine::followExcavator()
{
    if(nav_vis_server_idle_)
    {
        ROS_INFO("Following Excavator");
        navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS; 
        navigation_vision_client_->sendGoal(navigation_vision_goal_);
        nav_vis_server_idle_ = false;
    }
    else if(navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("Followed to the death");
        robot_state_ = PARK_AT_EXCAVATOR;
        nav_vis_server_idle_ = true;
    }
}

void HaulerStateMachine::parkAtExcavator()
{
    if(park_server_idle_)
    {
        ROS_INFO("Parking Hauler");
        park_robot_goal_.hopper_or_excavator = OBJECT_DETECTION_EXCAVATOR_CLASS;
        park_robot_client_->sendGoal(park_robot_goal_);
        park_server_idle_ = false;
    }
    else if(park_robot_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("Parked");
        robot_state_ = ACCEPT_VOLATILE;
        park_server_idle_ = true;
    }
}

void HaulerStateMachine::waitTillFilled()
{
    if(hauler_filled_)
    {
        ROS_INFO("Parking Hauler");
        robot_state_ = GO_TO_PROC_PLANT;
        hauler_filled_ = false;
    }
}

void HaulerStateMachine::goToProcPlant()
{
    if(nav_vis_server_idle_)
    {
        ROS_INFO("Going To Proc Plant");
        navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS; 
        navigation_vision_client_->sendGoal(navigation_vision_goal_);
        nav_vis_server_idle_ = false;
    }
    else if(navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("Followed to the death");
        robot_state_ = PARK_AT_HOPPER;
        nav_vis_server_idle_ = true;
    }
}

void HaulerStateMachine::parkAtHopper()
{
    if(park_server_idle_)
    {
        ROS_INFO("Parking Hauler");
        park_robot_goal_.hopper_or_excavator = OBJECT_DETECTION_HOPPER_CLASS;
        park_robot_client_->sendGoal(park_robot_goal_);
        park_server_idle_ = false;
    }
    else if(park_robot_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("Parked");
        robot_state_ = DUMP_VOLATILE;
        park_server_idle_ = true;
    }
}

void HaulerStateMachine::dumpVolatile()
{
    if(hauler_server_idle_)
    {
        ROS_INFO("Dumping");
        hauler_goal_.desired_state = true;
        hauler_client_->sendGoal(hauler_goal_);
        hauler_server_idle_ = false;
    }
    else if (hauler_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {    
        ROS_INFO("Done dona done");
        hauler_server_idle_ = true;
        robot_state_ = INIT;
    }
}