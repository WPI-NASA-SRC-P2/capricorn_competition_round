#include<operations/hauler_state_machine.h>

HaulerStateMachine::HaulerStateMachine(ros::NodeHandle nh, const std::string& robot_name):nh_(nh)
{
    robot_name_ = robot_name;
    
    navigation_client_ = new NavigationClient(NAVIGATION_ACTIONLIB, true);
    hauler_client_ = new HaulerClient(HAULER_ACTIONLIB, true);
    navigation_vision_client_ = new NavigationVisionClient(robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
    park_robot_client_ = new ParkRobotClient(robot_name + COMMON_NAMES::PARK_HAULER_ACTIONLIB, true);

    dig_site_location_ = nh_.subscribe("/"+CAPRICORN_TOPIC + "/" + robot_name_ + VOLATILE_LOCATION_TOPIC, 1000, &HaulerStateMachine::digSiteLocCB, this);
    hauler_parked_pub_ = nh.advertise<std_msgs::Empty>("/"+CAPRICORN_TOPIC + HAULER_ARRIVED_TOPIC, 1000);
}

HaulerStateMachine::~HaulerStateMachine()
{
    delete navigation_client_;
    delete hauler_client_;
}

void HaulerStateMachine::digSiteLocCB(const geometry_msgs::PoseStamped &msg)
{
    // TODO: lock guard
    ROS_WARN("here");
    dig_site_pose_ = msg;
    volatile_found_ = true;
}

void HaulerStateMachine::startStateMachine()
{
    // Waiting for the servers to start
    navigation_client_->waitForServer();
    hauler_client_->waitForServer();
    ROS_WARN("Ready");
    navigation_vision_client_->waitForServer();
    ROS_WARN("Navigation Client Ready");
    park_robot_client_->waitForServer();
    ROS_WARN("Parker Client Ready");
    
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
            parkHauler();
            break;
        case HAULER_STATES::ACCEPT_VOLATILE:
            // digVolatile();
            break;
        case HAULER_STATES::GO_TO_PROC_PLANT:
            // dumpVolatile();
            break;
        case HAULER_STATES::PARK_AT_HOPPER:
            // robot_state_ = INIT; 
            // this is temporary, ideally it should publish to schedular
            // that it is available, and schedular should send the next
            // Goal to follow. 
            break;
        case HAULER_STATES::DUMP_VOLATILE:
            // dumpVolatile();
            break;

        default:
            ROS_ERROR_STREAM(robot_name_ + " state machine encountered unhandled state!");
            break;
        }
        // sleep for some time
        ros::Duration(SLEEP_TIME).sleep();
        ros::spinOnce();

        // call function to update the states (Because most of these things are actionlibs
        // So they will be running in a seperate node)
        // This function should keep track of the executed actionlibs, and their current states
        // If succeeded
    }
}

void HaulerStateMachine::initState()
{
    if(volatile_found_)
    {
        robot_state_ = GO_TO_DIG_SITE;
        volatile_found_ = false;
        ROS_WARN("Going to location");
    }
    return;
}

void HaulerStateMachine::goToDigSite()
{
    if(nav_server_idle_)
    {
        ROS_WARN("Goal action requested");
        
        // This is big hack for the demo. Ideally, it should still try to get to the location, and 
        // terminate when it is close enough. We don't have a functionality for 'close enough' 
        // Hence this hack
        geometry_msgs::PoseStamped temp_location = dig_site_pose_;
        temp_location.pose.position.x -= 5;
        temp_location.pose.position.y -= 5;

        navigation_action_goal_.pose = temp_location;
        navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

        navigation_client_->sendGoal(navigation_action_goal_);
        nav_server_idle_ = false;
    }
    else if(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) // ( || scout_found || close_enough)
    {
        ROS_WARN("GOAL FINISHED");
        robot_state_ = FOLLOW_EXCAVATOR;
        nav_server_idle_ = true;
    }
}

void HaulerStateMachine::followExcavator()
{
    if(nav_vis_server_idle_)
    {
        ROS_WARN("Following Excavator");

        // desired target object, any object detection class
        navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS; 
        navigation_vision_client_->sendGoal(navigation_vision_goal_);
        nav_vis_server_idle_ = false;
    }
    else if(navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_WARN("Followed to the death");
        robot_state_ = PARK_AT_EXCAVATOR;
        nav_vis_server_idle_ = true;
    }
}


void HaulerStateMachine::parkHauler()
{
    if(nav_server_idle_)
    {
        ROS_WARN("Parking Hauler");
        // Again, hack for the demo. It should register the location of scout, and 
        // go to the location where it thought the scout was.
        park_robot_goal_.hopper_or_excavator = OBJECT_DETECTION_EXCAVATOR_CLASS;

        park_robot_client_->sendGoal(park_robot_goal_);
        nav_server_idle_ = false;
    }
    else if(park_robot_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_WARN("Parked");
        robot_state_ = ACCEPT_VOLATILE;
        nav_server_idle_ = true;
    }
}

// void HaulerStateMachine::digVolatile()
// {
//     if(hauler_server_idle_)
//     {
//         operations::HaulerGoal goal;
//         goal.task = START_DIGGING; 
        
//         // Odom location is unreliable to use. 
//         // How to get these values / Do we need it?
//         // These are the testing values, can be chagned
//         // They start digging from the left of the robot
//         goal.target.x = 0.7; 
//         goal.target.y = 2;
//         goal.target.z = 0;

//         hauler_client_->sendGoal(goal);
//         hauler_server_idle_ = false;
//     }
//     else
//     {
//         // if(volatile not found)
//         //      Change location or something 
//         // else if
//         if (hauler_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//         {    
//             hauler_server_idle_ = true;
//             robot_state_ = DUMP_VOLATILE;
//         }
//     }
// }

// void HaulerStateMachine::dumpVolatile()
// {
//     if(hauler_server_idle_)
//     {
//         operations::HaulerGoal goal;
//         goal.task = START_UNLOADING; 
        
//         // Should be tested with Endurance's parking code and 
//         // These values should be tuned accordingly
//         goal.target.x = 0.7; 
//         goal.target.y = -2;
//         goal.target.z = 0;

//         hauler_client_->sendGoal(goal);
//         hauler_server_idle_ = false;
//     }
//     else
//     {
//         if (hauler_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//         {    
//             hauler_server_idle_ = true;
//             //  if(volatile completely dug)
//             //      next_state
//             //  else
//             robot_state_ = DIG_VOLATILE;
//         }
//     }
// }