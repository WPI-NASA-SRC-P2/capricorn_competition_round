#include<operations/hauler_state_machine.h>

HaulerStateMachine::HaulerStateMachine(ros::NodeHandle nh, const std::string& robot_name):nh_(nh)
{
    robot_name_ = robot_name;
    
    navigation_client_ = new NavigationClient(NAVIGATION_ACTIONLIB, true);
    hauler_client_ = new HaulerClient(HAULER_ACTIONLIB, true);

    // dig_site_location_ = nh_.subscribe("/"+CAPRICORN_TOPIC + SCOUT_1 + VOLATILE_LOCATION_TOPIC, 1000, &HaulerStateMachine::scoutVolLocCB, this);
    hauler_parked_pub_ = nh.advertise<std_msgs::Empty>("/"+CAPRICORN_TOPIC + HAULER_ARRIVED_TOPIC, 1000);
}

HaulerStateMachine::~HaulerStateMachine()
{
    delete navigation_client_;
    delete hauler_client_;
}

// void HaulerStateMachine::scoutVolLocCB(const geometry_msgs::PoseStamped &msg)
// {
//     // Lock-guards are not desired here. This message is published once per volatile
//     // So, there may not be a case in which we need to 'keep hold' on the old message
//     // Schedular should take care of sending tasks only when the hauler is free to 
//     // Take up the task
//     vol_pose_ = msg;
//     volatile_found_ = true;
// }

void HaulerStateMachine::startStateMachine()
{
    // Waiting for the servers to start
    navigation_client_->waitForServer();
    hauler_client_->waitForServer();

    while (ros::ok() && state_machine_continue_)
    {   
        switch (robot_state_)
        {
        case HAULER_STATES::INIT:
            // initState();
            break;
        case HAULER_STATES::GO_TO_DIG_SITE:
            // goToVolatile();
            break;
        case HAULER_STATES::PARK_AT_EXCAVATOR:
            // parkHauler();
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

// void HaulerStateMachine::initState()
// {
//     if(volatile_found_)
//     {
//         robot_state_ = GO_TO_VOLATILE;
//         volatile_found_ = false;
//         ROS_INFO("Going to location");
//     }
//     return;
// }

// void HaulerStateMachine::goToVolatile()
// {
//     ROS_INFO("Goal Received");
//     if(nav_server_idle_)
//     {
//         ROS_INFO("Goal action requested");
        
//         // This is big hack for the demo. Ideally, it should still try to get to the location, and 
//         // terminate when it is close enough. We don't have a functionality for 'close enough' 
//         // Hence this hack
//         geometry_msgs::PoseStamped temp_location = vol_pose_;
//         temp_location.pose.position.y -= 5;

//         navigation_action_goal_.pose = temp_location;
//         navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

//         navigation_client_->sendGoal(navigation_action_goal_);
//         nav_server_idle_ = false;
//     }
//     else if(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) // ( || scout_found || close_enough)
//     {
//         ROS_INFO("GOAL FINISHED");
//         robot_state_ = PARK_AND_PUB;
//         nav_server_idle_ = true;
//         std_msgs::Empty empty_message;
//         hauler_ready_pub_.publish(empty_message);
//     }
// }

// void HaulerStateMachine::parkHauler()
// {
//     ROS_INFO("Actual Goal Received");
//     if(nav_server_idle_)
//     {
//         ROS_INFO("Goal action requested");
//         // Again, hack for the demo. It should register the location of scout, and 
//         // go to the location where it thought the scout was.
//         navigation_action_goal_.pose = vol_pose_;
//         navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

//         navigation_client_->sendGoal(navigation_action_goal_);
//         nav_server_idle_ = false;
//     }
//     else if(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
//     {
//         ROS_INFO("GOAL FINISHED");
//         robot_state_ = DIG_VOLATILE;
//         nav_server_idle_ = true;
//     }
// }

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