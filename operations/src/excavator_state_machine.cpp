#include<operations/excavator_state_machine.h>

ExcavatorStateMachine::ExcavatorStateMachine(ros::NodeHandle nh, const std::string& robot_name):nh_(nh)
{
    robot_name_ = robot_name;
    navigation_client_ = new NavigationClient_(NAVIGATION_ACTIONLIB, true);
    excavator_arm_client_ = new ExcavatorClient_(EXCAVATOR_ACTIONLIB, true);
    sub_scout_vol_location_ = nh_.subscribe(CAPRICORN_TOPIC + SCOUT_1 + VOLATILE_LOCATION_TOPIC, 1000, &ExcavatorStateMachine::scoutVolLocCB, this);
}

ExcavatorStateMachine::~ExcavatorStateMachine()
{
    delete navigation_client_;
    delete excavator_arm_client_;
}

void ExcavatorStateMachine::scoutVolLocCB(const geometry_msgs::PoseStamped &msg)
{
    vol_pose = msg;
    volatile_found_ = true;
    ROS_INFO("Testing for message");
}

void ExcavatorStateMachine::startStateMachine()
{
    navigation_client_->waitForServer();
    excavator_arm_client_->waitForServer();
    while (ros::ok() && state_machine_continue_)
    {
        switch (robot_state_)
        {
        case EXCAVATOR_STATES::INIT:
            initState();
            break;
        case EXCAVATOR_STATES::GO_TO_VOLATILE:
            // goToVolatile();
            break;
        case EXCAVATOR_STATES::PARK_AND_PUB:
            // Call actionlib for resource_locate
            // ResourceLocator
            break;
        case EXCAVATOR_STATES::FIND_VOLATILE:
            // Stop the robot, brake.
            // Wait for a flag from excavator that it is ready to take over
            break;
        case EXCAVATOR_STATES::DIG_AND_DUMP:
            // Once flag is received, move some distance aside. 
            break;
        case EXCAVATOR_STATES::NEXT_QUE_TASK:
            // Mahimana's method to go back to the recharge station
            // Find and get to the recharge station
            break;

        default:
            ROS_ERROR_STREAM(robot_name_ + " state machine encountered unhandled state!");
            break;
        }
        // sleep for some time
        ros::Duration(0.5).sleep();
        ros::spinOnce();

        // call function to update the states (Because most of these things are actionlibs
        // So they will be running in a seperate node)
        // This function should keep track of the executed actionlibs, and their current states
        // If succeeded
    }
}

void ExcavatorStateMachine::initState()
{
    if(volatile_found_)
    {
        robot_state_ = GO_TO_VOLATILE;
        volatile_found_ = false;
        ROS_INFO("Going to location");
    }
    return;
}

// Subscribe to: CAPRICORN_TOPIC + robot_name + VOLATILE_LOCATION_TOPIC
// void ExcavatorStateMachine::volatileLocationCB()
// {
//     // If message received, update the 'volatile_pred_location_' for volatile, 
//     // And set the 'volatile_found_' flag as true
// }

// void ExcavatorStateMachine::goToVolatile()
// {
//     if(navigation_client_->state() == idle)
//     {
//         navigation_action_goal_->volatile_pred_location_;
//         navigation_client_->sendGoal(goal);
//     }
//     else if (navigation_client_->state() == busy)
//     {
//         // wait
//     }
//     else if (completed)
//     {
//         robot_state_ = PARK_AND_PUB;
//     }
// }