#include<operations/excavator_state_machine.h>

ExcavatorStateMachine::ExcavatorStateMachine(ros::NodeHandle nh, const std::string& robot_name):nh_(nh)
{
    robot_name_ = robot_name;
    
    navigation_client_ = new NavigationClient(NAVIGATION_ACTIONLIB, true);
    excavator_arm_client_ = new ExcavatorClient(EXCAVATOR_ACTIONLIB, true);

    sub_scout_vol_location_ = nh_.subscribe("/" + CAPRICORN_TOPIC + "/" + robot_name_ + VOLATILE_LOCATION_TOPIC, 1000, &ExcavatorStateMachine::scoutVolLocCB, this);
    excavator_ready_pub_ = nh.advertise<std_msgs::Empty>("/"+CAPRICORN_TOPIC + EXCAVATOR_ARRIVED_TOPIC, 1000);
}

ExcavatorStateMachine::~ExcavatorStateMachine()
{
    delete navigation_client_;
    delete excavator_arm_client_;
}

void ExcavatorStateMachine::scoutVolLocCB(const geometry_msgs::PoseStamped &msg)
{
    // Lock-guards are not desired here. This message is published once per volatile
    // So, there may not be a case in which we need to 'keep hold' on the old message
    // Schedular should take care of sending tasks only when the excavator is free to 
    // Take up the task
    vol_pose_ = msg;
    volatile_found_ = true;
}

void ExcavatorStateMachine::startStateMachine()
{
    // Waiting for the servers to start
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
            goToVolatile();
            break;
        case EXCAVATOR_STATES::PARK_AND_PUB:
            parkExcavator();
            break;
        case EXCAVATOR_STATES::DIG_VOLATILE:
            digVolatile();
            break;
        case EXCAVATOR_STATES::DUMP_VOLATILE:
            dumpVolatile();
            break;
        case EXCAVATOR_STATES::NEXT_QUE_TASK:
            robot_state_ = INIT; 
            // this is temporary, ideally it should publish to schedular
            // that it is available, and schedular should send the next
            // Goal to follow. 
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

void ExcavatorStateMachine::goToVolatile()
{
    ROS_INFO("Goal Received");
    if(nav_server_idle_)
    {
        ROS_INFO("Goal action requested");
        
        // This is big hack for the demo. Ideally, it should still try to get to the location, and 
        // terminate when it is close enough. We don't have a functionality for 'close enough' 
        // Hence this hack
        geometry_msgs::PoseStamped temp_location = vol_pose_;
        temp_location.pose.position.y -= 5;

        navigation_action_goal_.pose = temp_location;
        navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

        navigation_client_->sendGoal(navigation_action_goal_);
        nav_server_idle_ = false;
    }
    else if(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) // ( || scout_found || close_enough)
    {
        ROS_INFO("GOAL FINISHED");
        robot_state_ = PARK_AND_PUB;
        nav_server_idle_ = true;
        std_msgs::Empty empty_message;
        excavator_ready_pub_.publish(empty_message);
    }
}

void ExcavatorStateMachine::parkExcavator()
{
    ROS_INFO("Actual Goal Received");
    if(nav_server_idle_)
    {
        ROS_INFO("Goal action requested");
        // Again, hack for the demo. It should register the location of scout, and 
        // go to the location where it thought the scout was.
        navigation_action_goal_.pose = vol_pose_; 
        navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

        navigation_client_->sendGoal(navigation_action_goal_);
        nav_server_idle_ = false;
    }
    else if(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("GOAL FINISHED");
        robot_state_ = DIG_VOLATILE;
        nav_server_idle_ = true;
    }
}

void ExcavatorStateMachine::digVolatile()
{
    if(excavator_server_idle_)
    {
        operations::ExcavatorGoal goal;
        goal.task = START_DIGGING; 
        
        // Odom location is unreliable to use. 
        // How to get these values / Do we need it?
        // These are the testing values, can be chagned
        // They start digging from the left of the robot
        goal.target.x = 0.7; 
        goal.target.y = 2;
        goal.target.z = 0;

        excavator_arm_client_->sendGoal(goal);
        excavator_server_idle_ = false;
    }
    else
    {
        // if(volatile not found)
        //      Change location or something 
        // else if
        if (excavator_arm_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {    
            excavator_server_idle_ = true;
            robot_state_ = DUMP_VOLATILE;
        }
    }
}

void ExcavatorStateMachine::dumpVolatile()
{
    if(excavator_server_idle_)
    {
        operations::ExcavatorGoal goal;
        goal.task = START_UNLOADING; 
        
        // Should be tested with Endurance's parking code and 
        // These values should be tuned accordingly
        goal.target.x = 0.7; 
        goal.target.y = -2;
        goal.target.z = 0;

        excavator_arm_client_->sendGoal(goal);
        excavator_server_idle_ = false;
    }
    else
    {
        if (excavator_arm_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {    
            excavator_server_idle_ = true;
            //  if(volatile completely dug)
            //      next_state
            //  else
            robot_state_ = DIG_VOLATILE;
        }
    }
}