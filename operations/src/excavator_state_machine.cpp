#include<operations/excavator_state_machine.h>

ExcavatorStateMachine::ExcavatorStateMachine(ros::NodeHandle nh, const std::string& robot_name):nh_(nh)
{
    robot_name_ = robot_name;
    
    navigation_client_ = new NavigationClient(NAVIGATION_ACTIONLIB, true);
    excavator_arm_client_ = new ExcavatorClient(EXCAVATOR_ACTIONLIB, true);
    navigation_vision_client_ = new NavigationVisionClient(robot_name + NAVIGATION_VISION_ACTIONLIB, true);
    
    objects_sub_ = nh.subscribe("/" + CAPRICORN_TOPIC + robot_name + OBJECT_DETECTION_OBJECTS_TOPIC, 1, &ExcavatorStateMachine::objectsCallback, this);
    lookout_pos_sub_ = nh_.subscribe("/" + CAPRICORN_TOPIC + "/" + robot_name_ + LOOKOUT_LOCATION_TOPIC, 1000, &ExcavatorStateMachine::lookoutLocCB, this);
    sub_scout_vol_location_ = nh_.subscribe("/" + CAPRICORN_TOPIC + "/" + robot_name_ + VOLATILE_LOCATION_TOPIC, 1000, &ExcavatorStateMachine::scoutVolLocCB, this);
    hauler_parked_sub_ = nh.subscribe("/" + CAPRICORN_TOPIC + SCHEDULER_TOPIC + "/hauler_parked", 1, &ExcavatorStateMachine::haulerParkedCB, this);
    
    hauler_go_back_ = nh.advertise<std_msgs::Empty>(CAPRICORN_TOPIC  + HAULER_FILLED, 1000);
    park_hauler_pub_ = nh.advertise<std_msgs::Empty>("/" + CAPRICORN_TOPIC + PARK_HAULER, 1000);
    excavator_ready_pub_ = nh.advertise<std_msgs::Empty>("/" + CAPRICORN_TOPIC + EXCAVATOR_ARRIVED_TOPIC, 1000);
}

ExcavatorStateMachine::~ExcavatorStateMachine()
{
    delete navigation_client_;
    delete excavator_arm_client_;
}

void ExcavatorStateMachine::haulerParkedCB(std_msgs::Empty msg)
{
    ROS_WARN("Callback Received");
    hauler_parked_ = true;
}

void ExcavatorStateMachine::lookoutLocCB(const geometry_msgs::PoseStamped &msg)
{
    ROS_WARN("CB");
    const std::lock_guard<std::mutex> lock(navigation_mutex); 
    next_nav_goal_ = msg;
    lookout_loc_received_ = true;
}


void ExcavatorStateMachine::scoutVolLocCB(const geometry_msgs::PoseStamped &msg)
{
    const std::lock_guard<std::mutex> lock(navigation_mutex); 
    next_nav_goal_ = msg;
    volatile_found_ = true;
}


/**
 * @brief Callback function which subscriber to Objects message published from object detection
 * 
 * @param objs 
 */
void ExcavatorStateMachine::objectsCallback(const perception::ObjectArray& objs) 
{
    const std::lock_guard<std::mutex> lock(g_objects_mutex); 
    g_objects_ = objs;
}

// ISSUES:
    // What if scout finds volatile before reaching/receiving goal?
    // Current state machine is VERY streamlined, and any condition
    // requiring changing states out of order is not coded. 
    // this is VERY ESSENTIAL, as messages will not be received in 
    // any particulaar order

    // For starting with the issue discussed above, should set a 
    // 'desired_state' in subscriber, and current state should 
    // terminate its current job, and execute a 'transition_state'

    // Currently, only thing that is being checked is if 'navigation' 
    // is on or not. That is not a very good way.

void ExcavatorStateMachine::startStateMachine()
{
    // Waiting for the servers to start
    navigation_client_->waitForServer();
    excavator_arm_client_->waitForServer();
    // navigation_vision_client_->waitForServer(); //Not being used currently
    ROS_WARN("Servers started");

    while (ros::ok() && state_machine_continue_)
    {   
        switch (robot_state_)
        {
        case EXCAVATOR_STATES::INIT:
            initState();
            break;
        case EXCAVATOR_STATES::KEEP_LOOKOUT:
            goToLookout();
            break;
        case EXCAVATOR_STATES::GO_TO_SCOUT:
            goToScout();
            break;
        case EXCAVATOR_STATES::FIND_SCOUT:
            findScout();
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
    if(lookout_loc_received_)
    {   
        robot_state_ = KEEP_LOOKOUT;
        lookout_loc_received_ = false;
        lookout_reached_ = false;
        ROS_INFO("Going to lookout location");
    }
    return;
}

void ExcavatorStateMachine::goToLookout()
{
    if(nav_server_idle_ && !lookout_reached_)
    {
        ROS_INFO("Goal action requested");
        
        // This is a location excavator will get via scheduler. The excavator will
        // get to a location close enough to the scout's scouting area, so that
        // when it finds volatile, excavator won't take long to get there.

        navigation_action_goal_.pose = next_nav_goal_;
        navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

        navigation_client_->sendGoal(navigation_action_goal_);
        nav_server_idle_ = false;
    }
    else if(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("GOAL FINISHED");
        nav_server_idle_ = true;
        lookout_reached_ = true;

        if(volatile_found_)
            robot_state_ = GO_TO_SCOUT;
    }
}

void ExcavatorStateMachine::goToScout()
{
    if(nav_server_idle_)
    {
        ROS_INFO("Going going");
        geometry_msgs::PoseStamped temp_msg;
        temp_msg = next_nav_goal_;
        temp_msg.pose.position.x -= 5;
        temp_msg.pose.orientation.w = 1;

        navigation_action_goal_.pose = temp_msg;
        navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

        navigation_client_->sendGoal(navigation_action_goal_);
        nav_server_idle_ = false;
    }
    else if(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("Reached to the scout");
        robot_state_ = FIND_SCOUT;
        nav_server_idle_ = true;
    }
}

void ExcavatorStateMachine::parkExcavator()
{
    ROS_INFO("Actual Goal Received");
    if(nav_server_idle_)
    {
        std_msgs::Empty empty_message;
        excavator_ready_pub_.publish(empty_message);
     
        ROS_INFO("Goal action requested");

        // Again, hack for the demo. It should register the location of scout, and 
        // go to the location where it thought the scout was.

        geometry_msgs::PoseStamped scout_stamped;
        scout_stamped.header = next_nav_goal_.header;
        // ##########################################################
        // ###########  H A C K  ################ //
        scout_stamped.pose.position.x = 40;
        scout_stamped.pose.position.y = 8;
        scout_stamped.pose.orientation.z = -1;
        // ROS_INFO_STREAM(scout_stamped);
        navigation_action_goal_.pose = scout_stamped;      // Position estimation is not perfect
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
    if(excavator_server_idle_ && !clods_in_scoop_)
    {
        std_msgs::Empty empty_message;
        park_hauler_pub_.publish(empty_message);
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
        digging_attempt_++;
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
            clods_in_scoop_ = true;
        }
    }
}

void ExcavatorStateMachine::dumpVolatile()
{
    if(excavator_server_idle_ && hauler_parked_)
    {
        operations::ExcavatorGoal goal;
        goal.task = START_UNLOADING; 
        
        // Should be tested with Endurance's parking code and 
        // These values should be tuned accordingly
        goal.target.x = 0.85; 
        goal.target.y = -2;
        goal.target.z = 0;

        excavator_arm_client_->sendGoal(goal);
        excavator_server_idle_ = false;
        clods_in_scoop_ = false;
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
            
            if(digging_attempt_ > digging_tries_)
            {
                ROS_INFO_STREAM("Done"<<digging_attempt_);
                std_msgs::Empty empty_msg;
                hauler_go_back_.publish(empty_msg);
                digging_attempt_ = 0;
                robot_state_ = INIT;
            }
        }
    }
}

void ExcavatorStateMachine::findScout()
{
    if(nav_server_idle_)
    {
        navigation_action_goal_.angular_velocity = -0.25; //
        navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;

        navigation_client_->sendGoal(navigation_action_goal_);
        nav_server_idle_ = false;
    }
    else if(!nav_server_idle_)  // Very bad
    {
        bool scout_found = updateScoutLocation();
        if (scout_found)
        {
            navigation_action_goal_.angular_velocity = 0.0; //
            navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;
            navigation_client_->sendGoal(navigation_action_goal_);

            updateScoutLocation();
            nav_server_idle_ = true;
            robot_state_ = PARK_AND_PUB;
        }
    }
}

bool ExcavatorStateMachine::updateScoutLocation()
{
    // ROBOT SHOULD BE ROTATING TO FIND SCOUT, IN CASE CAMERA WAS NOT ALIGNED
    const std::lock_guard<std::mutex> lock(g_objects_mutex); 

    perception::ObjectArray objects = g_objects_;
    perception::Object object;
    geometry_msgs::Point scout_loc;
    bool object_found = false;

    for(int i = 0; i < objects.number_of_objects; i++)
    {   
        // ROS_INFO_STREAM(object.label);
        object = objects.obj.at(i);
        if(object.label == OBJECT_DETECTION_SCOUT_CLASS && object.score>0.7)
        {
            // Store the object's location
            scout_loc = object.point;
            // ROS_INFO_STREAM(object.point);
            object_found = true;
            break;
        }
    }

    if(!object_found)
        return false;

    scout_loc_stamp_.header.frame_id = robot_name_ + SENSOR_BAR_GAZEBO; // Needs to be confirmed
    scout_loc_stamp_.header.stamp = ros::Time(0);
    scout_loc_stamp_.point = scout_loc;
    return true;
}