#include<operations/excavator_state_machine.h>

ExcavatorStateMachine::ExcavatorStateMachine(ros::NodeHandle nh, const std::string& robot_name):nh_(nh)
{
    robot_name_ = robot_name;
    
    navigation_client_ = new NavigationClient(NAVIGATION_ACTIONLIB, true);
    excavator_arm_client_ = new ExcavatorClient(EXCAVATOR_ACTIONLIB, true);
    navigation_vision_client_ = new NavigationVisionClient(robot_name + NAVIGATION_VISION_ACTIONLIB, true);
    
    // SHOULD BE TAKEN CARE OF BY NAMESPACE
    // ALL THE PREFIXES SHOULD BE REMOVED 
    // (/capricorn/robot_name_1) should be set via namespace, not from here
    objects_sub_ = nh.subscribe("/" + CAPRICORN_TOPIC + robot_name + OBJECT_DETECTION_OBJECTS_TOPIC, 1, &ExcavatorStateMachine::objectsCallback, this);
    lookout_pos_sub_ = nh_.subscribe("/" + CAPRICORN_TOPIC + "/" + robot_name_ + LOOKOUT_LOCATION_TOPIC, 1000, &ExcavatorStateMachine::lookoutLocCB, this);
    sub_scout_vol_location_ = nh_.subscribe("/" + CAPRICORN_TOPIC + "/" + robot_name_ + VOLATILE_LOCATION_TOPIC, 1000, &ExcavatorStateMachine::scoutVolLocCB, this);
    hauler_parked_sub_ = nh.subscribe("/" + CAPRICORN_TOPIC + SCHEDULER_TOPIC + HAULER_PARKED_TOPIC, 1, &ExcavatorStateMachine::haulerParkedCB, this);
    
    return_hauler_pub_ = nh.advertise<std_msgs::Empty>("/" + CAPRICORN_TOPIC  + HAULER_FILLED, 1000);
    park_hauler_pub_ = nh.advertise<std_msgs::Empty>("/" + CAPRICORN_TOPIC + PARK_HAULER, 1000);
    excavator_ready_pub_ = nh.advertise<std_msgs::Empty>("/" + CAPRICORN_TOPIC + EXCAVATOR_ARRIVED_TOPIC, 1000);
}

ExcavatorStateMachine::~ExcavatorStateMachine()
{
    delete navigation_client_;
    delete excavator_arm_client_;
    delete navigation_vision_client_;
}

void ExcavatorStateMachine::haulerParkedCB(std_msgs::Empty msg)
{
    const std::lock_guard<std::mutex> lock(hauler_parked_mutex); 
    hauler_parked_ = true;
}

void ExcavatorStateMachine::lookoutLocCB(const geometry_msgs::PoseStamped &msg)
{
    ROS_INFO_STREAM("CB");
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
}

void ExcavatorStateMachine::initState()
{
    if(lookout_loc_received_)
    {   
        robot_state_ = STATE_MACHINE_TASK::EXCAVATOR_KEEP_LOOKOUT;
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
            robot_state_ = STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT;
    }
}

void ExcavatorStateMachine::goToScout()
{
  ////////////////////////////////////
  //////// MAHI CHANGE HERE //////////
  ////////////////////////////////////
    if(nav_server_idle_)
    {
        ROS_INFO("Going going");
        geometry_msgs::PoseStamped temp_msg;
        navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_SCOUT_CLASS;
        navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_REACH;
        navigation_vision_client_->sendGoal(navigation_vision_goal_);
        nav_server_idle_ = false;
    }
    else if(navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("Reached to the scout");
        robot_state_ = STATE_MACHINE_TASK::EXCAVATOR_PARK_AND_PUB;
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
        /////////////////////////////////////////////
        //// Hardcoded straigh walk for 1.5 meters ////
        /////////////////////////////////////////////
        
        geometry_msgs::PoseStamped hard_coded_pose;
        hard_coded_pose.header.frame_id = robot_name_ + ROBOT_BASE;
        hard_coded_pose.pose.position.x = 1.5;
        navigation_action_goal_.pose = hard_coded_pose;      // Position estimation is not perfect
        navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

        navigation_client_->sendGoal(navigation_action_goal_);
        nav_server_idle_ = false;
    }
    else if(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("GOAL FINISHED");
        robot_state_ = STATE_MACHINE_TASK::EXCAVATOR_DIG_VOLATILE;
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
            robot_state_ = STATE_MACHINE_TASK::EXCAVATOR_DUMP_VOLATILE;
            clods_in_scoop_ = true;
        }
    }
}

void ExcavatorStateMachine::dumpVolatile()
{
    const std::lock_guard<std::mutex> lock(hauler_parked_mutex); 
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
            robot_state_ = STATE_MACHINE_TASK::EXCAVATOR_DIG_VOLATILE;
            
            if(digging_attempt_ > DIGGING_TRIES_)
            {
                ROS_INFO_STREAM("Done"<<digging_attempt_);
                std_msgs::Empty empty_msg;
                return_hauler_pub_.publish(empty_msg);
                digging_attempt_ = 0;
                robot_state_ = STATE_MACHINE_TASK::EXCAVATOR_INIT;
            }
        }
    }
}

void ExcavatorStateMachine::findScout()
{
    if(nav_server_idle_)
    {
        navigation_action_goal_.angular_velocity = -0.25;
        navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;

        navigation_client_->sendGoal(navigation_action_goal_);
        nav_server_idle_ = false;
    }
    else if(!nav_server_idle_)  // Very bad
    {
        bool scout_found = updateScoutLocation();
        if (scout_found)
        {   
            // Stops excavator from spinning. 
            navigation_action_goal_.angular_velocity = 0.0; 
            navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;
            navigation_client_->sendGoal(navigation_action_goal_);

            updateScoutLocation();
            nav_server_idle_ = true;
            robot_state_ = STATE_MACHINE_TASK::EXCAVATOR_PARK_AND_PUB;
        }
    }
}

bool ExcavatorStateMachine::updateScoutLocation()
{
    // ROBOT SHOULD BE ROTATING TO FIND SCOUT, IN CASE CAMERA WAS NOT ALIGNED
    const std::lock_guard<std::mutex> lock(g_objects_mutex); 

    perception::ObjectArray objects = g_objects_;
    perception::Object object;
    geometry_msgs::PoseStamped scout_loc;
    bool object_found = false;

    for(int i = 0; i < objects.number_of_objects; i++)
    {   
        // ROS_INFO_STREAM(object.label);
        object = objects.obj.at(i);
        if(object.label == OBJECT_DETECTION_SCOUT_CLASS && object.score>0.7)
        {
            // Store the object's location
            scout_loc = object.point;
            scout_loc.pose.position.z -= 2;
            // ROS_INFO_STREAM(object.point);
            object_found = true;
            break;
        }
    }

    if(!object_found)
        return false;

    scout_loc_stamp_.header.frame_id = robot_name_ + SENSOR_BAR_GAZEBO; // Needs to be confirmed
    scout_loc_stamp_.header.stamp = ros::Time(0);
    scout_loc_stamp_.point = scout_loc.pose.position;
    return true;
}

bool checkTask(STATE_MACHINE_TASK task)
{
    return EXCAVATOR_TASKS.find(task) != EXCAVATOR_TASKS.end();
}


/**
 * @brief Function which gets executed when any goal is received to actionlib
 * 
 * @param goal for action lib
 * @param as variable to send feedback
 * @param sm ExcavatorStateMachine object
 */
void execute(const operations::ExcavatorStateMachineTaskGoalConstPtr& goal, SM_SERVER* as, ExcavatorStateMachine* sm)
{
    operations::ExcavatorStateMachineTaskResult result;
    // Waiting for the servers to start
    sm->navigation_client_->waitForServer();
    sm->excavator_arm_client_->waitForServer();
    sm->navigation_vision_client_->waitForServer(); //Not being used currently
    ROS_WARN("Servers started");

    sm->robot_state_ = (STATE_MACHINE_TASK)goal->task;

    if(!checkTask(sm->robot_state_))
    {
        // the class is not valid, send the appropriate result
        result.result = COMMON_NAMES::COMMON_RESULT::INVALID_GOAL;
        as->setAborted(result, "Invalid Task");
        ROS_INFO_STREAM("Invalid Task - "<<g_robot_name<<" State Machine");
        return;
    }

    while (ros::ok() && sm->state_machine_continue_)
    {   
        switch (sm->robot_state_)
        {
        case STATE_MACHINE_TASK::EXCAVATOR_INIT:
            sm->initState();
            break;
        case STATE_MACHINE_TASK::EXCAVATOR_KEEP_LOOKOUT:
            sm->goToLookout();
            break;
        case STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT:
            sm->goToScout();
            break;
        case STATE_MACHINE_TASK::EXCAVATOR_FIND_SCOUT:
            sm->findScout();
            break;
        case STATE_MACHINE_TASK::EXCAVATOR_PARK_AND_PUB:
            sm->parkExcavator();
            break;
        case STATE_MACHINE_TASK::EXCAVATOR_DIG_VOLATILE:
            sm->digVolatile();
            break;
        case STATE_MACHINE_TASK::EXCAVATOR_DUMP_VOLATILE:
            sm->dumpVolatile();
            break;
        case STATE_MACHINE_TASK::EXCAVATOR_NEXT_QUE_TASK:
            sm->robot_state_ = STATE_MACHINE_TASK::EXCAVATOR_INIT; 
            // this is temporary, ideally it should publish to schedular
            // that it is available, and schedular should send the next
            // Goal to follow. 
            break;

        default:
            ROS_ERROR_STREAM(sm->robot_name_ + " state machine encountered unhandled state!");
            break;
        }
        // sleep for some time
        ros::Duration(sm->SLEEP_TIME).sleep();
        ros::spinOnce();

        // call function to update the states (Because most of these things are actionlibs
        // So they will be running in a seperate node)
        // This function should keep track of the executed actionlibs, and their current states
        // If succeeded
    }
}

/**
 * @brief Function called when the goal is cancelled
 * 
 */
void cancelGoal()
{
    ROS_INFO_STREAM("Cancelling "<<g_robot_name<<"  Vision Goal");
}

int main(int argc, char* argv[])
{
    if(argc != 2 && argc != 4)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");

        return -1;
    }

    g_robot_name = argv[1];

    ros::init(argc, argv, g_robot_name + "_sm");
    ros::NodeHandle nh;

    ExcavatorStateMachine* sm = new ExcavatorStateMachine(nh, g_robot_name);

    SM_SERVER server(nh, g_robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, boost::bind(&execute, _1, &server, sm), false);
	server.registerPreemptCallback(&cancelGoal);
    server.start();

    ROS_INFO("Started Excavator State Machine Actionlib Server");
    ros::spin();

    ROS_WARN("Excavator state machine died!\n");

    return 0;
}