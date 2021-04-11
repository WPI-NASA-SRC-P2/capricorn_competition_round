#include <operations/scout_state_machine.h>
#include <operations/NavigationAction.h>

ScoutStateMachine::ScoutStateMachine(ros::NodeHandle nh, const std::string& robot_name):nh_(nh)
{
    robot_name_ = robot_name;
    navigation_client_ = new NavigationClient_(NAVIGATION_ACTIONLIB, true);
    resource_localiser_client_ = new ResourceLocaliserClient_(RESOURCE_LOCALISER_ACTIONLIB, true);

    volatile_sub_ = nh.subscribe("/" + robot_name + VOLATILE_SENSOR_TOPIC, 1000, &ScoutStateMachine::processVolatileMessage, this);

    // TODO TODO TODO: Replace (or switch based on debug flat) with real odometry topic
    robot_odom_sub_ = nh.subscribe(CAPRICORN_TOPIC + robot_name + CHEAT_ODOM_TOPIC, 1000, &ScoutStateMachine::processOdomMessage, this);

    excavator_ready_sub_ = nh.subscribe(CAPRICORN_TOPIC + EXCAVATOR_ARRIVED_TOPIC, 1000, &ScoutStateMachine::processExcavatorMessage, this);

    volatile_pub_ = nh.advertise<geometry_msgs::PoseStamped>(CAPRICORN_TOPIC + SCHEDULER_TOPIC + VOLATILE_LOCATION_TOPIC, 1000);
}

ScoutStateMachine::~ScoutStateMachine()
{
    delete navigation_client_;
    delete resource_localiser_client_;
}

void ScoutStateMachine::processVolatileMessage(const srcp2_msgs::VolSensorMsg::ConstPtr& vol_msg)
{
    if(vol_msg->vol_type != std::string("none"))
    {
        std::lock_guard<std::mutex> vol_flag_guard(vol_flag_mutex_);
        vol_detected_ = true;
    }
}

void ScoutStateMachine::processOdomMessage(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    std::lock_guard<std::mutex> pose_guard(robot_pose_mutex_);
    robot_pose_.header = odom_msg->header;
    robot_pose_.pose = odom_msg->pose.pose;
}

void ScoutStateMachine::processExcavatorMessage(const std_msgs::Empty::ConstPtr& odom_msg)
{
    std::lock_guard<std::mutex> excavator_ready_guard(excavator_ready_mutex_);

    excavator_ready_ = true;
}

geometry_msgs::PoseStamped ScoutStateMachine::getRobotPose()
{
    std::lock_guard<std::mutex> pose_guard(robot_pose_mutex_);
    return robot_pose_;
}

void ScoutStateMachine::startStateMachine()
{
    navigation_client_->waitForServer();

    while (ros::ok() && state_machine_continue_)
    {
        switch (robot_state_)
        {
        case LOCATOR_STATES::INIT:
        {
            // Call funtion to go to 'unexplored' region
            // For the demo purpose, call goToGoal(20,0) in map frame

            if(first_iter_)
            {
                printf("Init first iteration: Sending robot to unexplored region.\n");

                // Demo: unexplored region is a constant location
                geometry_msgs::PoseStamped unexplored;

                unexplored.header.frame_id = MAP;

                unexplored.pose.position.x = 40;
                unexplored.pose.position.y = 9;
                unexplored.pose.position.z = 0;

                unexplored.pose.orientation.w = 1;
                unexplored.pose.orientation.x = 0;
                unexplored.pose.orientation.y = 0;
                unexplored.pose.orientation.z = 0;

                navigation_action_goal_.pose = unexplored;
                navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

                // Send the goal to the navigation server
                navigation_client_->sendGoal(navigation_action_goal_);

                first_iter_ = false;
            }

            if(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                printf("Finished going to unexplored region. Beginning search.\n");

                first_iter_ = true;

                robot_state_ = LOCATOR_STATES::SEARCH;
            }
            break;
        }
        case LOCATOR_STATES::SEARCH:
        {
            // Call a function to execute search algorithm
            // For the demo, spiral motion
            // (update nav action to enum, call spiral)
            // Explicitly terminate nav goal once resource found (volatile topic)

            if(first_iter_)
            {
                printf("Beginning search for volatiles.\n");

                // Create a spiral goal. For now, there are no extra parameters passed along.
                navigation_action_goal_.drive_mode = NAV_TYPE::SPIRAL;

                // Send the spiral goal to the server
                navigation_client_->sendGoal(navigation_action_goal_);

                first_iter_ = false;

                // Lock the volatile flag, and reset the flag to ensure that previous volatile detections are reset
                std::lock_guard<std::mutex> vol_flag_lock(vol_flag_mutex_);
                vol_detected_ = false;
            }

            // Lock the vol flag mutex
            std::lock_guard<std::mutex> vol_flag_lock(vol_flag_mutex_);

            if(vol_detected_)
            {
                printf("Found a volatile! Localizing...\n");

                // Cancel the spiral goal
                navigation_client_->cancelGoal();

                first_iter_ = true;

                robot_state_ = LOCATOR_STATES::LOCATE;
            }

            break;
        }
        case LOCATOR_STATES::LOCATE:
        {
            // Call actionlib for resource_locate
            // ResourceLocator
            // actionlib call to resource localiser
            // resource localiser then interfaces with navigation

            if(first_iter_)
            {
                printf("Beginning localization.\n");

                resource_localiser_client_->sendGoal(resource_localiser_goal_);

                first_iter_ = false;
            }

            // What if error abort or something else?

            if(resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                printf("Resource localized.\n");

                first_iter_ = true;

                robot_state_ = LOCATOR_STATES::FOUND;
            }
            else if(resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::ABORTED)
            {
                ros::Duration(5.0).sleep();
                ROS_INFO("Retrying to localise the resource");
                resource_localiser_client_->sendGoal(resource_localiser_goal_);
            }
            break;
        }
        case LOCATOR_STATES::FOUND:
        {
            // Stop the robot, brake.
            // Publish fake message (for now) for excavator to come and get resource (publish PoseStamped)
            // Wait for a flag from excavator that it is ready to take over

            if(first_iter_)
            {
                printf("Publishing volatile location, then waiting for excavator response.\n");

                // Publish the most recent robot pose. This should be on top of the volatile.
                volatile_pub_.publish(getRobotPose());

                // Reset the first iteration flag
                first_iter_ = false;

                // Lock the flag mutex
                std::lock_guard<std::mutex> excavator_ready_guard(excavator_ready_mutex_);

                // Reset the excavator ready flag
                excavator_ready_ = false;
            }

            // Lock the flag mutex
            std::lock_guard<std::mutex> excavator_ready_guard(excavator_ready_mutex_);
        
            if(excavator_ready_)
            {
                printf("Excavator has arrived. Moving out of the way.\n");

                first_iter_ = true;

                robot_state_ = LOCATOR_STATES::MOVE_OUT;
            }
            break;
        }
        case LOCATOR_STATES::MOVE_OUT:
        {
            // Once flag is received (excavator topic, something), move some distance aside (+/- y)
            // For demo, stay in this state (don't return to search)

            if(first_iter_)
            {
                printf("Moving out of the way.\n");

                navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

                // Go 3 meters to the left of the current pose
                geometry_msgs::PoseStamped go_left;

                go_left.header.frame_id = robot_name_ + ROBOT_CHASSIS;

                go_left.pose.position.x = 0;
                go_left.pose.position.y = 3;
                go_left.pose.position.z = 0;

                go_left.pose.orientation.w = 1;
                go_left.pose.orientation.x = 0;
                go_left.pose.orientation.y = 0;
                go_left.pose.orientation.z = 0;

                navigation_action_goal_.pose = go_left;

                navigation_client_->sendGoal(navigation_action_goal_);

                first_iter_ = false;
            }

            if(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                printf("Done moving out of the way. DEMO: Staying in place, not returning to spiral.\n");
            }
            break;
        }
        case LOCATOR_STATES::RECHARGE:
        {
            // Mahimana's method to go back to the recharge station
            // Find and get to the recharge station

            printf("Recharging! Just kidding, nobody has bothered to implement me yet.\n");
            break;
        }
        case LOCATOR_STATES::WAIT_FOR_STATE_UPDATE:
        {
            // This means there is nothing to do. 
            // Just hang around for any update

            printf("Waiting for a state update.\n");
            break;
        }
        default:
        {
            ROS_ERROR_STREAM(robot_name_ + " state machine encountered unhandled state!");
            break;
        }
        }

        ros::spinOnce();
    }
}