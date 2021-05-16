#include <operations/excavator_state_machine.h>

ExcavatorStateMachine::ExcavatorStateMachine(ros::NodeHandle nh, const std::string &robot_name) : nh_(nh), robot_name_(robot_name)
{
    navigation_client_ = new NavigationClient(NAVIGATION_ACTIONLIB, true);
    excavator_arm_client_ = new ExcavatorClient(EXCAVATOR_ACTIONLIB, true);
    navigation_vision_client_ = new NavigationVisionClient(robot_name + NAVIGATION_VISION_ACTIONLIB, true);
}

ExcavatorStateMachine::~ExcavatorStateMachine()
{
    delete navigation_client_;
    delete excavator_arm_client_;
    delete navigation_vision_client_;
}

bool ExcavatorStateMachine::goToLoc(const geometry_msgs::PoseStamped &loc)
{
    ROS_INFO("Going to given location");

    navigation_action_goal_.pose = loc;
    navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

    navigation_client_->sendGoal(navigation_action_goal_);
    navigation_client_->waitForResult();
    return (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ExcavatorStateMachine::goToScout()
{
    ROS_INFO("Going going");
    geometry_msgs::PoseStamped temp_msg;
    navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_SCOUT_CLASS;
    navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_REACH;
    navigation_vision_client_->sendGoal(navigation_vision_goal_);
    navigation_vision_client_->waitForResult();
    return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ExcavatorStateMachine::parkExcavator()
{
    ROS_INFO("Actual Goal Received");
    /////////////////////////////////////////////
    //// Hardcoded straigh walk for 1.5 meters ////
    /////////////////////////////////////////////

    geometry_msgs::PoseStamped hard_coded_pose;
    hard_coded_pose.header.frame_id = robot_name_ + ROBOT_BASE;
    hard_coded_pose.pose.position.x = 1.5;
    navigation_action_goal_.pose = hard_coded_pose; // Position estimation is not perfect
    navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

    navigation_client_->sendGoal(navigation_action_goal_);
    navigation_client_->waitForResult();
    return (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ExcavatorStateMachine::digVolatile()
{
    bool volatile_found = false;

    operations::ExcavatorGoal goal;
    goal.task = START_DIGGING;

    // These are the testing values, can be chagned
    // They start digging from the left of the robot
    goal.target.x = 1;
    goal.target.y = 0;
    goal.target.z = 0;

    excavator_arm_client_->sendGoal(goal);
    digging_attempt_++;
    excavator_arm_client_->waitForResult();
    return (excavator_arm_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ExcavatorStateMachine::dumpVolatile()
{
    operations::ExcavatorGoal goal;
    goal.task = START_UNLOADING;

    // Should be tested with Endurance's parking code and
    // These values should be tuned accordingly
    goal.target.x = 0.85;
    goal.target.y = -2;
    goal.target.z = 0;

    excavator_arm_client_->sendGoal(goal);
    excavator_arm_client_->waitForResult();
    return true;
}

bool ExcavatorStateMachine::digAndDumpVolatile()
{
    bool volatile_found = false;
    while (digVolatile())
    {
        volatile_found = true;
        dumpVolatile();
    }
    goToDefaultArmPosition();
    return volatile_found;
}

bool ExcavatorStateMachine::goToDefaultArmPosition()
{
    operations::ExcavatorGoal goal;
    goal.task = GO_TO_DEFAULT;

    excavator_arm_client_->sendGoal(goal);
    excavator_arm_client_->waitForResult();
    return true;
}