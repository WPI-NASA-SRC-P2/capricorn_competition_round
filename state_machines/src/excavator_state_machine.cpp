#include <state_machines/excavator_state_machine.h>

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
    ROS_INFO_STREAM(robot_name_ << " State Machine: Going to given location");
    navigation_action_goal_.pose = loc;
    navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

    navigation_client_->sendGoal(navigation_action_goal_);
    navigation_client_->waitForResult();
    return (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ExcavatorStateMachine::goToScout()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Going to Scout");
    geometry_msgs::PoseStamped temp_msg;
    navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_SCOUT_CLASS;
    navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_REACH;
    navigation_vision_client_->sendGoal(navigation_vision_goal_);
    navigation_vision_client_->waitForResult();
    return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ExcavatorStateMachine::parkExcavator()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Parking to Excavator");
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
    ROS_INFO_STREAM(robot_name_ << " State Machine: Digging Volatile");
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
    ROS_INFO_STREAM(robot_name_ << " State Machine: Dumping Volatile");
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
    ROS_INFO_STREAM(robot_name_ << " State Machine: Excavating Volatile");
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
    ROS_INFO_STREAM(robot_name_ << " State Machine: Going to Default Excavator Arm Position");
    operations::ExcavatorGoal goal;
    goal.task = GO_TO_DEFAULT;

    excavator_arm_client_->sendGoal(goal);
    excavator_arm_client_->waitForResult();
    return true;
}

bool ExcavatorStateMachine::resetOdometry(const geometry_msgs::Pose& POSE)
{
    resetExcavatorOdometryClient_ = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
    maploc::ResetOdom srv;
    srv.request.ref_pose.header.frame_id = COMMON_NAMES::ODOM;
    srv.request.target_robot_name = COMMON_NAMES::EXCAVATOR_1;
    srv.request.ref_pose.pose = POSE;

    return resetExcavatorOdometryClient_.call(srv);
}

bool ExcavatorStateMachine::resetOdometry()
{
    resetExcavatorOdometryClient_ = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
    maploc::ResetOdom srv;
    srv.request.ref_pose.header.frame_id = COMMON_NAMES::ODOM;
    srv.request.target_robot_name = COMMON_NAMES::EXCAVATOR_1;
    srv.request.use_ground_truth = true;

    return resetExcavatorOdometryClient_.call(srv);
}