#include <state_machines/hauler_state_machine.h>

HaulerStateMachine::HaulerStateMachine(ros::NodeHandle nh, const std::string &robot_name) : nh_(nh), robot_name_(robot_name)
{
    // Actionlib initializations
    navigation_client_ = new NavigationClient(NAVIGATION_ACTIONLIB, true);
    hauler_client_ = new HaulerClient(HAULER_ACTIONLIB, true);
    navigation_vision_client_ = new NavigationVisionClient(robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
    park_robot_client_ = new ParkRobotClient(robot_name + COMMON_NAMES::PARK_HAULER_ACTIONLIB, true);
}

HaulerStateMachine::~HaulerStateMachine()
{
    delete navigation_client_;
    delete hauler_client_;
    delete navigation_vision_client_;
    delete park_robot_client_;
}

bool HaulerStateMachine::goToLoc(const geometry_msgs::PoseStamped &loc)
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Going Back to Goal (High Level Vision Goal)");
    navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_OBS_GOTO_GOAL;
    navigation_vision_goal_.goal_loc = loc;
    navigation_vision_client_->sendGoal(navigation_vision_goal_);
    navigation_vision_client_->waitForResult();
    return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerStateMachine::followExcavator()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Following Excavator");
    navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
    navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_FOLLOW;
    navigation_vision_client_->sendGoal(navigation_vision_goal_);
    // navigation_vision_client_->waitForResult();
    return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerStateMachine::parkAtExcavator(std::string excavator_name)
{
    // try to face excavator first: Just a HOTFIX, might need to remove it later as it doesnt take care of the fact that the excavator could be in a crater
    // navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
    // navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_CENTER;
    // navigation_vision_client_->sendGoal(navigation_vision_goal_);
    // navigation_vision_client_->waitForResult();
    // do we need a duration check?

    ROS_INFO_STREAM(robot_name_ << " State Machine: Parking at Excavator");
    park_robot_goal_.hopper_or_excavator = excavator_name;
    park_robot_client_->sendGoal(park_robot_goal_);
    park_robot_client_->waitForResult();
    return (park_robot_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerStateMachine::goToProcPlant()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Going to Processing Plant");
    navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
    navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_REACH;
    navigation_vision_client_->sendGoal(navigation_vision_goal_);
    navigation_vision_client_->waitForResult();
    return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerStateMachine::parkAtHopper()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Parking to hopper");
    park_robot_goal_.hopper_or_excavator = OBJECT_DETECTION_HOPPER_CLASS;
    park_robot_client_->sendGoal(park_robot_goal_);
    park_robot_client_->waitForResult();
    return (park_robot_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerStateMachine::dumpVolatile()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Dumping Volatile");
    hauler_goal_.desired_state = true;
    hauler_client_->sendGoal(hauler_goal_);
    hauler_client_->waitForResult();
    return (hauler_client_->getState().isDone());
}

bool HaulerStateMachine::undockExcavator()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Undocking from Excavator");
    navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
    navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_UNDOCK;
    navigation_vision_client_->sendGoal(navigation_vision_goal_);
    navigation_vision_client_->waitForResult();
    return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerStateMachine::undockHopper()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Undocking from Excavator");
    navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_HOPPER_CLASS;
    navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_UNDOCK;
    navigation_vision_client_->sendGoal(navigation_vision_goal_);
    navigation_vision_client_->waitForResult();
    return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerStateMachine::dumpVolatileToProcPlant()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Dumping Volatile to Processing Plant (High Level Goal)");
    return (undockExcavator() && goToProcPlant() && parkAtHopper() && dumpVolatile() && resetOdometry() && undockHopper());
}

bool HaulerStateMachine::resetOdometryAtHopper()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Resetting Odometry at Hopper");
    return (/*undockExcavator() && */ faceProcessingPlant() && goToProcPlant() && parkAtHopper() && resetOdometry() && undockHopper());
}

bool HaulerStateMachine::goBackToExcavator(const geometry_msgs::PoseStamped &loc)
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Going Back to Excavator (High Level Goal)");
    navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_OBS_GOTO_GOAL;
    navigation_action_goal_.pose = loc;
    navigation_vision_client_->sendGoal(navigation_vision_goal_);
    navigation_vision_client_->waitForResult();
    return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerStateMachine::resetOdometry()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Reseting odom with GT");
    resetHaulerOdometryClient_ = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
    maploc::ResetOdom srv;
    // srv.request.ref_pose.header.frame_id = COMMON_NAMES::ODOM;
    // srv.request.ref_pose.pose.orientation.w = 1; //Need to check this out, but included this because having 'w' as non-zero gives a NAN value somewhere in some processing and odometry reset fails somehwere in the source files of rtabmap
    srv.request.target_robot_name = COMMON_NAMES::HAULER_1;
    srv.request.use_ground_truth = true;
    return resetHaulerOdometryClient_.call(srv);
}

bool HaulerStateMachine::faceProcessingPlant()
{
    navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
    navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_CENTER;
    navigation_vision_client_->sendGoal(navigation_vision_goal_);
    navigation_vision_client_->waitForResult();

    return navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool HaulerStateMachine::goToLocObject(const geometry_msgs::PoseStamped &target_loc, std::string target_object)
{
    navigation_vision_goal_.desired_object_label = target_object;
    navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_NAV_AND_NAV_VISION;
    navigation_vision_goal_.goal_loc = target_loc;
    navigation_vision_client_->sendGoal(navigation_vision_goal_);
    navigation_vision_client_->waitForResult();
    return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}