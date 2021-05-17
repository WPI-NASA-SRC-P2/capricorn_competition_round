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
    ROS_INFO_STREAM(robot_name_ << " State Machine: Going to given location");
    navigation_action_goal_.pose = loc;
    navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;

    navigation_client_->sendGoal(navigation_action_goal_);
    navigation_client_->waitForResult();
    return (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerStateMachine::followExcavator()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Following Excavator");
    navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
    navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_FOLLOW;
    navigation_vision_client_->sendGoal(navigation_vision_goal_);
    navigation_vision_client_->waitForResult();
    return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerStateMachine::parkAtExcavator(std::string excavator_name)
{
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
    return (hauler_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerStateMachine::undockExcavator()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Undocking from Excavator");
    return false;
}

bool HaulerStateMachine::undockHopper()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Undocking from Hopper");
    return false;
}

bool HaulerStateMachine::dumpVolatileToProcPlant()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Dumping Volatile to Processing Plant (High Level Goal)");
    return (goToProcPlant() && parkAtHopper() && dumpVolatile() && undockHopper());
}

bool HaulerStateMachine::goBackToExcavator()
{
    ROS_INFO_STREAM(robot_name_ << " State Machine: Going Back to Excavator (High Level Goal)");
    return false;
}