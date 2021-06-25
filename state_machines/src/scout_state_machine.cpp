#include <state_machines/scout_state_machine.h>

ScoutStateMachine::ScoutStateMachine(ros::NodeHandle nh, const std::string &robot_name) : nh_(nh), robot_name_(robot_name)
{
  resource_localiser_client_ = new ResourceLocaliserClient_(RESOURCE_LOCALISER_ACTIONLIB, true);
  navigation_vision_client_ = new NavigationVisionClient(robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
  spiralClient_ = nh_.serviceClient<operations::Spiral>(SCOUT_SEARCH_SERVICE);

  volatile_sub_ = nh.subscribe("/" + robot_name_ + VOLATILE_SENSOR_TOPIC, 1000, &ScoutStateMachine::volatileSensorCB, this);

  waitForServerConnections();
}

ScoutStateMachine::~ScoutStateMachine()
{
  delete resource_localiser_client_;
  delete navigation_vision_client_;
}

void ScoutStateMachine::waitForServerConnections()
{
  resource_localiser_client_->waitForServer();
  navigation_vision_client_->waitForServer();
  spiralClient_.waitForExistence();
}

bool ScoutStateMachine::startSearchingVolatile()
{
  continue_spiral_ = true;
  // If starting spiral fails
  if (!resumeSearchingVolatile(true))
    return false;

  while (ros::ok() && continue_spiral_)
  {
    if (near_volatile_)
      return true;
    ros::Duration(0.1).sleep();
  }
}

bool ScoutStateMachine::stopSearchingVolatile()
{
  continue_spiral_ = false;
  return resumeSearchingVolatile(false);
}

bool ScoutStateMachine::resumeSearchingVolatile(const bool resume)
{
  operations::Spiral srv;
  srv.request.resume_spiral_motion = resume;
  return spiralClient_.call(srv);
}

bool ScoutStateMachine::locateVolatile()
{
  stopSearchingVolatile();

  operations::ResourceLocaliserGoal goal;
  resource_localiser_client_->sendGoal(goal);
  resource_localiser_client_->waitForResult();
  return (resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ScoutStateMachine::undockRobot()
{
  navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
  navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_UNDOCK;
  navigation_vision_client_->sendGoal(navigation_vision_goal_);
  navigation_vision_client_->waitForResult();
  return navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool ScoutStateMachine::undockRobot(const geometry_msgs::PoseStamped &POSE)
{
  // face the excavator in preparation for resetting odometry
  // navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
  // navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_CENTER;
  // navigation_vision_client_->sendGoal(navigation_vision_goal_);
  // navigation_vision_client_->waitForResult();

  // reset the odometry using the 180-degree rotated pose received from scheduler if facing excavator succeeds
  // if (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  resetOdometry(POSE);

  // finish the undocking process
  navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
  navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_UNDOCK;
  navigation_vision_client_->sendGoal(navigation_vision_goal_);
  navigation_vision_client_->waitForResult();
  return navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

/**
 * @brief Callback for sensor topic
 * 
 * @param msg 
 */
void ScoutStateMachine::volatileSensorCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg)
{
  near_volatile_ = !(msg->distance_to == -1);
}

/**
 * @brief Used for sending scout to a specific location using both 3D location and visual navigation (helper function for other states)
 * 
 * @param target_loc @param target_object
 */
bool ScoutStateMachine::goToLocObject(const geometry_msgs::PoseStamped &target_loc, std::string target_object)
{
  navigation_vision_goal_.desired_object_label = target_object;
  navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_NAV_AND_NAV_VISION;
  navigation_vision_goal_.goal_loc = target_loc;
  navigation_vision_client_->sendGoal(navigation_vision_goal_);
  navigation_vision_client_->waitForResult();
  return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ScoutStateMachine::resetOdometry(const geometry_msgs::PoseStamped &POSE)
{
  resetScoutOdometryClient_ = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
  maploc::ResetOdom srv;
  ROS_WARN_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << "Scout odometry has been reset");
  srv.request.ref_pose.header.frame_id = COMMON_NAMES::ODOM;
  srv.request.target_robot_name = COMMON_NAMES::SCOUT_1;
  srv.request.use_ground_truth = false;
  srv.request.ref_pose = POSE;

  return resetScoutOdometryClient_.call(srv);
}

// this should never be called ideally
bool ScoutStateMachine::resetOdometry()
{
  resetScoutOdometryClient_ = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
  maploc::ResetOdom srv;
  ROS_WARN_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << "Scout odometry has been reset");
  srv.request.ref_pose.header.frame_id = COMMON_NAMES::ODOM;
  srv.request.target_robot_name = COMMON_NAMES::SCOUT_1;
  srv.request.use_ground_truth = true;

  return resetScoutOdometryClient_.call(srv);
}

bool ScoutStateMachine::faceProcessingPlant()
{
  navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
  navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_CENTER;
  navigation_vision_client_->sendGoal(navigation_vision_goal_);
  navigation_vision_client_->waitForResult();

  return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}
bool ScoutStateMachine::syncOdometry(const geometry_msgs::PoseStamped &POSE)
{
  ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << "Syncing Scout odom");
  if (faceProcessingPlant())
  {
    return resetOdometry(POSE);
  }
  else
  {
    return false;
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << "Did not face processing plant yet!");
  }
}
