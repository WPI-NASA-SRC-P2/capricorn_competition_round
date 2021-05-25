#include <state_machines/scout_state_machine.h>

ScoutStateMachine::ScoutStateMachine(ros::NodeHandle nh, const std::string &robot_name) : nh_(nh), robot_name_(robot_name)
{
  resource_localiser_client_ = new ResourceLocaliserClient_(RESOURCE_LOCALISER_ACTIONLIB, true);
  navigation_vision_client_ = new NavigationVisionClient(robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
  volatile_sub_ = nh.subscribe("/" + robot_name_ + VOLATILE_SENSOR_TOPIC, 1000, &ScoutStateMachine::volatileSensorCB, this);
}

ScoutStateMachine::~ScoutStateMachine()
{
  delete resource_localiser_client_;
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
  spiralClient_ = nh_.serviceClient<operations::Spiral>(SCOUT_SEARCH_SERVICE);
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
  return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
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

bool ScoutStateMachine::resetOdometry(const geometry_msgs::PoseStamped &POSE)
{
  resetScoutOdometryClient_ = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
  maploc::ResetOdom srv;
  srv.request.ref_pose.header.frame_id = COMMON_NAMES::ODOM;
  srv.request.target_robot_name = COMMON_NAMES::SCOUT_1;
  srv.request.use_ground_truth = false;
  srv.request.ref_pose = POSE;

  return resetScoutOdometryClient_.call(srv);
}

bool ScoutStateMachine::resetOdometry()
{
  resetScoutOdometryClient_ = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
  maploc::ResetOdom srv;
  srv.request.ref_pose.header.frame_id = COMMON_NAMES::ODOM;
  srv.request.target_robot_name = COMMON_NAMES::SCOUT_1;
  srv.request.use_ground_truth = true;

  return resetScoutOdometryClient_.call(srv);
}

bool ScoutStateMachine::syncOdometry(const geometry_msgs::PoseStamped &POSE)
{
  ROS_INFO("Syncing Scout odom");
  navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
  navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_CENTER;
  navigation_vision_client_->sendGoal(navigation_vision_goal_);
  navigation_vision_client_->waitForResult();
  if (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    return resetOdometry(POSE);
  }
}
