#include <state_machines/scout_state_machine.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S C O U T   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ScoutBaseState::ScoutBaseState(ros::NodeHandle nh, const std::string &robot_name) : nh_(nh), robot_name_(robot_name)
{
  ROS_INFO("Base State Constructor");
  resource_localiser_client_ = new ResourceLocaliserClient_(RESOURCE_LOCALISER_ACTIONLIB, true);
  navigation_vision_client_ = new NavigationVisionClient(robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);

  volatile_sub_ = nh_.subscribe("/" + robot_name_ + VOLATILE_SENSOR_TOPIC, 1000, &ScoutBaseState::volatileSensorCB, this);
  objects_sub_ = nh_.subscribe(CAPRICORN_TOPIC + robot_name_ + OBJECT_DETECTION_OBJECTS_TOPIC, 1, &ScoutBaseState::objectsCallback, this);
}

ScoutBaseState::~ScoutBaseState()
{
  delete resource_localiser_client_;
  delete navigation_vision_client_;
}

/**
 * @brief Callback for sensor topic
 * 
 * @param msg 
 */
void ScoutBaseState::volatileSensorCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg)
{
  near_volatile_ = !(msg->distance_to == -1);
}

void ScoutBaseState::objectsCallback(const perception::ObjectArray::ConstPtr objs)
{
  const std::lock_guard<std::mutex> lock(objects_mutex_);
  vision_objects_ = objs;
  ROS_INFO("YAAYYYYY");
  objects_msg_received_ = true;
}

bool ScoutBaseState::entryPoint()
{
  return true;
}

bool ScoutBaseState::exec()
{
  return true;
}

bool ScoutBaseState::exitPoint()
{
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// U N D O C K   S T A T E  ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

Undock::Undock(ros::NodeHandle nh, const std::string &robot_name):ScoutBaseState(nh,robot_name)
{
}

bool Undock::entryPoint()
{
  bool result = false;
  while(!objects_msg_received_ && ros::ok())
  {
    ROS_INFO("Waiting to receive image");
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  if (vision_objects_->number_of_objects > 0)
  {
    const std::lock_guard<std::mutex> lock(objects_mutex_);
    ROS_INFO("Checking Obstacles");
    float direction = checkObstacle(vision_objects_->obj);
    result = (abs(direction) > 0.0);
  }
  return result;
}

bool Undock::exec()
{
  ROS_INFO("Undock Exec");
  operations::NavigationVisionGoal navigation_vision_goal;
  navigation_vision_goal.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
  navigation_vision_goal.mode = COMMON_NAMES::NAV_VISION_TYPE::V_UNDOCK;
  navigation_vision_client_->sendGoal(navigation_vision_goal);
  navigation_vision_client_->waitForResult();
  return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool Undock::exitPoint()
{
  ROS_INFO("Undock Exit Point");
  navigation_vision_client_->cancelGoal();
  return true;
}
