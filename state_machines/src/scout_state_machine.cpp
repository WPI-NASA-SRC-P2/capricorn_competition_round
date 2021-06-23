#include <state_machines/scout_state_machine.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S C O U T   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ScoutBaseState::ScoutBaseState(ros::NodeHandle nh, const std::string &robot_name) : nh_(nh), robot_name_(robot_name)
{
  resource_localiser_client_ = new ResourceLocaliserClient_(RESOURCE_LOCALISER_ACTIONLIB, true);
  navigation_vision_client_ = new NavigationVisionClient(robot_name + "/" + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
  navigation_client_ = new NavigationClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);

  spiralClient_ = nh_.serviceClient<operations::Spiral>(SCOUT_SEARCH_SERVICE);

  volatile_sub_ = nh_.subscribe("/" + robot_name_ + VOLATILE_SENSOR_TOPIC, 1000, &ScoutBaseState::volatileSensorCB, this);
  objects_sub_ = nh_.subscribe(CAPRICORN_TOPIC + robot_name_ + OBJECT_DETECTION_OBJECTS_TOPIC, 1, &ScoutBaseState::objectsCallback, this);
}

ScoutBaseState::~ScoutBaseState()
{
  delete resource_localiser_client_;
  delete navigation_vision_client_;
  delete navigation_client_;
}

/**
 * @brief Callback for sensor topic
 * 
 * @param msg 
 */
void ScoutBaseState::volatileSensorCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg)
{
  near_volatile_ = !(msg->distance_to == -1);
  new_volatile_msg_ = true;
}

void ScoutBaseState::objectsCallback(const perception::ObjectArray::ConstPtr objs)
{
  const std::lock_guard<std::mutex> lock(objects_mutex_);
  vision_objects_ = objs;
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
  ROS_INFO("Undock Entrypoint");
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

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S E A R C H   S T A T E  ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

Search::Search(ros::NodeHandle nh, const std::string &robot_name):ScoutBaseState(nh,robot_name)
{
}

bool Search::entryPoint()
{
  ROS_INFO("Searching Entry point");
  bool result = true;
  while(!objects_msg_received_ && ros::ok())
  {
    ROS_INFO("Waiting to receive image");
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  if (vision_objects_->number_of_objects > 0)
  {
    const std::lock_guard<std::mutex> lock(objects_mutex_);
    float direction = checkObstacle(vision_objects_->obj);
    result = !(abs(direction) > 0.0);
  }
  return result;
}

bool Search::exec()
{
  return resumeSearchingVolatile(true);
}

bool Search::exitPoint()
{
  return resumeSearchingVolatile(false);
}

bool Search::resumeSearchingVolatile(bool resume)
{
  operations::Spiral srv;
  srv.request.resume_spiral_motion = resume;
  return spiralClient_.call(srv);
}



///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////  V O L A T I L E   L O C A T I O N  S T A T E  /////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

Locate::Locate(ros::NodeHandle nh, const std::string &robot_name):ScoutBaseState(nh,robot_name)
{
}

bool Locate::entryPoint()
{
  ROS_INFO("Resource Localizing entrypoint");
  while(!new_volatile_msg_ && ros::ok())
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  new_volatile_msg_ = false;
  return near_volatile_;
}

bool Locate::exec()
{
  ROS_INFO("Resource Localizing exec");
  operations::ResourceLocaliserGoal goal;
  resource_localiser_client_->sendGoal(goal);
  resource_localiser_client_->waitForResult();
  return (resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool Locate::exitPoint()
{
  ROS_INFO("Resource Localizing exitpoint");
  resource_localiser_client_->cancelGoal();
}
