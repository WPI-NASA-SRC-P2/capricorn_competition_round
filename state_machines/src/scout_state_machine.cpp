#include <state_machines/scout_state_machine.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S C O U T   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Singleton implementation */
ScoutBaseState* ScoutBaseState::getScoutBaseState(ros::NodeHandle nh, const std::string &robot_name)
{
  static ScoutBaseState* currentObject_ = nullptr;

  // check if an object of this class already exists, if not create one
  if (currentObject_ == nullptr)
  {
    static ScoutBaseState obj(nh, robot_name);
    currentObject_ = &obj;
    ROS_WARN("ONCE CALLED");
  }
  return currentObject_;
}

ScoutBaseState::ScoutBaseState(ros::NodeHandle nh, const std::string &robot_name) : nh_(nh), robot_name_(robot_name)
{
  resource_localiser_client_ = new ResourceLocaliserClient_(RESOURCE_LOCALISER_ACTIONLIB, true);
  navigation_vision_client_ = new NavigationVisionClient(robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
  volatile_sub_ = nh_.subscribe("/" + robot_name_ + VOLATILE_SENSOR_TOPIC, 1000, &ScoutBaseState::volatileSensorCB, this);
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

