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

bool ScoutStateMachine::solarRecharge()
{   
    // asumme that the robot isn't in a state
    // change the positon // rotating robot
    // 
    // if(naviagtion_client_goal.forward_velocity == 0){
    //   // rotate the robot
    //   const srcp2_msgs::SystemMonitorMsg &msg;
    //   if(msg.solar_ok)
    //   {
    //     //change to power save mode
    //   }
    //   else()
    //   {
    //     //rotate the robot
    //     while(!msg.solar_ok)
    //     {
    //       //roTate incrementally
    //     }
    //   }
    //   //naviagtion_client_goal.forward_velocity = 
    // }
    // //naviagtion_client_goal.forward_velocity = 

    naviagtion_client_->sendGoal();
    navigation_client_->waitForResult();
    //return (navigation_client_->getState() == actionLib::SimpleClientGoalState::SUCCEEDED);
    return true;
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
