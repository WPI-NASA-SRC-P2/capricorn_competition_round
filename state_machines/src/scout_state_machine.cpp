#include <state_machines/scout_state_machine.h>

ScoutStateMachine::ScoutStateMachine(ros::NodeHandle nh, const std::string &robot_name) : nh_(nh), robot_name_(robot_name)
{
    resource_localiser_client_ = new ResourceLocaliserClient_(RESOURCE_LOCALISER_ACTIONLIB, true);
    volatile_sub_ = nh.subscribe("/" + robot_name_ + VOLATILE_SENSOR_TOPIC, 1000, &ScoutStateMachine::volatileSensorCB, this);
}

ScoutStateMachine::~ScoutStateMachine()
{
    delete resource_localiser_client_;
}

bool ScoutStateMachine::startSearchingVolatile()
{
  // If starting spiral fails
  if(!resumeSearchingVolatile(true))
    return false;
  
  while(ros::ok())
  {
    if(near_volatile_)
        return true;
    ros::Duration(0.1).sleep();
  }
}

bool ScoutStateMachine::stopSearchingVolatile()
{
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
  return (resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ;
}

bool ScoutStateMachine::undockRobot()
{
  return true;
}


/**
 * @brief Callback for sensor topic
 * 
 * @param msg 
 */
void ScoutStateMachine::volatileSensorCB(const srcp2_msgs::VolSensorMsg::ConstPtr& msg)
{
	near_volatile_ = !(msg->distance_to == -1);
}
