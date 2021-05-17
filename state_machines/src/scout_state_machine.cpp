#include <state_machines/scout_state_machine.h>

ScoutStateMachine::ScoutStateMachine(ros::NodeHandle nh, const std::string &robot_name) : nh_(nh), robot_name_(robot_name)
{
    navigation_client_ = new NavigationClient(NAVIGATION_ACTIONLIB, true);
    navigation_vision_client_ = new NavigationVisionClient(robot_name + NAVIGATION_VISION_ACTIONLIB, true);
    resource_localiser_client_ = new ResourceLocaliserClient_(RESOURCE_LOCALISER_ACTIONLIB, true);
}

ScoutStateMachine::~ScoutStateMachine()
{
    delete navigation_client_;
    delete navigation_vision_client_;
    delete resource_localiser_client_;
}

bool ScoutStateMachine::startSearchingVolatile()
{
  return resumeSearchingVolatile(true);
}

bool ScoutStateMachine::stopSearchingVolatile()
{
  return resumeSearchingVolatile(false);
}

bool ScoutStateMachine::resumeSearchingVolatile(const bool resume)
{
  spiralClient_ = nh_.serviceClient<operations::Spiral>(SCOUT_SEARCH_SERVICE);
  ros::Duration(1).sleep();
  ROS_INFO_STREAM("Spiraling");
  operations::Spiral srv;
  srv.request.resume_spiral_motion = false;
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