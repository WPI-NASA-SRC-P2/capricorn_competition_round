#include <state_machines/scout_state_machine.h>

ScoutStateMachine::ScoutStateMachine(ros::NodeHandle nh, const std::string &robot_name) : nh_(nh), robot_name_(robot_name)
{
    resource_localiser_client_ = new ResourceLocaliserClient_(RESOURCE_LOCALISER_ACTIONLIB, true);
}

ScoutStateMachine::~ScoutStateMachine()
{
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