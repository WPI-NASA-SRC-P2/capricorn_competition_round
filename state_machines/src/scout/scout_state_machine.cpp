#include <state_machines/scout_state_machine.h>
#include <iostream>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S C O U T   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ScoutState::ScoutState(uint32_t un_id) :
    State(un_id, ToString("mystate_", un_id))
{
  robot_name_ = COMMON_NAMES::SCOUT_1_NAME;
  resource_localiser_client_ = new ResourceLocaliserClient_(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + RESOURCE_LOCALISER_ACTIONLIB, true);
  /** @todo: FIX NAVIGATIONVISIONCLIENT TO BE CORRECT TOPIC */
  navigation_vision_client_ = new NavigationVisionClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + robot_name_ + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
  navigation_client_ = new NavigationClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);
  ROS_INFO("Waiting for the scout action servers...");
  navigation_vision_client_->waitForServer();
  navigation_client_->waitForServer();
  resource_localiser_client_->waitForServer();
  
  ROS_INFO("All scout action servers started!");

  // spiral client for spiral motion
  ROS_INFO("waiting for spiral client");
  spiralClient_ = nh_.serviceClient<operations::Spiral>(CAPRICORN_TOPIC + robot_name_ + "/" + SCOUT_SEARCH_SERVICE);
  spiralClient_.waitForExistence();
  ROS_INFO("Spiral client started");
  
  volatile_sub_ = nh_.subscribe("/" + robot_name_ + VOLATILE_SENSOR_TOPIC, 1000, &ScoutState::volatileSensorCB, this);
  objects_sub_ = nh_.subscribe(CAPRICORN_TOPIC + robot_name_ + OBJECT_DETECTION_OBJECTS_TOPIC, 1, &ScoutState::objectsCallback, this);
}

ScoutState::~ScoutState()
{
  delete resource_localiser_client_;
  delete navigation_vision_client_;
  delete navigation_client_;
}

/****************************************/
/****************************************/


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// C A L L B A C K S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Callback for sensor topic
 * 
 * @param msg 
 */
void ScoutState::volatileSensorCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg)
{
  near_volatile_ = !(msg->distance_to == -1);
  new_volatile_msg_ = true;
}

void ScoutState::objectsCallback(const perception::ObjectArray::ConstPtr objs)
{
  const std::lock_guard<std::mutex> lock(objects_mutex_);
  vision_objects_ = objs;
  objects_msg_received_ = true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// U N D O C K   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Undock::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   first_ = true;
   ROS_INFO("entrypoint of undock");
}

State& Undock::transition()
{
   if(!(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)) {
      ROS_INFO("remaining in undock");
      return *this;
   }
   ROS_INFO("transitioning out of undock");
   return getState(SCOUT_SEARCH_VOLATILE); // should be SCOUT_SEARCH_VOLATILE
}

void Undock::step()
{
   if(first_)
   {
      ROS_INFO_STREAM(robot_name_ << " State Machine: Undocking from volatile");
      geometry_msgs::PoseStamped pt;
      pt.header.frame_id = robot_name_ + ROBOT_BASE;
      pt.pose.position.x = -6;
      pt.pose.orientation.w = 1;
      
      navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
      navigation_action_goal_.pose = pt;
      navigation_client_->sendGoal(navigation_action_goal_);
      ROS_INFO_STREAM("Goal sent : " << navigation_action_goal_);
      // navigation_client_->waitForResult();
      first_ = false;
   }   
   else
      ROS_INFO_STREAM("Undock stepping, first_ = false now");
}

void Undock::exitPoint() 
{
   ROS_INFO("exitpoint of undock, cancelling undock goal");
   navigation_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S E A R C H   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Search::entryPoint()
{
   /** @TODO: Add no objects in vision functionality */
   // start off with spiraling
   srv.request.resume_spiral_motion = true;
   near_volatile_ = false;
   ROS_INFO("entering scout_search state");
}

State& Search::transition()
{
   // if no transition, stay in current state
   if(!near_volatile_) {
      // stay in SCOUT_SEARCH_VOLATILE
      return *this;
   }
   // if near the volatile, switch to scout_locate_volatile state
   ROS_INFO("volatile detected, transitioning to scout_undock state");
   return getState(SCOUT_LOCATE_VOLATILE);
}

void Search::step()
{
   // execute spiral motion
   ROS_INFO("Executing spiral motion");
   spiralClient_.call(srv);
}

void Search::exitPoint()
{
   // cancel spiral motion 
   srv.request.resume_spiral_motion = false;
   spiralClient_.call(srv);
   ROS_INFO("Exited spiral search.");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  L O C A T E  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Locate::entryPoint()
{
   // we assume we are near the volatile
   near_volatile_ = true;
   first_ = true;
}

State& Locate::transition()
{
   // switch states once completed with locating the volatile
   if(!(resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && near_volatile_) {
      return *this;
   }
   ROS_WARN("Volatile found, moving to undock state");
   return getState(SCOUT_UNDOCK);
}

void Locate::step()
{
   if(first_)
   {
      resource_localiser_client_->sendGoal(goal);
      ROS_INFO_STREAM("Sending resource localization goal");
      first_ = false;
   }
   ROS_INFO_STREAM("Locating Step Function!");
}

void Locate::exitPoint()
{
   // none at the moment
   resource_localiser_client_->cancelGoal();
   near_volatile_ = false;
}

//////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// M A I N ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{
   ros::init(argc, argv, "scout_state_machine");
   ros::NodeHandle nh;

   try {
      ScoutScheduler cSchd;
      cSchd.addState(new Search());
      cSchd.addState(new Undock());
      cSchd.addState(new Locate());
      cSchd.setInitialState(SCOUT_SEARCH_VOLATILE);
      // cSchd.setInitialState(SCOUT_UNDOCK);
      cSchd.exec();
      return 0;
   }
   catch(StateMachineException& ex) {
      std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
   }
}
