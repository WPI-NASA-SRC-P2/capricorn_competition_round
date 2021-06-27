#include <state_machines/scout_state_machine.h>
#include <iostream>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S C O U T   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ScoutState::ScoutState(uint32_t un_id, ros::NodeHandle nh, std::string robot_name) :
    State(un_id, ToString("mystate_", un_id), nh, robot_name)
{
  robot_name_ = robot_name;
  ROS_INFO_STREAM("SCOUT STATE MACHINE: robot_name_ = " + robot_name_);
  // robot_state_status fields set to default values when state is constructed
  robot_current_state_ = un_id;
  current_state_done_ = false;
  last_state_succeeded_ = false;

   
  resource_localiser_client_ = new ResourceLocaliserClient_(CAPRICORN_TOPIC + robot_name + "/" + RESOURCE_LOCALISER_ACTIONLIB, true);
  /** @todo: FIX NAVIGATIONVISIONCLIENT TO BE CORRECT TOPIC */
  navigation_vision_client_ = new NavigationVisionClient(CAPRICORN_TOPIC + robot_name_ + "/" + robot_name_ + NAVIGATION_VISION_ACTIONLIB, true);
  navigation_client_ = new NavigationClient(CAPRICORN_TOPIC + robot_name_ + "/" + NAVIGATION_ACTIONLIB, true);
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
   
   // update the current status of the robot and publish it
   
}

bool Undock::isDone()
{
   // update the status of current state
   current_state_done_ = navigation_vision_client_->getState().isDone();

   return current_state_done_;
}

bool Undock::hasSucceeded()
{
   // update the status of current state
   // last_state_succeeded_ = (navigation_client_->getState() == actionlib::status::SUCCESS);
   last_state_succeeded_ = (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   
   return last_state_succeeded_;
}

void Undock::step()
{
   if(first_)
   {
      ROS_INFO_STREAM(robot_name_ << " State Machine: Undocking from volatile");
      navigation_vision_goal.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
      navigation_vision_goal.mode = COMMON_NAMES::NAV_VISION_TYPE::V_UNDOCK;
      navigation_vision_client_->sendGoal(navigation_vision_goal);
      first_ = false; 
      ROS_INFO_STREAM("Undock stepping, first_ = false now");
   }
}

void Undock::exitPoint() 
{
   ROS_INFO("exitpoint of undock, cancelling undock goal");
   navigation_vision_client_->cancelGoal();
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

bool Search::isDone()
{
   // if near the volatile, then the state is done
   current_state_done_ = near_volatile_;
   return (current_state_done_);
}

bool Search::hasSucceeded()
{
   // if near the volatile, then the state has succeeded
   last_state_succeeded_ = near_volatile_;
   return last_state_succeeded_;
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

bool Locate::isDone()
{
   // switch states once completed with locating the volatile
   current_state_done_ = (resource_localiser_client_->getState().isDone());
   return current_state_done_;
}

bool Locate::hasSucceeded()
{
   // state succeeded once rover is parked on top of volatile
   // last_state_succeeded_ = ((resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && near_volatile_);
   last_state_succeeded_ = (resource_localiser_client_->getState().isDone());
   return last_state_succeeded_;
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


// int main(int argc, char** argv)
// {
//    ros::init(argc, argv, "scout_state_machine");
//    ros::NodeHandle nh;

//    try {
//       ScoutScheduler cSchd;
//       // cSchd.initROS(nh, SCOUT_1_NAME);
//       cSchd.addState(new Search(nh, SCOUT_1_NAME));
//       cSchd.addState(new Undock(nh, SCOUT_1_NAME));
//       cSchd.addState(new Locate(nh, SCOUT_1_NAME));
//       cSchd.setInitialState(SCOUT_SEARCH_VOLATILE);
//       // cSchd.setInitialState(SCOUT_UNDOCK);
//       cSchd.exec();
//       return 0;
//    }
//    catch(StateMachineException& ex) {
//       std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
//    }
// }
