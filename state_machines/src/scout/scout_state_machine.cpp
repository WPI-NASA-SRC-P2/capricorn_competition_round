#include <state_machines/scout_state_machine.h>
#include <iostream>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S C O U T   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ScoutState::ScoutState(uint32_t un_id, uint32_t un_max_count) :
    State(un_id, ToString("mystate_", un_id)),
    m_unMaxCount(un_max_count)
{
  robot_name_ = COMMON_NAMES::SCOUT_1;
  resource_localiser_client_ = new ResourceLocaliserClient_(RESOURCE_LOCALISER_ACTIONLIB, true);
  navigation_vision_client_ = new NavigationVisionClient(robot_name_ + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
  navigation_client_ = new NavigationClient(robot_name_ + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);

  spiralClient_ = nh_.serviceClient<operations::Spiral>(SCOUT_SEARCH_SERVICE);

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
   return getState(SCOUT_UNDOCK); // should be SCOUT_SEARCH_VOLATILE
}

void Undock::step()
{
   if(first_)
   {
      ROS_INFO_STREAM(robot_name_ << " State Machine: Undocking from volatile");
      geometry_msgs::PoseStamped pt;
      pt.header.frame_id = robot_name_ + ROBOT_BASE;
      pt.pose.position.x = -3;
      pt.pose.orientation.w = 1;
      
      navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
      navigation_action_goal_.pose = pt;
      navigation_client_->sendGoal(navigation_action_goal_);
      ROS_INFO_STREAM("Goal sent : " << navigation_action_goal_);
      navigation_client_->waitForResult();
      // first_ = false;
   }   
   else
      ROS_INFO_STREAM("Undock stepping, first_ = false now");
}

void Undock::exitPoint() 
{
   // reset first to false so that undock can be rerun at the next volatile
   first_ = false;
   ROS_INFO("exitpoint of undock");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S E A R C H   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Search::entryPoint()
{
   // none at the moment
}

State& Search::transition()
{
   // if no transition, stay in current state
   if(!near_volatile_) {
      // stay in SCOUT_SEARCH_VOLATILE
      return *this;
   }
   // if near the volatile, switch to scout_locate_volatile state
   return getState(SCOUT_UNDOCK);
}

void Search::step()
{
   ++m_unCount;
   ROS_INFO_STREAM("Searching Step Function!");
}

void Search::exitPoint()
{
   // none at the moment
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  L O C A T E  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Locate::entryPoint()
{
   // none at the moment
}

State& Locate::transition()
{
   // if no transition, stay in current state
   if(m_unCount <= 10 && near_volatile_) {
      return *this;
   }
   return getState(SCOUT_UNDOCK);
}

void Locate::step()
{
   ++m_unCount;
   ROS_INFO_STREAM("LOcatingStep Function!");
}

void Locate::exitPoint()
{
   // none at the moment
}

//////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// M A I N ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{
   ros::init(argc, argv, "scout_state_machine");
   ros::NodeHandle nh;

   try {
      ScoutScheduler cSchd(700);
      cSchd.addState(new Search());
      cSchd.addState(new Undock());
      cSchd.addState(new Locate());
      // cSchd.setInitialState(SCOUT_SEARCH_VOLATILE);
      cSchd.setInitialState(SCOUT_UNDOCK);
      cSchd.exec();
      return 0;
   }
   catch(StateMachineException& ex) {
      std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
   }
}
