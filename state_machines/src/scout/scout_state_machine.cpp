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

  spiralClient_ = nh_.serviceClient<operations::Spiral>(SCOUT_SEARCH_SERVICE);

  volatile_sub_ = nh_.subscribe("/" + robot_name_ + VOLATILE_SENSOR_TOPIC, 1000, &ScoutState::volatileSensorCB, this);
  objects_sub_ = nh_.subscribe(CAPRICORN_TOPIC + robot_name_ + OBJECT_DETECTION_OBJECTS_TOPIC, 1, &ScoutState::objectsCallback, this);
}

ScoutState::~ScoutState()
{
  delete resource_localiser_client_;
  delete navigation_vision_client_;
}

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

/****************************************/
/****************************************/


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// U N D O C K   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


State& Undock::transition()
{
   if(m_unCount < m_unMaxCount) {
      return *this;
   }
   return getState(SCOUT_SEARCH_VOLATILE);
}

void Undock::step()
{
   ++m_unCount;
   ROS_INFO_STREAM("Undock Step Function!");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S E A R C H   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

State& Search::transition()
{
   // if no transition, stay in current state
   // @todo:   check if (!near_volatile) 
   if(m_unCount < m_unMaxCount) {
      return *this;
   }
   return getState(SCOUT_UNDOCK);
}

void Search::step()
{
   ++m_unCount;
   ROS_INFO_STREAM("Searching Step Function!");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S E A R C H   L O C A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

State& Locate::transition()
{
   // if no transition, stay in current state
   if(m_unCount < m_unMaxCount) {
      return *this;
   }
   return getState(SCOUT_LOCATE_VOLATILE);
}

void Locate::step()
{
   ++m_unCount;
   ROS_INFO_STREAM("Searching Step Function!");
}



//////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// M A I N ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{
   ros::init(argc, argv, "scout_state_machine");
   ros::NodeHandle nh;

   try {
      ScoutScheduler cSchd(70);
      cSchd.addState(new Search());
      cSchd.addState(new Locate());
      cSchd.setInitialState(SCOUT_SEARCH_VOLATILE);
      cSchd.exec();
      return 0;
   }
   catch(StateMachineException& ex) {
      std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
   }
}
