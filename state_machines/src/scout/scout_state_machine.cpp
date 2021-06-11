#include <state_machines/scout_state_machine.h>
#include <iostream>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S C O U T   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
ScoutState::ScoutState(uint32_t un_id, uint32_t un_max_count) :
    State(un_id, ToString("mystate_", un_max_count)),
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

// /****************************************/
// /****************************************/

// class MyState10 : public ScoutState {
   
// public:
   
//    MyState10() : ScoutState(STATE_10, 10) {}

//    State& Transition() override {
//       if(m_unCount < m_unMaxCount) {
//          return *this;
//       }
//       return GetState(STATE_20);
//    }
   
// };

// /****************************************/
// /****************************************/

// class MyState20 : public ScoutState {
   
// public:
   
//    MyState20() : ScoutState(STATE_20, 20) {}

//    State& Transition() override {
//       if(m_unCount < m_unMaxCount) {
//          return *this;
//       }
//       return GetState(STATE_5);
//    }
   
// };

/****************************************/
/****************************************/

class MyScheduler : public RobotScheduler {
   
public:

   MyScheduler(uint32_t un_max_t) :
      m_unT(0),
      m_unMaxT(un_max_t) {}

   void step() override {
      /* Increase time counter */
      ++m_unT;
      std::cout << "t = " << m_unT << std::endl;
      /* Call parent class step */
      RobotScheduler::step();
   }

   bool done() override {
      return m_unT >= m_unMaxT;
   }

private:

   uint32_t m_unT;
   uint32_t m_unMaxT;
};

/****************************************/
/****************************************/

int main(int argc, char** argv)
{
   ros::init(argc, argv, "scout_state_machine");
   ros::NodeHandle nh;

   try {
      MyScheduler cSchd(70);
      cSchd.addState(new Undock());
    //   cSchd.AddState(new MyState10());
    //   cSchd.AddState(new MyState20());
      cSchd.setInitialState(SCOUT_UNDOCK);
      cSchd.exec();
      return 0;
   }
   catch(StateMachineException& ex) {
      std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
   }
}
