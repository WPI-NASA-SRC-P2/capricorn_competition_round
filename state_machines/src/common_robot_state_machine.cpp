#include <state_machines/common_robot_state_machine.h>
#include <algorithm>

/****************************************/
/****************************************/

StateMachineException::StateMachineException(const std::string& str_msg) :
   m_strMsg(str_msg) {}

/****************************************/
/****************************************/

std::string StateMachineException::getMessage() const {
   return m_strMsg;
}

/****************************************/
/****************************************/

State& State::getState(uint32_t un_state) {
   return m_pcRobotScheduler->getState(un_state);
}

/****************************************/
/****************************************/

//UNDERSTANDING: When we add states in addState(state), the state is assigned to a scheduler. 
void State::setRobotScheduler(RobotScheduler& c_robot_scheduler) {
   m_pcRobotScheduler = &c_robot_scheduler;
}

/****************************************/
/****************************************/

/**RobotScheduler constructor, created primarily to run subscriber to update the states in the callback. Not virtual, so the destructor is not virtual either. 
*/
RobotScheduler::RobotScheduler(const ros::NodeHandle &nh, std::string robot_name) : nh_(nh), robot_name_(robot_name)
{
   // set subscriber for updating desired state from scheduler
   desired_state_sub_ = nh_.subscribe(CAPRICORN_TOPIC + ROBOTS_DESIRED_STATE_TOPIC, 2, &RobotScheduler::desiredStateCB, this);
}

RobotScheduler::~RobotScheduler() {
   std::for_each(
      m_mapStates.begin(),
      m_mapStates.end(),
      [](std::pair<uint32_t, State*> c_item){
         delete c_item.second;
      });
}

/****************************************/
/****************************************/

void RobotScheduler::desiredStateCB(const state_machines::robot_desired_state::ConstPtr &msg)
{
   // only change state if the desired state is a new one (to prevent calling entrypoint too often)
   // check for name equality
   
   // if(getState(msg->robot_desired_state) != *m_pcCurrent)
   if(msg->robot_name == robot_name_)
   {
      new_state_request = true;
      new_state_ = (STATE_MACHINE_TASK) msg->robot_desired_state;
      goal_pose_ = msg->goal_pose;
      ROS_INFO_STREAM(" state_machines | common_robot_state_machine.h | RobotScheduler | Pose RECEIVED = " << goal_pose_);
      ROS_INFO_STREAM("New state received: " << new_state_);             //Checking if the casting isn't messed up. 
   }                
    
}

//UNDERSTANDING: Adding states to an unordered map (map of states declared in the header).

void RobotScheduler::addState(State* pc_state) {
   if(m_mapStates.find(pc_state->getId()) == m_mapStates.end()) {
      m_mapStates[pc_state->getId()] = pc_state;
      pc_state->setRobotScheduler(*this);
   }
   else {
      ROS_ERROR_STREAM("Duplicated state id " << pc_state->getId());
   }
}

/****************************************/
/****************************************/

State& RobotScheduler::getState(uint32_t un_id) {
   auto pcState = m_mapStates.find(un_id);
   if(pcState != m_mapStates.end()) {
      return *(pcState->second);
   }
   else {
      ROS_ERROR_STREAM("Can't get state id "<< un_id);
   }
}

/****************************************/
/****************************************/

//UNDERSTANDING: First finds the state in the map and then sets the entry point of the current state.

void RobotScheduler::setInitialState(uint32_t un_state) {
   auto pcState = m_mapStates.find(un_state);
   // if state exists in map, then set it to the initial state of the scheduler
   if(pcState != m_mapStates.end()) {
      // acquire value of the state (every map has a key(first) and a value(second))
      m_pcCurrent = pcState->second;
      // completes entry point of the initial state
      m_pcCurrent->entryPoint();
   }
   else {
      ROS_ERROR_STREAM("Can't set initial state to "<< un_state);
   }
}


/****************************************/
/****************************************/
/** @HOTFIX:  Something has to be done about done()*/ 
bool RobotScheduler::done(){
   // just keep running in loop by returning false
   return false;
}

/****************************************/
/****************************************/

void RobotScheduler::step() {
   /* Only execute if 'current' was initialized */
   if(m_pcCurrent) {
      /* Attempt a transition, every state of every rover has its own transition() */
      State* cNewState = m_pcCurrent;
      if (new_state_request)
      {
         cNewState = &getState(new_state_);
         new_state_request = false;
      }

      if(cNewState != m_pcCurrent) {
         /* Perform transition */
         m_pcCurrent->exitPoint();
         cNewState->entryPoint();
         m_pcCurrent = cNewState;
      }
      /* Execute current state */
<<<<<<< HEAD
      if(!m_pcCurrent->isDone())
         m_pcCurrent->step();
      m_pcCurrent->updateStatus();
=======
      m_pcCurrent->publishStatus();
      m_pcCurrent->step();
>>>>>>> 2b0dcae5a34eb9065d97f5c737d03ea911149199
   }
   else {
      ROS_ERROR("The RobotScheduler has not been initialized, you must call SetInitialState()");
   }
}

/****************************************/
/****************************************/

//UNDERSTANDING: Each robot has its own done() and this is what is checked to perform step()
void RobotScheduler::exec() {
   while(!done() && ros::ok()) 
   {
      step();
      ros::Duration(0.5).sleep();
      ros::spinOnce();
   }
}



/****************************************/
/****************************************/
