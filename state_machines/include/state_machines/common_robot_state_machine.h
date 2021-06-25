#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utils/common_names.h>
#include "ros/ros.h"
#include <state_machines/robot_state_status.h>
#include <state_machines/robot_desired_state.h>
using namespace COMMON_NAMES;

// class MacroState;
class State;
class RobotScheduler;

/****************************************/
/****************************************/

class StateMachineException {
   
public:
   
   StateMachineException(const std::string& str_msg);
   std::string getMessage() const;

private:

   std::string m_strMsg;
};

/****************************************/
/****************************************/

// /**
//  * The macrostate is a state machine for team-level coordination.
//  *
//  * A state in the macrostate is a collection of individual robot states.
//  */
// class MacroState {
//   public:
// };

/****************************************/
/****************************************/

class RobotScheduler {
   
public:
   RobotScheduler(const ros::NodeHandle &nh, std::string robot_name);

   ~RobotScheduler();

   void addState(State* pc_state);

   State& getState(uint32_t un_id);

   void setInitialState(uint32_t un_state);
   
   void step();

   void exec();

   bool done();
   
   void setState(STATE_MACHINE_TASK new_state);

   /**
   * @brief Scheduler robot desired state callback for assigning robot's state
   * 
   * @param msg 
   */
   void desiredStateCB(const state_machines::robot_desired_state::ConstPtr &msg);

   geometry_msgs::PoseStamped getDesiredPose(){
      ROS_INFO_STREAM(" state_machines | common_robot_state_machine.h | RobotScheduler | Pose Sent = " << goal_pose_);
      return goal_pose_;}

private:
   ros::NodeHandle nh_;
   State* m_pcCurrent;
   std::unordered_map<uint32_t, State*> m_mapStates;
   geometry_msgs::PoseStamped goal_pose_;

protected:
   bool new_state_request = false;
   STATE_MACHINE_TASK new_state_;
   ros::Subscriber desired_state_sub_;
   std::string robot_name_;
};

/****************************************/
/****************************************/

class State {
   
public:

   State(uint32_t un_id,
         const std::string& str_name, ros::NodeHandle nh, std::string robot_name) :
      m_pcRobotScheduler(nullptr),
      m_unId(un_id),
      m_strName(str_name), nh_(nh), robot_name_(robot_name) {

      // publish the robot's status
      status_pub_ = nh_.advertise<state_machines::robot_state_status>(CAPRICORN_TOPIC + ROBOTS_CURRENT_STATE_TOPIC, 10);
     
      }

   virtual ~State() {}

   uint32_t getId() const { return m_unId; }

   const std::string& getName() const { return m_strName; }
   
   virtual void entryPoint() = 0;
   
   virtual void exitPoint() = 0;
   
   virtual void step() = 0;
   
   virtual bool isDone() = 0;
   
   virtual bool hasSucceeded() = 0;

   virtual State& transition() = 0;    //This should be "deprecated".

   State& getState(uint32_t un_state);

   void setRobotScheduler(RobotScheduler& c_robot_scheduler);

   void setRobotName(const std::string &robot_name){ robot_name_ = robot_name; }
   
   void updateStatus() 
   {
      status_.robot_name = robot_name_;
      status_.robot_current_state = robot_current_state_;
      status_.current_state_done = current_state_done_;
      status_.last_state_succeeded = last_state_succeeded_;
      status_pub_.publish(status_);
   }

private:
   
   uint32_t m_unId;
   std::string m_strName;

protected:
  RobotScheduler* m_pcRobotScheduler;

  ros::NodeHandle nh_;
  std::string robot_name_;
  
  int robot_current_state_;
  bool current_state_done_;
  bool last_state_succeeded_;
  state_machines::robot_state_status status_;
  ros::Publisher status_pub_;
  
};

#endif // STATE_MACHINE_H
