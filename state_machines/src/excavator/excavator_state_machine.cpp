#include <ros/ros.h>
#include <state_machines/excavator_state_machine.h>
#include <state_machines/RobotStateMachineTaskAction.h>
#include <actionlib/server/simple_action_server.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// E X C A V A T O R   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ExcavatorState::ExcavatorState(uint32_t un_id, uint32_t un_max_count) :
    State(un_id, ToString("mystate_", un_id)),
    m_unMaxCount(un_max_count)
{
  /** @todo: FIX SO THAT CAN USE ANY EXCAVATOR NAME, SO NO NAMESPACE ISSUES */
  robot_name_ = COMMON_NAMES::EXCAVATOR_1;
  /** @todo: FIX NAVIGATIONVISIONCLIENT TO BE CORRECT TOPIC */
  navigation_vision_client_ = new NavigationVisionClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + robot_name_ + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
  navigation_client_ = new NavigationClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);
  excavator_arm_client_ = new ExcavatorClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + COMMON_NAMES::EXCAVATOR_ACTIONLIB, true);
  ROS_INFO("Waiting for the excavator action servers...");
  navigation_vision_client_->waitForServer();
  navigation_client_->waitForServer();
  excavator_arm_client_->waitForServer();
  
  ROS_INFO("All excavator action servers started!");

//   objects_sub_ = nh_.subscribe(CAPRICORN_TOPIC + robot_name_ + OBJECT_DETECTION_OBJECTS_TOPIC, 1, &ExcavatorState::objectsCallback, this);
}

ExcavatorState::~ExcavatorState()
{
  delete navigation_vision_client_;
  delete navigation_client_;
  delete excavator_arm_client_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// C A L L B A C K S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** TODO: Check if the object detection callback is needed*/

/** TODO: Implememt entrypoint, transition, step and exit points for all states mentioned in the flow shown by ashay
 *  STATE_MACHINE_TASK::EXCAVATOR_GO_TO_LOC,  --> 2
    STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT,
    STATE_MACHINE_TASK::EXCAVATOR_PARK_AND_PUB--> 3
    STATE_MACHINE_TASK::EXCAVATOR_DIG_AND_DUMP_VOLATILE, --> 4
    STATE_MACHINE_TASK::EXCAVATOR_GOTO_DEFAULT_ARM_POSE, --> 1
    STATE_MACHINE_TASK::EXCAVATOR_RESET_ODOM_GROUND_TRUTH,
    STATE_MACHINE_TASK::EXCAVATOR_RESET_ODOM,
    STATE_MACHINE_TASK::EXCAVATOR_SYNC_ODOM,
    STATE_MACHINE_TASK::EXCAVATOR_FACE_PROCESSING_PLANT,
    STATE_MACHINE_TASK::EXCAVATOR_GO_TO_REPAIR};
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O _ T O _ S C O U T   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void GoToScout::entryPoint(const geometry_msgs::PoseStamped &target_loc)
void GoToScout::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
    first_ = true;
    // setting target_loc manually
    target_loc_ = geometry_msgs::PoseStamped();
    target_loc_.pose.position.x = 7.6;
    target_loc_.pose.position.y = 20.0;
    target_loc_.pose.position.z = 1.6;
    target_loc_.header.frame_id = "map";
    /** TODO: default arm position should be the state that is called before this state, etc */ 
    // goToDefaultArmPosition();
    ROS_INFO("entrypoint of go_to_scout");
    /** TODO: check if target object exists in the nav server?*/
}

State& GoToScout::transition()
{
//    if(!(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)) {
//       ROS_INFO("remaining in undock");
//       return *this;
//    }
//    ROS_INFO("transitioning out of undock");
   return getState(EXCAVATOR_GO_TO_SCOUT); // should be SCOUT_SEARCH_VOLATILE
}

void GoToScout::step()
{
   if(first_)
   {
        ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Going to scout");
        navigation_vision_goal_.desired_object_label = COMMON_NAMES::OBJECT_DETECTION_SCOUT_CLASS;
        navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_NAV_AND_NAV_VISION;
        navigation_vision_goal_.goal_loc = target_loc_;
        navigation_vision_client_->sendGoal(navigation_vision_goal_);
        first_ = false;
   }   
   else
        ROS_INFO_STREAM("GoToScout stepping, first_ = false now");
}

void GoToScout::exitPoint() 
{
   ROS_INFO("exitpoint of GoToScout, cancelling GoToScout goal");
   navigation_vision_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// D E F A U L T _ A R M   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GoToDefaultArmPosition::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
    first_ = true;
   //  counter_ = 0;
    ROS_INFO("entrypoint of goToDefaultArmPosition");
}

State& GoToDefaultArmPosition::transition()
{
   //  return getState(EXCAVATOR_GOTO_DEFAULT_ARM_POSE);
   if(first_) {
      ROS_INFO("remaining in default arm position state");
      return *this;
   }
   ROS_INFO("transitioning out of default arm position state");
   return getState(EXCAVATOR_GO_TO_SCOUT); // should be SCOUT_SEARCH_VOLATILE
   // return *this;
}

void GoToDefaultArmPosition::step()
{

/*
    ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Going to Default Excavator Arm Position");
    operations::ExcavatorGoal goal;
    goal.task = GO_TO_DEFAULT;

    excavator_arm_client_->sendGoal(goal);
    excavator_arm_client_->waitForResult();
    return true;
*/

   if(first_)
   {
        ROS_WARN_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Going to Default Excavator Arm Position");
        excavator_arm_goal_.task = EXCAVATOR_ARM_TASK::GO_TO_DEFAULT;
        excavator_arm_client_->sendGoal(excavator_arm_goal_);
        excavator_arm_client_->waitForResult();
        first_ = false;
   }   
   else
        ROS_WARN_STREAM("GoToDefaultArmPosition stepping, first_ = false now");
}

void GoToDefaultArmPosition::exitPoint() 
{
   ROS_WARN("exitpoint of GoToScout, cancelling GoToScout goal");
   /** TODO: check if it is actually possible to cancel excavator arm client goal*/
   // excavator_arm_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// P A R K _ A N D _ P U B   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ParkAndPub::entryPoint()
{
   //set entry variables
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Parking to Excavator");
   begin_ = ros::Time::now().toSec();
   current_ = ros::Time::now().toSec();
}

State& ParkAndPub::transition()
{
   // set transition condition (after 4 seconds, finish parking)
   if(current_ <= (begin_ + 4.0))
   {
      // ROS_INFO("remaining in park and pub state");
      return *this;
   }
   else
   {
      return getState(EXCAVATOR_GOTO_DEFAULT_ARM_POSE);
   }   
}

void ParkAndPub::step()
{
   /////////////////////////////////////////////
   //// Hardcoded straigh walk for 1.5 meters ////
   /////////////////////////////////////////////
   navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;
   navigation_action_goal_.forward_velocity = 0.6;   
   navigation_action_goal_.angular_velocity = 0;
   navigation_client_->sendGoal(navigation_action_goal_);
   ros::Duration(0.1).sleep();
   // ros::Duration(4).sleep();
   // current time (to ensure duration of action is only 4 seconds)
   current_ = ros::Time::now().toSec();

   // navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;
   // navigation_action_goal_.forward_velocity = 0.0;   
   // navigation_action_goal_.angular_velocity = 0;
   // navigation_client_->sendGoal(navigation_action_goal_);
   // ros::Duration(0.5).sleep();

   // return (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

void ParkAndPub::exitPoint()
{
   // cleanup the state (cancel nav goal)
   ROS_INFO("Excavator Parking Completed");
   navigation_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// D I G  A N D  D U M P  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DigAndDump::entryPoint()
{
   // initialize required variables
   volatile_found_ = false;
   dig_ = true;
   dump_ = false;
   digging_attempt_ = 0;
}

State& DigAndDump::transition()
{
   // set transition condition (after 4 seconds, finish parking)
   if(!volatile_found_)
   {
      // ROS_INFO("remaining in park and pub state");
      return *this;
   }
   else
   {
      return getState(EXCAVATOR_GOTO_DEFAULT_ARM_POSE);
   }   
}

void DigAndDump::step()
{
   // execute dig and dump process
   if(dig_)
   {
      excavator_arm_goal_.task = START_DIGGING;

      // These are the testing values, can be chagned
      // They start digging from the left of the robot
      if (new_vol_loc_flag_ == 1)
         excavator_arm_goal_.target.x = 1;
      else
         excavator_arm_goal_.target.x = 0;
      excavator_arm_goal_.target.y = 0;
      excavator_arm_goal_.target.z = 0;

      new_vol_loc_flag_ = 0; // Flag set to zero when excavator continues at a volatile location

      excavator_arm_client_->sendGoal(excavator_arm_goal_);
      digging_attempt_++;
      excavator_arm_client_->waitForResult();
      dump_ = (excavator_arm_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);   
      if(dump_)
         dig_ = false;
      if(digging_attempt_ >= 2)  //Change this number according to strategy
      {
         dig_ = false;
         dump_ = true;
         ROS_WARN_STREAM("DIGGING ATTEMPT TIMEOUT, ATTEMPTS MADE: " + digging_attempt_);
      }
   }
   if(dump_)
   {
      ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Dumping Volatile");
      excavator_arm_goal_.task = START_UNLOADING;

      // Should be tested with Endurance's parking code and
      // These values should be tuned accordingly
      excavator_arm_goal_.target.x = 0.85;
      excavator_arm_goal_.target.y = -2;
      excavator_arm_goal_.target.z = 0;

      excavator_arm_client_->sendGoal(excavator_arm_goal_);
      excavator_arm_client_->waitForResult();
      volatile_found_ = (excavator_arm_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   }
}

void DigAndDump::exitPoint()
{
   // cleanup (cancel goals)
   excavator_arm_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// M A I N ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{
   ros::init(argc, argv, "excavator_state_machine");
   ros::NodeHandle nh;

   try {
      ExcavatorScheduler cSchd(700);
      
      cSchd.addState(new GoToDefaultArmPosition());
      cSchd.addState(new GoToScout());
      cSchd.addState(new ParkAndPub());
      cSchd.addState(new DigAndDump());
      cSchd.setInitialState(EXCAVATOR_DIG_AND_DUMP_VOLATILE);
      // cSchd.setInitialState(EXCAVATOR_GOTO_DEFAULT_ARM_POSE);
      // cSchd.setInitialState(EXCAVATOR_GO_TO_SCOUT);
      cSchd.exec();
      return 0;
   }
   catch(StateMachineException& ex) {
      std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
   }
}