#include <ros/ros.h>
#include <state_machines/excavator_state_machine.h>
#include <state_machines/RobotStateMachineTaskAction.h>
#include <actionlib/server/simple_action_server.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// E X C A V A T O R   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ExcavatorState::ExcavatorState(uint32_t un_id, ros::NodeHandle nh, std::string robot_name) :
    State(un_id, ToString("mystate_", un_id), nh, robot_name)
{
  /** @todo: FIX SO THAT CAN USE ANY EXCAVATOR NAME, SO NO NAMESPACE ISSUES */
  robot_name_ = robot_name;
  robot_current_state_ = un_id;
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
    STATE_MACHINE_TASK::EXCAVATOR_GO_TO_REPAIR,
    STATE_MACHINE_TASK::EXCAVATOR_PRE_HAULER_PARK_MANEUVER};
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
   //  target_loc_ = geometry_msgs::PoseStamped();
   //  target_loc_.pose.position.x = 7.6;
   //  target_loc_.pose.position.y = 20.0;
   //  target_loc_.pose.position.z = 1.6;
   //  target_loc_.header.frame_id = "map";
    ROS_INFO("entrypoint of go_to_scout");
    target_loc_ = m_pcRobotScheduler->getDesiredPose();
}

State& GoToScout::transition()
{
   return getState(EXCAVATOR_GO_TO_SCOUT); 
}

void GoToScout::step()
{
   if(first_)
   {
        ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Going to scout");
        navigation_vision_goal_.desired_object_label = COMMON_NAMES::OBJECT_DETECTION_SCOUT_CLASS;
        navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_NAV_AND_NAV_VISION;
      //   ROS_WARN("Target Loc for GOTOSCOUT");
        target_loc_ = m_pcRobotScheduler->getDesiredPose();
      //   ROS_WARN_STREAM(target_loc_);
        if(target_loc_.pose.position.x != 0.0)   /** TODO: should be handled in navigation stack */
        {
         navigation_vision_goal_.goal_loc = target_loc_;
         navigation_vision_client_->sendGoal(navigation_vision_goal_);
         ROS_INFO("Excavator State Machine: SUCCESSFUL POSE RECEIVED");
         first_ = false;
        }
        
   }   
   // else
      //   ROS_INFO_STREAM("GoToScout stepping, first_ = false now");
}

void GoToScout::exitPoint() 
{
   ROS_INFO("exitpoint of GoToScout, cancelling GoToScout goal");
   navigation_vision_client_->cancelGoal();
}

bool GoToScout::isDone() {
   current_state_done_ = navigation_vision_client_->getState().isDone();
   return current_state_done_;
} 

bool GoToScout::hasSucceeded() {
   last_state_succeeded_ = (navigation_vision_result_.result == COMMON_RESULT::SUCCESS);
   return last_state_succeeded_;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// D E F A U L T _ A R M   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GoToDefaultArmPosition::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
    first_ = true;
    ROS_INFO("entrypoint of goToDefaultArmPosition");
}

State& GoToDefaultArmPosition::transition()
{
   // //  return getState(EXCAVATOR_GOTO_DEFAULT_ARM_POSE);
   // if(first_) {
   //    ROS_INFO("remaining in default arm position state");
   //    return *this;
   // }
   // ROS_INFO("transitioning out of default arm position state");
   // return getState(EXCAVATOR_GO_TO_SCOUT); // should be SCOUT_SEARCH_VOLATILE
   // // return *this;
}

void GoToDefaultArmPosition::step()
{

/*
    ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Going to Default Excavator Arm Position");
    operations::ExcavatorGoal goal;
    goal.task = GO_TO_DEFAULT;

    excavator_arm_client_->sendGoal(goal);
   //  excavator_arm_client_->waitForResult();
    return true;
*/

   if(first_)
   {
        ROS_WARN_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Going to Default Excavator Arm Position");
        excavator_arm_goal_.task = EXCAVATOR_ARM_TASK::GO_TO_DEFAULT;
        excavator_arm_client_->sendGoal(excavator_arm_goal_);
      //   excavator_arm_client_->waitForResult();
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

bool GoToDefaultArmPosition::isDone() {
   current_state_done_ = excavator_arm_client_->getState().isDone();
   return current_state_done_;
} 

bool GoToDefaultArmPosition::hasSucceeded() {

   last_state_succeeded_ = (excavator_arm_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   return last_state_succeeded_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// P A R K _ A N D _ P U B   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ParkAndPub::entryPoint()
{
   //set entry variables
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Parking to Scout");
   first_ = true;
}

void ParkAndPub::step()
{
      if(first_){
         closeInToScout();
         first_ = false;
      }
}

void ParkAndPub::exitPoint()
{
   // cleanup the state (cancel nav goal)
   ROS_INFO("Excavator Parking Completed");
   navigation_vision_client_->cancelGoal();
}

bool ParkAndPub::isDone() 
{
   current_state_done_ = navigation_vision_client_->getState().isDone();
   // if(current_state_done_)
      // ROS_WARN_STREAM("Park and Pub Done");
   return current_state_done_;
} 

bool ParkAndPub::hasSucceeded() 
{
   last_state_succeeded_ = (navigation_vision_result_.result == COMMON_RESULT::SUCCESS);
   if(last_state_succeeded_)
      ROS_WARN_STREAM("Park and Pub to Scout Completed Successfully");
   return last_state_succeeded_;
}

/**Kind of unnecessary because when the excavator arrives at the scout, it is already centered*/
void ParkAndPub::navToScout()
{
   //use vision nav to get to scout
   navigation_vision_goal_.desired_object_label = COMMON_NAMES::OBJECT_DETECTION_SCOUT_CLASS;
   navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_CENTER;
   navigation_vision_client_->sendGoal(navigation_vision_goal_);
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << "NAV VISION GOAL SENT");
    // once at centering, keep centering until finished, then will exit the state 
   //first_ = false;
}


void ParkAndPub::closeInToScout()
{
   navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
   navigation_action_goal_.epsilon = 0.1;                      //TUNABLE
   navigation_action_goal_.pose.header.frame_id = robot_name_+"_base_footprint";
   navigation_action_goal_.pose.pose.position.x = PARAM_EXCAVATOR_HIT_SCOUT;        
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << "driving on top of volatile");
   navigation_client_->sendGoal(navigation_action_goal_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// P R E _ P A R K _ H A U L E R   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PreParkHauler::entryPoint()
{
   //set entry variables
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Pre-parking to Hauler");
   first_ = true;
   goToVolatileDone_ = false;
   centerHaulerDone_ = false;
   getInArmPositionDone_ = false;
   goal_= goal_states_::GO_TO_VOLATILE;
}

/** TODO: 
 * Drive on top of volatile using NAV::MANUAL
 * Center it to the hauler
 * Finally, move the excavator back a little to make sure thearm digsa right at the volatile. 
 * And set up an enum of goals and check for their completion and set flags. 
 * Set up seperate functions for all goals. 
 * */ 
void PreParkHauler::step()
{  
   switch (goal_)
   {
   case GO_TO_VOLATILE:
      goToVolatile();
      if(goToVolatileDone_){
         goal_ = goal_states_::CENTER_TO_HAULER;
         first_ = true;
      }
      break;
   case CENTER_TO_HAULER:
      centerHauler();
      if(centerHaulerDone_){
         goal_ = goal_states_::GET_IN_DIGGING_POSITION;
         first_ = true;
      }
      break;
   case GET_IN_DIGGING_POSITION:
      getInArmPosition();
      break;
   default:
      ROS_WARN_STREAM("Incorrect goal found:"<<goal_);
      break;
   }
}



void PreParkHauler::exitPoint()
{
   // cleanup the state (cancel nav goal)
   ROS_INFO("Excavator Parking Completed");
   navigation_action_goal_.epsilon = 0.5;
   navigation_client_->cancelGoal();
   navigation_vision_client_->cancelGoal();
}

bool PreParkHauler::isDone() {
   // bool nav_done = navigation_client_->getState().isDone();
   
   // current_state_done_ = vis_nav_done && nav_done;
   current_state_done_ = getInArmPositionDone_;
   if(current_state_done_)
      ROS_WARN_STREAM("PreParkHauler Completed");
   return current_state_done_;

} 

bool PreParkHauler::hasSucceeded() {

   last_state_succeeded_ = getInArmPositionDone_ && centerHaulerDone_;
   if(last_state_succeeded_)
      ROS_WARN_STREAM("PreParkHauler Completed Successfully");
   return last_state_succeeded_;
   // return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

void PreParkHauler::goToVolatile() {
   //use manual nav goal to get on top of the volatile (with epsilon of 0.1 for higher accuracy)
   if(first_){
      navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
      navigation_action_goal_.epsilon = 0.1;                      //TUNABLE
      navigation_action_goal_.pose.header.frame_id = robot_name_+"_base_footprint";
      navigation_action_goal_.pose.pose.position.x = 1.0;         //TUNABLE->Should sit as close to the volatile as possible. 
      ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << "driving on top of volatile");
      navigation_client_->sendGoal(navigation_action_goal_);
      first_ = false;
   }
   else
      goToVolatileDone_ = (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

void PreParkHauler::centerHauler() {
   // centers excavator to scout to ensure proper scout undock/reset (excavator already reaches close to scout in GoToScout)
   if(first_) {
      navigation_vision_goal_.desired_object_label = COMMON_NAMES::OBJECT_DETECTION_HAULER_CLASS;
      navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_CENTER;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << "NAV VISION GOAL SENT");
      // once at centering, keep centering until finished, then will exit the state 
      first_ = false;
   }
   else
      centerHaulerDone_ = navigation_vision_client_->getState().isDone();
      // centerHaulerDone_ = (navigation_vision_result_.result == COMMON_RESULT::SUCCESS);
}

void PreParkHauler::getInArmPosition() {
   //use manual nav goal to get to good dig position
   if(first_){
      navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;
      navigation_action_goal_.forward_velocity = -0.6;
      navigation_action_goal_.angular_velocity = 0;
      ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << "driving to digging position");
      navigation_client_->sendGoal(navigation_action_goal_);
      ros::Duration(0.5).sleep();
      navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;
      navigation_action_goal_.forward_velocity = 0.0;   
      navigation_action_goal_.angular_velocity = 0;
      ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << "UNDOCKING: backing up beep beep beep");
      navigation_client_->sendGoal(navigation_action_goal_);
      ros::Duration(0.5).sleep();
      first_ = false;
   }
   else
      getInArmPositionDone_ = true;
      // getInArmPositionDone_ = (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// D I G  A N D  D U M P  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DigAndDump::entryPoint()
{
   // initialize required variables
   volatile_found_ = false;
   dig_ = true;
   dump_ = true;
   last_state_dig_ = false;
   digging_server_succeeded_ = true;
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
   if(last_state_dig_ && digging_server_succeeded_)
   {
      if(dump_){
         dumpVolatile();
         dump_ = false;
         digging_attempt_ = 0;
      }
   }
   else if(!last_state_dig_ && digging_server_succeeded_)
   {
      if(dig_){
         digVolatile();
         dig_ = false;
      }
   }
   // this else if is to be used as a replacement for the one above for implementing digging attempts   
   // else if(!last_state_dig_ && (digging_server_succeeded_ || digging_attempt < 2))

   digging_server_succeeded_ = (excavator_arm_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   // digging_attempt_ += (excavator_arm_client_->getState().isDone();
   ROS_INFO_STREAM("Digging server succeeded = " << digging_server_succeeded_);
   // ROS_INFO_STREAM("Digging server succeeded = " << digging_server_succeeded_);
   // digVolatile();

}

void DigAndDump::exitPoint()
{
   // cleanup (cancel goals)
   excavator_arm_client_->cancelGoal();
}

bool DigAndDump::isDone() {
   current_state_done_ = excavator_arm_client_->getState().isDone();
   return current_state_done_;
} 

bool DigAndDump::hasSucceeded() {

   last_state_succeeded_ = (excavator_arm_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   return last_state_succeeded_;
}

void DigAndDump::digVolatile()
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
   // digging_attempt_++;
   // excavator_arm_client_->waitForResult();
   // dump_ = (excavator_arm_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);   
   // if(dump_)
   //    dig_ = false;
   // if(digging_attempt_ >= 2)  //Change this number according to strategy
   // {
   //    // dig_ = false;
   //    // dump_ = true;
   //    last_state_dig_ = true;
   //    ROS_WARN_STREAM("DIGGING ATTEMPT TIMEOUT, ATTEMPTS MADE: " + digging_attempt_);
   // }
   last_state_dig_ = true;
   dump_ = true;
}

void DigAndDump::dumpVolatile()
{
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Dumping Volatile");
   excavator_arm_goal_.task = START_UNLOADING;

   // Should be tested with Endurance's parking code and
   // These values should be tuned accordingly
   excavator_arm_goal_.target.x = 0.85;
   excavator_arm_goal_.target.y = -2;
   excavator_arm_goal_.target.z = 0;

   excavator_arm_client_->sendGoal(excavator_arm_goal_);
   // excavator_arm_client_->waitForResult();
   volatile_found_ = (excavator_arm_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   last_state_dig_ = false;
   dig_ = true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// M A I N ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O _ T O _ L O C   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ExcavatorGoToLoc::entryPoint()
{
    // declare entrypoint variables
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Go To Loc");
    // pose of the excavator, supposed to be provided by scheduler
    target_loc_ = m_pcRobotScheduler->getDesiredPose();    
    first_ = true;
}

void ExcavatorGoToLoc::step()
{
    // go to excavator using planner+vision goal
    if(first_)
    {
        navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
        navigation_action_goal_.pose = target_loc_;
        navigation_client_->sendGoal(navigation_action_goal_);
        first_ = false;
    }//new ma
}

bool ExcavatorGoToLoc::isDone() {
   current_state_done_ = navigation_client_->getState().isDone();
   return current_state_done_;
} 

bool ExcavatorGoToLoc::hasSucceeded() {
   last_state_succeeded_ = (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   if(last_state_succeeded_)
      ROS_WARN_STREAM("Go to Scout Completed Successfully");
   return last_state_succeeded_;
}

void ExcavatorGoToLoc::exitPoint()
{
    // clean up (cancel goals)
    navigation_client_->cancelGoal();
    ROS_INFO("Reached scout, preparing to park");
}

// int main(int argc, char** argv)
// {
//    ros::init(argc, argv, "excavator_state_machine");
//    ros::NodeHandle nh;

//    try {
//       ExcavatorScheduler cSchd(700);
      
//       cSchd.addState(new GoToDefaultArmPosition());
//       cSchd.addState(new GoToScout());
//       cSchd.addState(new ParkAndPub());
//       cSchd.addState(new DigAndDump());
//       cSchd.setInitialState(EXCAVATOR_DIG_AND_DUMP_VOLATILE);
//       // cSchd.setInitialState(EXCAVATOR_GOTO_DEFAULT_ARM_POSE);
//       // cSchd.setInitialState(EXCAVATOR_GO_TO_SCOUT);
//       cSchd.exec();
//       return 0;
//    }
//    catch(StateMachineException& ex) {
//       std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
//    }
// }