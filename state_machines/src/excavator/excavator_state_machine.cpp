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
  park_robot_client_ = new ParkRobotClient(CAPRICORN_TOPIC + robot_name_ + "/" + robot_name_ + COMMON_NAMES::PARK_HAULER_ACTIONLIB, true);
  ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: Waiting for the excavator action servers...");
  navigation_vision_client_->waitForServer();
  navigation_client_->waitForServer();
  excavator_arm_client_->waitForServer();
  park_robot_client_->waitForServer();
  
  ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: All excavator action servers started!");

//   objects_sub_ = nh_.subscribe(CAPRICORN_TOPIC + robot_name_ + OBJECT_DETECTION_OBJECTS_TOPIC, 1, &ExcavatorState::objectsCallback, this);
  odom_sub_ = nh_.subscribe("/" + robot_name_ + RTAB_ODOM_TOPIC, 10, &ExcavatorState::odomCallback, this);
}

ExcavatorState::~ExcavatorState()
{
  delete navigation_vision_client_;
  delete navigation_client_;
  delete excavator_arm_client_;
  delete park_robot_client_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// C A L L B A C K S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void ExcavatorState::odomCallback(const nav_msgs::Odometry odom)
{
   odom_ = odom;
   // excavator_pose_ = odom.pose.pose;
   excavator_pose_.pose = odom.pose.pose;
   excavator_pose_.header = odom.header;
   
}

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
    ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: entrypoint of go_to_scout");
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
        ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: State Machine: Going to scout");
        navigation_vision_goal_.desired_object_label = COMMON_NAMES::OBJECT_DETECTION_SCOUT_CLASS;
        navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_NAV_AND_NAV_VISION;
      //   ROS_WARN("Target Loc for GOTOSCOUT");
        target_loc_ = m_pcRobotScheduler->getDesiredPose();
      //   ROS_WARN_STREAM(target_loc_);
        if(target_loc_.pose.position.x != 0.0)   /** TODO: should be handled in navigation stack */
        {
         navigation_vision_goal_.goal_loc = target_loc_;
         navigation_vision_client_->sendGoal(navigation_vision_goal_);
         ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: Excavator State Machine: SUCCESSFUL POSE RECEIVED");
         first_ = false;
        }
        
   }   
   // else
      //   ROS_INFO_STREAM("GoToScout stepping, first_ = false now");
}

void GoToScout::exitPoint() 
{
   ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: exitpoint of GoToScout, cancelling GoToScout goal");
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
    ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: entrypoint of goToDefaultArmPosition");
}

State& GoToDefaultArmPosition::transition()
{
   // //  return getState(EXCAVATOR_GOTO_DEFAULT_ARM_POSE);
   // if(first_) {
   //    ROS_INFO_STREAM("remaining in default arm position state");
   //    return *this;
   // }
   // ROS_INFO_STREAM("transitioning out of default arm position state");
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
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: State Machine: Parking to Scout");
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
   ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: Excavator Parking Completed");
   navigation_client_->cancelGoal();
}

bool ParkAndPub::isDone() 
{
   current_state_done_ = navigation_client_->getState().isDone();
   // if(current_state_done_)
      // ROS_WARN_STREAM("Park and Pub Done");
   return current_state_done_;
} 

bool ParkAndPub::hasSucceeded() 
{
   // last_state_succeeded_ = (navigation_result_.result == COMMON_RESULT::SUCCESS);
   last_state_succeeded_ = (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   // if(last_state_succeeded_)
   //    ROS_WARN_STREAM("Park and Pub to Scout Completed Successfully");
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
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: State Machine: Pre-parking to Hauler");
   first_ = true;
   goToVolatileDone_ = false;
   centerHaulerDone_ = false;
   getInArmPositionDone_ = false;
   goal_= goal_states_::GO_TO_VOLATILE;
}

/** TODO: 
 * Drive on top of volatile using NAV::MANUAL
 * Center it to the haulerbot_letion and set flags. 
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
      ROS_WARN_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: Incorrect goal found:"<<goal_);
      break;
   }
}



void PreParkHauler::exitPoint()
{
   // cleanup the state (cancel nav goal)
   ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: Excavator Parking Completed");
   navigation_action_goal_.epsilon = 0.5;
   navigation_client_->cancelGoal();
   navigation_vision_client_->cancelGoal();
}

bool PreParkHauler::isDone() {
   // bool nav_done = navigation_client_->getState().isDone();
   
   // current_state_done_ = vis_nav_done && nav_done;
   current_state_done_ = getInArmPositionDone_;
   // if(current_state_done_)
   //    ROS_WARN_STREAM("PreParkHauler Completed");
   return current_state_done_;

} 

bool PreParkHauler::hasSucceeded() {

   last_state_succeeded_ = getInArmPositionDone_ && centerHaulerDone_;
   // if(last_state_succeeded_)
   //    ROS_WARN_STREAM("PreParkHauler Completed Successfully");
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
   done_digging_ = false;
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
      // ROS_INFO_STREAM("remaining in park and pub state");
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
         volatile_found_ = true;
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
   done_digging_ = (excavator_arm_client_->getState() == actionlib::SimpleClientGoalState::ABORTED);
   // digging_attempt_ += (excavator_arm_client_->getState().isDone();
   // ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: Digging server succeeded = " << digging_server_succeeded_);
   // ROS_INFO_STREAM("Digging server succeeded = " << digging_server_succeeded_);
   // digVolatile();

}

void DigAndDump::exitPoint()
{
   // cleanup (cancel goals)
   excavator_arm_client_->cancelGoal();
}

bool DigAndDump::isDone() {
   current_state_done_ = done_digging_;
   return current_state_done_;
} 

bool DigAndDump::hasSucceeded() {

   last_state_succeeded_ = volatile_found_;
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
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: State Machine: Dumping Volatile");
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
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: State Machine: Go To Loc");
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
   // if(last_state_succeeded_)
   //    ROS_WARN_STREAM("Go to Scout Completed Successfully");
   return last_state_succeeded_;
}

void ExcavatorGoToLoc::exitPoint()
{
    // clean up (cancel goals)
    navigation_client_->cancelGoal();
    ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: Reached scout, preparing to park");
}

/** @TODO: Add resetodom macrostate from when it finally dumps the volatile till it parks at processing plant and then resets odometry */

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  R E S E T _ O D O M   C L A S S ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////




void ExcavatorResetOdomAtHopper::entryPoint()
{
   // we assume we are near the volatile
   first_GTPP = true;
   first_PAH = true;
   first_UFH = true;
   first_GTR = true;
   micro_state = GO_TO_PROC_PLANT;
   macro_state_succeeded = false;
   macro_state_done = false;
   state_done =false;
}

bool ExcavatorResetOdomAtHopper::isDone()
{
   // switch states once completed with locating the volatile
   current_state_done_ = macro_state_done;
   return current_state_done_;
}

bool ExcavatorResetOdomAtHopper::hasSucceeded()
{
   // state succeeded once rover is parked on top of volatile
   // last_state_succeeded_ = ((resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && near_volatile_);
   last_state_succeeded_ = macro_state_succeeded;
   return last_state_succeeded_;
}

void ExcavatorResetOdomAtHopper::step()
{
   switch (micro_state)
   {
   case GO_TO_PROC_PLANT: 
      goToProcPlant();
      break;
   case PARK_AT_HOPPER:
      parkAtHopper();
      break;
   case UNDOCK_FROM_HOPPER:
      undockFromHopper();
      break;
   case RESET_ODOM_AT_HOPPER:
      resetOdom();
      break;
   case GO_TO_REPAIR_STATION:
      goToRepair();
      break;
   case EXCAVATOR_IDLE:
      idleExcavator();
      break;
   default:
      break;
   }
}

void ExcavatorResetOdomAtHopper::goToProcPlant()
{
   if (first_GTPP)
   {
      operations::NavigationVisionGoal navigation_vision_goal;
      navigation_vision_goal.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
      navigation_vision_goal.mode = V_REACH;
      navigation_vision_client_->sendGoal(navigation_vision_goal);
      first_GTPP = false;
      return;
   }

   bool is_done = (navigation_vision_client_->getState().isDone());
   if (is_done)
   {
      if (navigation_vision_client_->getResult()->result == COMMON_RESULT::SUCCESS)
         micro_state = PARK_AT_HOPPER;
      // else
         // state go to 0 0
   }
}

void ExcavatorResetOdomAtHopper::parkAtHopper()
{
   if (first_PAH)
   {
      park_robot_goal_.hopper_or_excavator = OBJECT_DETECTION_HOPPER_CLASS;
      park_robot_client_->sendGoal(park_robot_goal_);
      first_PAH = false;
      return;
   }

   bool is_done = (park_robot_client_->getState().isDone());
   if (is_done)
   {
      if (park_robot_client_->getResult()->result == COMMON_RESULT::SUCCESS){
         first_PAH = true;
         micro_state = PARK_AT_HOPPER;
      }
   }
}

void ExcavatorResetOdomAtHopper::undockFromHopper()
{
   if (first_UFH)
   {
      operations::NavigationVisionGoal navigation_vision_goal;
      navigation_vision_goal.desired_object_label = OBJECT_DETECTION_HOPPER_CLASS;
      navigation_vision_goal.mode = COMMON_NAMES::NAV_VISION_TYPE::V_HARDCODED_UNDOCK;
      navigation_vision_client_->sendGoal(navigation_vision_goal);
      first_UFH = false;
      return;
   }

   bool is_done = (navigation_vision_client_->getState().isDone());
   if (is_done)
   {
      if (navigation_vision_client_->getResult()->result == COMMON_RESULT::SUCCESS)
         micro_state = RESET_ODOM_AT_HOPPER;
      // Dont find a reason it should fail,
   }
}

void ExcavatorResetOdomAtHopper::resetOdom()
{
   ros::ServiceClient resetOdometryClient = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
   maploc::ResetOdom srv;
   srv.request.target_robot_name = robot_name_;
   srv.request.at_hopper = true;
   // macro_state_succeeded = resetOdometryClient.call(srv);
   // macro_state_done = true;
   micro_state = GO_TO_REPAIR_STATION;
   return;
}

void ExcavatorResetOdomAtHopper::goToRepair()
{
   if(first_GTR)
   {
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_REPAIR_STATION_CLASS;
      navigation_vision_goal_.mode = V_REACH;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Going to repair station vision goal sent");  
      first_GTR = false;
      return;
   }
   
   bool is_done = (navigation_vision_client_->getState().isDone());
   if (is_done)
   {
      macro_state_done = true;
      macro_state_succeeded = (navigation_vision_client_->getResult()->result == COMMON_RESULT::SUCCESS);
      if (macro_state_succeeded)
         micro_state = EXCAVATOR_IDLE;
      // Dont find a reason it should fail,
   }
}

void ExcavatorResetOdomAtHopper::exitPoint()
{
   // none at the moment
   navigation_vision_client_->cancelGoal();
   park_robot_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  G O  TO  R E P A I R  S T A T I O N  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ExcavatorGoToRepairStation::entryPoint()
{
   first_ = true;
}

bool ExcavatorGoToRepairStation::isDone()
{
   current_state_done_ = navigation_vision_client_->getState().isDone();
   return current_state_done_;
}

bool ExcavatorGoToRepairStation::hasSucceeded()
{
   last_state_succeeded_ = (navigation_vision_result_.result == COMMON_RESULT::SUCCESS);
   // if(last_state_succeeded_)
   //    ROS_WARN_STREAM("Excavator Go to Repair Station Completed Successfully");
   return last_state_succeeded_;
}

void ExcavatorGoToRepairStation::step()
{

   if (first_)
   {
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_REPAIR_STATION_CLASS;
      navigation_vision_goal_.mode = V_REACH;
      // navigation_vision_goal_.target_loc = target_loc_;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      first_ = false;
   }
   ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: Going to repair station Step Function!");
}

void ExcavatorGoToRepairStation::exitPoint()
{
   // none at the moment
   navigation_vision_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  G O  TO  S C O U T  R E C O V E R Y  S T A T E  C L A S S /////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ExcavatorGoToScoutRecovery::entryPoint()
{
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: State Machine: Entrypoint of GoToScoutRecovery.");
   // define the offset distance for generating poses
   search_offset_ = 1.0;
   // set up the four recovery poses
   createPoses();
   // reset the pose that is going to be checked
   pose_index_ = 0;

   cross_end_complete_ = false;
   search_done_ = false;
   first_ = true;
   search_failed_ = false;
   substate_ = 1;
}

/** LOGIC: Since step() runs in a loop, flags have to be checked for the sequence to be implemented correctly. 
 * if(first_) ==> goToCrossEnd
 * if(crossend DONE) ==> SearchForScout()
 * if(search successful) ==> last_state_succeeded ==> EXIT
 * if(search failed) ==> (first_ = true) ==> pose_index_++ ==> cross_end_completed set to FALSE
 * if(pose_index > 4) ==> last_state_succeeded = false
 * */

void ExcavatorGoToScoutRecovery::step() 
{
   // Sending goal of going to one of the stored poses.
   if(first_ && (substate_ == 1))
   {
      goToCrossEnd(pose_index_);
      first_ = false;
   }

   // Sending goal of searching for scout.
   if(first_ && (substate_ == 2))
   {
      searchForScout();
      ROS_INFO_STREAM("searching for the scout");
      first_ = false;
   }

   // check for completion of states
   if(!cross_end_complete_ && (substate_ == 1))
      cross_end_complete_ = navigation_client_->getState().isDone();
   if(!search_done_ && (substate_ == 2))
      search_done_ = navigation_vision_client_->getState().isDone();

   // If excavator reaches one of the stored poses, set flags that will enable sending goal of searching for scout
   if(cross_end_complete_ && (substate_ == 1))
   {
      substate_ = 2;
      first_ = true;
   }

   // Checking to see if search fails, if it does we update the excavator to go to the next recovery pose.
   if(search_done_ && (substate_ == 2))
      search_failed_ = (navigation_vision_result_.result == COMMON_RESULT::FAILED);

   // Here is where we update the recovery pose, set the conditions for going to one of those stored poses and 
   // tell that the cross end is incomplete so that it checks for completion of the new recovery pose. 
   if(search_failed_ && (pose_index_ < 4) && (substate_ == 2)) {
      first_ = true;
      cross_end_complete_ = false;
      substate_ = 1;
      pose_index_ ++;
      search_failed_ = false;
      search_done_ = false;
   }
   scout_found_ = (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   searches_exhausted_= (pose_index_ > 3);
}

// If the excavator finds the scout or all the recovery_poses_ are exhausted, this recovery state is done.
bool ExcavatorGoToScoutRecovery::isDone()
{
   current_state_done_ = (searches_exhausted_ || scout_found_);
   return current_state_done_;
}

// If the excavator finds the scout, state succeeds. 
bool ExcavatorGoToScoutRecovery::hasSucceeded()
{
   // succeeds if scout is found successfully 
   last_state_succeeded_ = scout_found_;
   return last_state_succeeded_;
}

void ExcavatorGoToScoutRecovery::exitPoint()
{
   navigation_vision_client_->cancelGoal();
   navigation_client_->cancelGoal();
}

// Excavator goes to X-Y offsets defined at the entry point from the pose of the excavator at the entry point. The index correspons to the index of the poses stored at that time.
void ExcavatorGoToScoutRecovery::goToCrossEnd(int index)
{
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: State Machine: Going to cross-end");
   navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
   ROS_INFO_STREAM("POSE INDEX = " << index);
   ROS_INFO_STREAM("Going to recovery pose : " << recovery_poses_[index]);
   navigation_action_goal_.pose = recovery_poses_[index];
   navigation_client_->sendGoal(navigation_action_goal_);
}

// Excavator searches for scout once it gets to one of those offset poses. 
void ExcavatorGoToScoutRecovery::searchForScout() 
{
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: State Machine: Going to scout");
   navigation_vision_goal_.desired_object_label = COMMON_NAMES::OBJECT_DETECTION_SCOUT_CLASS;
   navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_NAV_AND_NAV_VISION;
   target_loc_ = m_pcRobotScheduler->getDesiredPose();
   navigation_vision_goal_.goal_loc = target_loc_;
   navigation_vision_client_->sendGoal(navigation_vision_goal_);
   ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: Excavator State Machine: SUCCESSFUL POSE RECEIVED");
}

// When the excavator's Recovery method is triggered, it creates 4 offset poses from its current location and stores them in recovery_poses_
void ExcavatorGoToScoutRecovery::createPoses()
{
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]: State Machine: Creating poses.");
   
   recovery_pose_ = excavator_pose_;
   recovery_pose_.pose.position.x += search_offset_;
   recovery_poses_[0] = recovery_pose_;

   recovery_pose_ = excavator_pose_;
   recovery_pose_.pose.position.x -= search_offset_;
   recovery_poses_[1] = recovery_pose_;

   recovery_pose_ = excavator_pose_;
   recovery_pose_.pose.position.y += search_offset_;
   recovery_poses_[2] = recovery_pose_;

   recovery_pose_ = excavator_pose_;
   recovery_pose_.pose.position.y -= search_offset_;
   recovery_poses_[3] = recovery_pose_;

   ROS_INFO_STREAM("Poses created: " << recovery_poses_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  V O L A T I L E  R E C O V E R Y  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void ExcavatorGoToRepairStation::entryPoint()
// {
//    first_ = true;
// }

// bool ExcavatorGoToRepairStation::isDone()
// {
//    current_state_done_ = navigation_vision_client_->getState().isDone();
//    return current_state_done_;
// }

// bool ExcavatorGoToRepairStation::hasSucceeded()
// {
//    last_state_succeeded_ = (navigation_vision_result_.result == COMMON_RESULT::SUCCESS);
//    // if(last_state_succeeded_)
//    //    ROS_WARN_STREAM("Excavator Go to Repair Station Completed Successfully");
//    return last_state_succeeded_;
// }

// void ExcavatorGoToRepairStation::step()
// {

//    if (first_)
//    {
//       navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_REPAIR_STATION_CLASS;
//       navigation_vision_goal_.mode = V_REACH;
//       // navigation_vision_goal_.target_loc = target_loc_;
//       navigation_vision_client_->sendGoal(navigation_vision_goal_);
//       first_ = false;
//    }
//    ROS_INFO_STREAM("STATE_MACHINES | excavator_state_machine | " << robot_name_ << " ]: Going to repair station Step Function!");
// }

// void ExcavatorGoToRepairStation::exitPoint()
// {
//    // none at the moment
//    navigation_vision_client_->cancelGoal();
// }