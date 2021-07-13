#include <state_machines/scout_state_machine.h>
#include <iostream>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S C O U T   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ScoutState::ScoutState(uint32_t un_id, ros::NodeHandle nh, std::string robot_name) :
    State(un_id, ToString("mystate_", un_id), nh, robot_name)
{
  robot_name_ = robot_name;
  // ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: " << robot_name_);
  // robot_state_status fields set to default values when state is constructed
  robot_current_state_ = un_id;
  current_state_done_ = false;
  last_state_succeeded_ = false;

   
  resource_localiser_client_ = new ResourceLocaliserClient_(CAPRICORN_TOPIC + robot_name + "/" + RESOURCE_LOCALISER_ACTIONLIB, true);
  /** @todo: FIX NAVIGATIONVISIONCLIENT TO BE CORRECT TOPIC */
  navigation_vision_client_ = new NavigationVisionClient(CAPRICORN_TOPIC + robot_name_ + "/" + robot_name_ + NAVIGATION_VISION_ACTIONLIB, true);
  navigation_client_ = new NavigationClient(CAPRICORN_TOPIC + robot_name_ + "/" + NAVIGATION_ACTIONLIB, true);
  park_robot_client_ = new ParkRobotClient(CAPRICORN_TOPIC + robot_name_ + "/" + robot_name_ + COMMON_NAMES::PARK_HAULER_ACTIONLIB, true);
  ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Waiting for the scout action servers...");
  navigation_vision_client_->waitForServer();
  navigation_client_->waitForServer();
  resource_localiser_client_->waitForServer();
  park_robot_client_->waitForServer();
  
  ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: All scout action servers started!");
  
  // spiral client for spiral motion
  ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: waiting for spiral client");
  spiralClient_ = nh_.serviceClient<operations::Spiral>(CAPRICORN_TOPIC + robot_name_ + "/" + SCOUT_SEARCH_SERVICE);
  spiralClient_.waitForExistence();
  ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Spiral client started");
  
  volatile_sub_ = nh_.subscribe("/" + robot_name_ + VOLATILE_SENSOR_TOPIC, 1000, &ScoutState::volatileSensorCB, this);
  objects_sub_ = nh_.subscribe(CAPRICORN_TOPIC + robot_name_ + OBJECT_DETECTION_OBJECTS_TOPIC, 1, &ScoutState::objectsCallback, this);
}

ScoutState::~ScoutState()
{
  delete resource_localiser_client_;
  delete navigation_vision_client_;
  delete navigation_client_;
  delete park_robot_client_;
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
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Entrypoint of undock");   
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
      ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Undocking from volatile");

      operations::NavigationVisionGoal navigation_vision_goal;
      navigation_vision_goal.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
      navigation_vision_goal.mode = COMMON_NAMES::NAV_VISION_TYPE::V_UNDOCK;
      navigation_vision_client_->sendGoal(navigation_vision_goal);
      first_ = false; 
      ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Undock stepping, first_ = false now");
   }
}

void Undock::exitPoint() 
{
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Exitpoint of undock, cancelling undock goal");
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
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout is beginning to search for volatile");
}

bool Search::isDone()
{
   // if near the volatile, then the state is done
   current_state_done_ = near_volatile_;

   // if(current_state_done_) {   /** @TEST: Remove thsi if running the scheduler */
   // srv.request.resume_spiral_motion = false;
   // spiralClient_.call(srv);
   // }

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
   spiralClient_.call(srv);
}

void Search::exitPoint()
{
   // cancel spiral motion 
   srv.request.resume_spiral_motion = false;
   spiralClient_.call(srv);
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: search volatile finished (at exitpoint(");

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  L O C A T E  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Locate::entryPoint()
{
   // we assume we are near the volatile
   near_volatile_ = true;
   first_ = true;
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout is beginning to locate volatile");

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
      ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Sending resource localization goal");
      first_ = false;
   }
}

void Locate::exitPoint()
{
   // none at the moment
   resource_localiser_client_->cancelGoal();
   near_volatile_ = false;
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Finished locating volatile (exitpoint reached)");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  R E S E T _ O D O M   C L A S S ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/** TODO: should be reseting at processing plant instead */
void ResetOdomAtHopper::entryPoint()
{
   // we assume we are near the volatile
   near_volatile_ = true;
   first_GTPP = true;
   first_PAH = true;
   first_UFH = true;
   first_GTR = true;
   micro_state = GO_TO_PROC_PLANT;
   macro_state_succeeded = false;
   macro_state_done = false;
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout beginning reset odom at hopper macrostate");
}

bool ResetOdomAtHopper::isDone()
{
   // switch states once completed with locating the volatile
   current_state_done_ = macro_state_done;
   return current_state_done_;
}

bool ResetOdomAtHopper::hasSucceeded()
{
   // state succeeded once rover is parked on top of volatile
   // last_state_succeeded_ = ((resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && near_volatile_);
   last_state_succeeded_ = macro_state_succeeded;
   return last_state_succeeded_;
}

void ResetOdomAtHopper::step()
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
   case SCOUT_IDLE:
      idleScout();
      break;
   default:
      break;
   }
}

void ResetOdomAtHopper::goToProcPlant()
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

void ResetOdomAtHopper::parkAtHopper()
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
      if (park_robot_client_->getResult()->result == COMMON_RESULT::SUCCESS)
         micro_state = UNDOCK_FROM_HOPPER;
      else
      {
         first_PAH = true;
         micro_state = PARK_AT_HOPPER;
      }
   }
}

void ResetOdomAtHopper::undockFromHopper()
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

void ResetOdomAtHopper::resetOdom()
{
   ros::ServiceClient resetOdometryClient = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
   maploc::ResetOdom srv;
   srv.request.target_robot_name = robot_name_;
   srv.request.at_hopper = true;
   resetOdometryClient.call(srv);
   // macro_state_done = true;
   micro_state = GO_TO_REPAIR_STATION;
   return;
}

//Go to repair station after resetting 
void ResetOdomAtHopper::goToRepair()
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
         micro_state = SCOUT_IDLE;
      // Dont find a reason it should fail,
   }
}

void ResetOdomAtHopper::exitPoint()
{
   // none at the moment
   navigation_vision_client_->cancelGoal();
   park_robot_client_->cancelGoal();
   near_volatile_ = false;
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout finished reseting odom at hopper (exitpoint)");
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  G O  TO  R E P A I R  S T A T I O N  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GoToRepairStation::entryPoint()
{
   first_ = true;
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout entering goToRepairStation state");
}

bool GoToRepairStation::isDone()
{
   current_state_done_ = navigation_vision_client_->getState().isDone();
   return current_state_done_;
}

bool GoToRepairStation::hasSucceeded()
{
   last_state_succeeded_ = (navigation_vision_result_.result == COMMON_RESULT::SUCCESS);
   if(last_state_succeeded_)
      ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout GoToRepairStation completed successfully");

   return last_state_succeeded_;
}

void GoToRepairStation::step()
{

   if (first_)
   {
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_REPAIR_STATION_CLASS;
      navigation_vision_goal_.mode = V_REACH;
      // navigation_vision_goal_.target_loc = target_loc_;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      first_ = false;
      ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Going to repair station vision goal sent");
   }
}

void GoToRepairStation::exitPoint()
{
   // none at the moment
   navigation_vision_client_->cancelGoal();
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout finished going to repair station (exitpoint)");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  P A R K  A T  R E P A I R  S T A T I O N  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void ParkAtRepairStation::entryPoint()
// {
//    first_ = true;
// }

// bool ParkAtRepairStation::isDone()
// {
//    current_state_done_ = park_robot_client_->getState().isDone();
//    return current_state_done_;
// }

// bool ParkAtRepairStation::hasSucceeded()
// {
//    last_state_succeeded_ = (park_robot_client_->getResult()->result == COMMON_RESULT::SUCCESS);
//    if(last_state_succeeded_)
//       ROS_WARN_STREAM("Scout Park at Repair Station Completed Successfully");
//    return last_state_succeeded_;
// }

// void ParkAtRepairStation::step()
// {

//    if (first_)
//    {
//       park_robot_goal_.hopper_or_excavator = OBJECT_DETECTION_REPAIR_STATION_CLASS;
//       park_robot_client_->sendGoal(park_robot_goal_);
//       first_ = false;
//    }
//    ROS_INFO_STREAM("Going to repair station Step Function!");
// }

// void ParkAtRepairStation::exitPoint()
// {
//    // none at the moment
//    park_robot_client_->cancelGoal();
// }

//////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// I D L E ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void IdleState::entryPoint()
{
   resource_localiser_client_->cancelGoal();
   navigation_vision_client_->cancelGoal();
   navigation_client_->cancelGoal();
   park_robot_client_->cancelGoal();
   
   operations::Spiral srv;
   srv.request.resume_spiral_motion = false;
   spiralClient_.call(srv);
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout has entered idle state, awaiting new state...");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O _ T O _ L O C   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ScoutGoToLoc::entryPoint()
{
    // declare entrypoint variables
    ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: State Machine: Go To Loc");
    // pose of the excavator, supposed to be provided by scheduler
    target_loc_ = m_pcRobotScheduler->getDesiredPose();    
    first_ = true;
}

void ScoutGoToLoc::step()
{
    // go to excavator using planner+vision goal
    if(first_)
    {
        navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
        navigation_action_goal_.pose = target_loc_;
        navigation_client_->sendGoal(navigation_action_goal_);
        navigation_client_->sendGoal(navigation_action_goal_);
        navigation_client_->sendGoal(navigation_action_goal_);
        navigation_client_->sendGoal(navigation_action_goal_);
        first_ = false;
    }
}

bool ScoutGoToLoc::isDone() {
   current_state_done_ = navigation_client_->getState().isDone();
   return current_state_done_;
} 

bool ScoutGoToLoc::hasSucceeded() {
   last_state_succeeded_ = (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   // if(last_state_succeeded_)
   //    ROS_WARN_STREAM("Go to Scout Completed Successfully");
   return last_state_succeeded_;
}

void ScoutGoToLoc::exitPoint()
{
    // clean up (cancel goals)
    navigation_client_->cancelGoal();
    ROS_INFO_STREAM("STATE_MACHINES | scout_state_machine | " << robot_name_ << " ]: Reached scout, preparing to park");
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
