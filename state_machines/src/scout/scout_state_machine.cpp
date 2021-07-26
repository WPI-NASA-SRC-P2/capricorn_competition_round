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

  SCOUT_1_RETURN_LOC.header.frame_id = COMMON_NAMES::MAP;
  SCOUT_1_RETURN_LOC.pose.position.x = 10.0;
  SCOUT_1_RETURN_LOC.pose.position.y = 5.0;
  SCOUT_1_RETURN_LOC.pose.orientation.z = 1.0;

  SCOUT_2_RETURN_LOC.header.frame_id = COMMON_NAMES::MAP;
  SCOUT_2_RETURN_LOC.pose.position.x = -20.0;
  SCOUT_2_RETURN_LOC.pose.position.y = -5;
  SCOUT_2_RETURN_LOC.pose.orientation.z = 1.0;

  odom_sub_ = nh_.subscribe("/" + robot_name_ + RTAB_ODOM_TOPIC, 10, &ScoutState::odomCallback, this);
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

void ScoutState::odomCallback(const nav_msgs::Odometry odom)
{
   odom_ = odom;
   // excavator_pose_ = odom.pose.pose;
   scout_pose_.pose = odom.pose.pose;
   scout_pose_.header = odom.header;
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// U N D O C K   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Undock::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   first_ = true;
   state_done_ = false;
   state_success_ = false;
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Entrypoint of undock");   
}

bool Undock::isDone()
{
   // update the status of current state
   // current_state_done_ = navigation_vision_client_->getState().isDone();
   current_state_done_ = state_done_;
   return current_state_done_;
}

bool Undock::hasSucceeded()
{
   // update the status of current state
   // last_state_succeeded_ = (navigation_client_->getState() == actionlib::status::SUCCESS);
   // last_state_succeeded_ = (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   last_state_succeeded_ = state_success_;
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
   // update whether the state is done and is successful
   state_done_ = navigation_vision_client_->getState().isDone();
   if(state_done_ && !(first_))
      state_success_ = (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
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
   current_state_done_ = false;
   last_state_succeeded_ = false;
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout is beginning to search for volatile");
   covered_waypoint_sub = nh_.subscribe(CAPRICORN_TOPIC + robot_name_ + SPIRAL_WAYPOINT_PUBLISHER, 1000, &Search::waypointsCoveredCB, this);
   waypoints_covered_yet = 0;
}

void Search::waypointsCoveredCB(std_msgs::UInt8 msg)
{
   total_waypoints_covered = msg.data;
   waypoints_covered_yet++;
}

bool Search::isDone()
{
   // if near the volatile, then the state is done
   // ROS_INFO_THROTTLE(3, "[ STATE MACHINES | scout_state_machine | %s ]: %i Waypoints covered by now", robot_name_.c_str(), waypoints_covered_yet);
   // ROS_INFO_THROTTLE(3, "[ STATE MACHINES | scout_state_machine | %s ]: reset needed or not: %i", robot_name_.c_str(), (waypoints_covered_yet >= MAX_WAYPOINTS_BEFORE_RESET));
   // if(current_state_done_) {   /** @TEST: Remove thsi if running the scheduler */
   // srv.request.resume_spiral_motion = false;
   // spiralClient_.call(srv);
   // }

   return (current_state_done_);
}

bool Search::hasSucceeded()
{
   // if near the volatile, then the state has succeeded
   return last_state_succeeded_;
}

void Search::step()
{
   // execute spiral motion
   spiralClient_.call(srv);
   current_state_done_ = (near_volatile_ || (waypoints_covered_yet >= MAX_WAYPOINTS_BEFORE_RESET));
   last_state_succeeded_ = near_volatile_;
   ros::spinOnce();
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
   state_done_ = false;
   state_success_ = false;
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout is beginning to locate volatile");

}

bool Locate::isDone()
{
   // switch states once completed with locating the volatile
   // current_state_done_ = (resource_localiser_client_->getState().isDone());
   current_state_done_ = state_done_;
   return current_state_done_;
}

bool Locate::hasSucceeded()
{
   // state succeeded once rover is parked on top of volatile
   // last_state_succeeded_ = ((resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && near_volatile_);
   // last_state_succeeded_ = (resource_localiser_client_->getState().isDone());
   last_state_succeeded_ = state_success_;
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
   // update whether the state is done and is successful
   state_done_ = (resource_localiser_client_->getState().isDone());
   if(state_done_ && !(first_))
      state_success_ = (resource_localiser_client_->getState().isDone());
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


void ResetOdomAtHopper::entryPoint()
{
   first_GTPP = true;                   // Go to Proc plant
   first_GTPPR = true;                  // Go to Proc plant Recovery
   second_GTPPR = true;                 // Go to Proc plant Recovery
   first_PAH = true;                    // Park at hopper
   first_UFH = true;                    // Undock from hopper
   resetOdomDone_ = false;
   micro_state = GO_TO_PROC_PLANT;      // First micro-state
   macro_state_succeeded = false;       // Macro-state success flag
   macro_state_done = false;            // Macro-state done flag
   state_done =false;

   // Setting poses
   // Currently not caring about orientations
   GTPP_pose_ = scout_pose_;         // Go to Proc plant Recovery pose (supposedly getting to this pose will enable 'seeing' the Proc plant as it is assumed the                                       // rover is at the left of repair station and hence cant see it. If its in a crater, hopefully travelling this 10m will get it out of it.)         
   GTPP_pose_.pose.position.x += 10.0;

}

bool ResetOdomAtHopper::isDone()
{
   // switch states once completed with locating the volatile
   current_state_done_ = macro_state_done;
   return current_state_done_;
}

bool ResetOdomAtHopper::hasSucceeded()
{
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
   case GO_TO_PROC_PLANT_RECOVERY:
      goToProcPlantRecovery();
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
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
      navigation_vision_goal_.mode = V_REACH;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Going to Processing Plant vision goal sent");
      first_GTPP = false;
      return;
   }

   bool is_done = (navigation_vision_client_->getState().isDone());
   bool has_succeeded = (navigation_vision_client_->getResult()->result == COMMON_RESULT::SUCCESS);
   if(is_done)
   {
      if(has_succeeded)
         micro_state = PARK_AT_HOPPER;
      else
      {
         micro_state = GO_TO_PROC_PLANT_RECOVERY;
         first_GTPPR = true;
         second_GTPPR = true;
      }
         
   }
}

void ResetOdomAtHopper::goToProcPlantRecovery()
{
   //Send for centering first. 
   if(first_GTPPR)
   {
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_REPAIR_STATION_CLASS;
      navigation_vision_goal_.mode = V_CENTER;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Centering to Repair Station vision goal sent");  
      first_GTPPR = false;
      return;
   }

   bool centering_done = (navigation_vision_client_->getState().isDone());
   bool is_done = false;

   // Once centering completed move 10 metres to the right of repair station. 
   if(centering_done)
   {
      if(second_GTPPR)
      {
         navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
         navigation_action_goal_.pose = GTPP_pose_;
         navigation_client_->sendGoal(navigation_action_goal_);
          ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Travelling to right of Repair Station in order to see Procesing plant: GOAL : " << GTPP_pose_);
         second_GTPPR = false;
         return;
      }
      else 
         is_done = navigation_client_->getState().isDone();   
   }
   if(is_done)
   {
      micro_state = GO_TO_PROC_PLANT;
      first_GTPP = true;
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
      if (park_robot_client_->getResult()->result == COMMON_RESULT::SUCCESS){
         first_UFH = true;
         micro_state = UNDOCK_FROM_HOPPER;  
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
   resetOdomDone_ = resetOdometryClient.call(srv);
   macro_state_succeeded = true;   // If the macro-state has reached all the way here, it deserves a success!
   macro_state_done = true;
   micro_state = SCOUT_IDLE;
   return;
}

void ResetOdomAtHopper::exitPoint()
{
   // none at the moment
   navigation_client_->cancelGoal();
   navigation_vision_client_->cancelGoal();
   park_robot_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  G O  TO  R E P A I R  S T A T I O N  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GoToRepairStation::entryPoint()
{
   first_GTR  = true;
   first_GTRR = true;
   second_GTRR = true;
   macro_state_done_ = false;
   macro_state_succeeded_ = false;
   GTRL_pose_ = (robot_name_ == COMMON_NAMES::SCOUT_1_NAME) ? SCOUT_1_RETURN_LOC : SCOUT_2_RETURN_LOC;
   GTRR_pose_.header.frame_id = robot_name_ + ROBOT_BASE;
   GTRR_pose_.pose.position.y = -10.0; 
   GTRR_pose_.pose.orientation.w = 1;
   micro_state = GO_TO_REPAIR;
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout entering goToRepairStation state");
}

bool GoToRepairStation::isDone()
{
   current_state_done_ = macro_state_done_;
   return current_state_done_;
}

bool GoToRepairStation::hasSucceeded()
{
   last_state_succeeded_ = macro_state_succeeded_;
   return last_state_succeeded_;
}

void GoToRepairStation::step()
{
   switch (micro_state)
   {
   case GO_TO_REPAIR: 
      goToRepair();
      break;
   case GO_TO_REPAIR_RECOVERY:
      goToRepairRecovery();
      break;
   case SCOUT_IDLE:
      idleScout();
      break;
   default:
      break;
   }
}

void GoToRepairStation::goToRepair()
{
   if(first_GTR)
   {
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_REPAIR_STATION_CLASS;
      navigation_vision_goal_.mode = V_NAV_AND_NAV_VISION;
      navigation_vision_goal_.goal_loc = GTRL_pose_;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Going to repair station vision goal sent");  
      first_GTR = false;
      return;
   }
   
   bool is_done = (navigation_vision_client_->getState().isDone());
   bool has_succeeded = (navigation_vision_client_->getResult()->result == COMMON_RESULT::SUCCESS);
   if(is_done)
   {
      if(!has_succeeded)
      {
         micro_state = GO_TO_REPAIR_RECOVERY;
         first_GTRR = true;
         second_GTRR = true;
      }
      else
      {
         macro_state_succeeded_ = true;
         macro_state_done_ = true;
      }          
   }
}

void GoToRepairStation::goToRepairRecovery()
{
   //Send for centering first. 
   if(first_GTRR)
   {
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
      navigation_vision_goal_.mode = V_CENTER;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Centering to Proc Plant vision goal sent");  
      first_GTRR = false;
      return;
   }

   bool centering_done = (navigation_vision_client_->getState().isDone());
   bool is_done = false;

   // Once centering completed move 10 metres to the right. 
   if(centering_done)
   {
      if(second_GTRR)
      {
         navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
         navigation_action_goal_.pose = GTRR_pose_;
         navigation_client_->sendGoal(navigation_action_goal_);
         ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Travelling to right of Processing Plant : GOAL : " << GTRR_pose_);
         second_GTRR = false;
         return;
      }
      else 
         is_done = navigation_client_->getState().isDone();   
   }
   if(is_done)
   {
      micro_state = GO_TO_REPAIR;
      first_GTR = true;
   }
      
}

void GoToRepairStation::exitPoint()
{
   // none at the moment
   navigation_vision_client_->cancelGoal();
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Scout finished going to repair station (exitpoint)");
}

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
   state_done_ = false;
   state_success_ = false;
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
   // update whether the state is done and is successful
   state_done_ = navigation_client_->getState().isDone();
   if(state_done_ && !(first_))
      state_success_ = (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ScoutGoToLoc::isDone() {
   // current_state_done_ = navigation_client_->getState().isDone();
   current_state_done_ = state_done_;
   return current_state_done_;
} 

bool ScoutGoToLoc::hasSucceeded() {
   // if(isDone() && !(first_))
   //    last_state_succeeded_ = (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   last_state_succeeded_ = state_success_;
   return last_state_succeeded_;
}

void ScoutGoToLoc::exitPoint()
{
    // clean up (cancel goals)
    navigation_client_->cancelGoal();
    ROS_INFO_STREAM("STATE_MACHINES | scout_state_machine | " << robot_name_ << " ]: Reached scout, preparing to park");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// V I S U A L  R E S E T  O D O M  S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void VisualResetOfOdometry::entryPoint()
{
   // declare entrypoint variables
   ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: State Machine: Going to do visual odom-reset"); 
   proc_plant_distance_ = 0.0;
   repair_station_distance_ = 0.0;
   no_of_measurements_ = 20;
   MAX_TRIES = 300;
   first_ = true;
   macro_state_done_ = false;
   macro_state_succeeded_ = false;

   micro_state = CENTER_TO_PROC_PLANT;
}

void VisualResetOfOdometry::step()
{
   switch (micro_state)
   {
   case CENTER_TO_PROC_PLANT: 
      centerToObject(OBJECT_DETECTION_PROCESSING_PLANT_CLASS);
      break;
   case GET_PROC_PLANT_DISTANCE:
      proc_plant_distance_ = getObjectDepth(OBJECT_DETECTION_PROCESSING_PLANT_CLASS);
      break;
   case CENTER_TO_REPAIR_STATION:
      centerToObject(OBJECT_DETECTION_REPAIR_STATION_CLASS);
      break;
   case GET_REPAIR_STATION_DISTANCE:
      repair_station_distance_ = getObjectDepth(OBJECT_DETECTION_REPAIR_STATION_CLASS);
      break;
   case CALL_RESET:
      visualResetOdom();
      break;
   case SCOUT_IDLE:
      idleScout();
      break;
   default:
      break;
   }
}

bool VisualResetOfOdometry::isDone()
{
   current_state_done_ = macro_state_done_;
   return current_state_done_;
}

bool VisualResetOfOdometry::hasSucceeded()
{
   last_state_succeeded_ = macro_state_succeeded_;
   return last_state_succeeded_;
}

void VisualResetOfOdometry::exitPoint()
{
   navigation_vision_client_->cancelGoal();
}

void VisualResetOfOdometry::centerToObject(const std::string& centering_object)
{
   if(first_)
   {
      navigation_vision_goal_.desired_object_label = centering_object;
      navigation_vision_goal_.mode = V_CENTER;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      ROS_INFO_STREAM("[STATE_MACHINES | scout_state_machine.cpp | " << robot_name_ << "]: Centering to " << centering_object << " vision goal sent"); 
      first_ = false;
   }

   // If centering is done, check if it has succeeded. If it does, we proceed to obtain its depth and orientation
   // If it doesnt succeed, we instead try to center to repair station.
   bool is_done = navigation_vision_client_->getState().isDone();
   bool has_succeeded = false;
   if(is_done)
   {
      has_succeeded = (navigation_vision_client_->getResult()->result == COMMON_RESULT::SUCCESS);
      if(has_succeeded)
      {
         if(centering_object == OBJECT_DETECTION_PROCESSING_PLANT_CLASS)
         {
            proc_plant_orientation_ = odom_.pose.pose.orientation;
            micro_state = GET_PROC_PLANT_DISTANCE;
         }
            
         else
         {
            repair_station_orientation_ = odom_.pose.pose.orientation;
            micro_state = GET_REPAIR_STATION_DISTANCE;
         }
      }
         
      else
      {
         if(centering_object == OBJECT_DETECTION_PROCESSING_PLANT_CLASS)
            micro_state = CENTER_TO_REPAIR_STATION;
         else
            micro_state = CALL_RESET;
      }         
      first_ = true;
   }  
}

float VisualResetOfOdometry::getObjectDepth(const std::string& centering_object) 
{
   // Try to get an image with the proc_plant in it, tries ==> no_of_attempts, reading_count ==> no_of_time we actually get a reading.
   int reading_count = 0, tries = 0;
   float sum_of_all_readings;
   while(ros::ok() && ((reading_count < no_of_measurements_) && (tries < MAX_TRIES))) // Can get stuck 
   {
      ros::spinOnce();   // Update object detection callback
      tries++;
      if(!objects_msg_received_)
      {
         ros::Duration(0.01).sleep();
         continue;
      }
      objects_msg_received_ = false;

      for(int i{}; i < vision_objects_->obj.size() ; i++ )
      {
         if(vision_objects_->obj.at(i).label == centering_object)
         {
            reading_count++;
            float camera_depth = vision_objects_->obj.at(i).point.pose.position.z;
            sum_of_all_readings += (std::sqrt(std::pow(camera_depth, 2) - std::pow(camera_offset_, 2)) + camera_offset_); 
            // ROS_INFO_STREAM("STATE_MACHINES | scout_state_machine | " << robot_name_ << " ]: Distance to " << centering_object << " is " << (sum_of_all_readings/reading_count));
            // ROS_INFO_STREAM("Sum of all readngs : " << sum_of_all_readings << " and reading_count = " << reading_count);
         }
      }
   }

   if(centering_object == OBJECT_DETECTION_PROCESSING_PLANT_CLASS)
      micro_state = CENTER_TO_REPAIR_STATION;
   if(centering_object == OBJECT_DETECTION_REPAIR_STATION_CLASS)
      micro_state = CALL_RESET;

   if(reading_count == 0)
      return 0.0;
   else   
   {
      // ROS_INFO_STREAM("STATE_MACHINES | scout_state_machine | Final distance to "<< centering_object << " is "<< (sum_of_all_readings/reading_count));
      return (sum_of_all_readings/reading_count);
   }
   
}

void VisualResetOfOdometry::visualResetOdom()
{

   ros::ServiceClient resetOdometryClient = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
   maploc::ResetOdom srv;

   srv.request.target_robot_name = robot_name_;
   srv.request.visual_reset.visual_reset = true;
   srv.request.visual_reset. depth_pp = proc_plant_distance_;
   srv.request.visual_reset. depth_rs = repair_station_distance_;
   srv.request.visual_reset.orientation_pp = proc_plant_orientation_;
   srv.request.visual_reset.orientation_rs = repair_station_orientation_;
   srv.request.visual_reset.robot_orientation = odom_.pose.pose.orientation;
   
   // ROS_INFO_STREAM("STATE_MACHINES | scout_state_machine | Distance to proc_plant is : " << proc_plant_distance_);
   // ROS_INFO_STREAM("STATE_MACHINES | scout_state_machine | Distance to repair_station is : " << repair_station_distance_);

   resetOdomDone_ = resetOdometryClient.call(srv);  
   macro_state_done_ = true;
   macro_state_succeeded_ = resetOdomDone_;
   micro_state = SCOUT_IDLE;

   if(resetOdomDone_)
      ROS_INFO_STREAM("STATE_MACHINES | scout_state_machine | " << robot_name_ << " ]: Odom reset successful!");
   else 
      ROS_INFO_STREAM("STATE_MACHINES | scout_state_machine | " << robot_name_ << " ]: Reset didn't happen, good luck wherever you are going."); 
}