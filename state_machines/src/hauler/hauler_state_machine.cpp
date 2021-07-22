#include <ros/ros.h>
#include <state_machines/hauler_state_machine.h>
#include <state_machines/RobotStateMachineTaskAction.h>
#include <actionlib/server/simple_action_server.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// H A U L E R   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

HaulerState::HaulerState(uint32_t un_id, ros::NodeHandle nh, std::string robot_name) :
    State(un_id, ToString("mystate_", un_id), nh, robot_name)
{
  /** @todo: FIX SO THAT CAN USE ANY EXCAVATOR NAME, SO NO NAMESPACE ISSUES */
  robot_name_ = robot_name;
//   ROS_WARN_STREAM("Hauler robot_name_ = " + robot_name_);
  robot_current_state_ = un_id;
  navigation_vision_client_ = new NavigationVisionClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + robot_name_ + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
  navigation_client_ = new NavigationClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);
  hauler_client_ = new HaulerClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + COMMON_NAMES::HAULER_ACTIONLIB, true);
  park_robot_client_ = new ParkRobotClient(COMMON_NAMES::CAPRICORN_TOPIC +  robot_name_ + "/" + robot_name_ + COMMON_NAMES::PARK_HAULER_ACTIONLIB, true); 
//   resetHaulerOdometryClient_ = nh.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);

  /** TODO:fix the common names PARK_HAULER, ALso 2 actions being published, 1 of them is running but not working.*/ 

  ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Waiting for the hauler action servers...");
  navigation_vision_client_->waitForServer();
  navigation_client_->waitForServer(); 
  hauler_client_->waitForServer(); 
  park_robot_client_->waitForServer(); 
  resetHaulerOdometryClient_.waitForExistence();
  ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: All hauler action servers started!");

  // Locations for haulers to go to in order to clear the traffic at the hopper, HAULER_2 parks near the hopper, HAULER_1 near the repair station. 
  HAULER_1_LOOKOUT_LOC.header.frame_id = COMMON_NAMES::MAP;
  HAULER_1_LOOKOUT_LOC.pose.position.x    = 5.0;
  HAULER_1_LOOKOUT_LOC.pose.position.y    = -5.0;
  HAULER_1_LOOKOUT_LOC.pose.orientation.z = 0.707;
  HAULER_1_LOOKOUT_LOC.pose.orientation.w = 0.707;

  HAULER_2_LOOKOUT_LOC.header.frame_id = COMMON_NAMES::MAP;
  HAULER_2_LOOKOUT_LOC.pose.position.x    = 10.0;
  HAULER_2_LOOKOUT_LOC.pose.position.y    = 15.0;
  HAULER_2_LOOKOUT_LOC.pose.orientation.z = -0.707;
  HAULER_2_LOOKOUT_LOC.pose.orientation.w = 0.707;

  HAULER_RETURN_LOC.header.frame_id = COMMON_NAMES::MAP;
  HAULER_RETURN_LOC.pose.position.x    = 10.0;
  HAULER_RETURN_LOC.pose.position.y    = 10.0;
  HAULER_RETURN_LOC.pose.orientation.w = 1;


  odom_sub_ = nh_.subscribe("/" + robot_name_ + RTAB_ODOM_TOPIC, 10, &HaulerState::odomCallback, this);

//   objects_sub_ = nh_.subscribe(CAPRICORN_TOPIC + robot_name_ + OBJECT_DETECTION_OBJECTS_TOPIC, 1, &HaulerState::objectsCallback, this);
}

HaulerState::~HaulerState()
{
  delete navigation_vision_client_;
  delete navigation_client_;
  delete hauler_client_ ;
  delete park_robot_client_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// C A L L B A C K S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HaulerState::odomCallback(const nav_msgs::Odometry odom)
{
   odom_ = odom;
   // excavator_pose_ = odom.pose.pose;
   hauler_pose_.pose = odom.pose.pose;
   hauler_pose_.header = odom.header;
   
}

/** TODO: Implememt entrypoint, transition, step and exit points for all states mentioned in the flow shown by ashay
    *   HAULER_GO_TO_LOC,                    //
    8-> HAULER_DUMP_VOLATILE_TO_PROC_PLANT, // 
    9   HAULER_GO_BACK_TO_EXCAVATOR,        // GoToExcavator
    9   HAULER_PARK_AT_EXCAVATOR,           // ParkExcavator
    10  HAULER_FOLLOW_EXCAVATOR,            // FollowExcavator
    3   HAULER_RESET_ODOM,                  // ResetOdom
    1   HAULER_GO_TO_PROC_PLANT, // GoToProcPlant
    2   HAULER_PARK_AT_HOPPER,   // ParkAtHopper
    7   HAULER_DUMP_VOLATILE,    // DumpVolatile                  
    6   HAULER_UNDOCK_EXCAVATOR, // UndockExcavator
    4   HAULER_UNDOCK_HOPPER,    // UndockHopper
    5-> HAULER_RESET_ODOM_AT_HOPPER, // ResetOdomMacro
   meh  HAULER_FACE_PROCESSING_PLANT
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O _ T O _ P R O C _ P L A N T   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GoToProcPlant::entryPoint()
{
    // setup initial variable values
    first_ = true;
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: State Machine: Going to Processing Plant");
    current_state_done_ = false;
    last_state_succeeded_ = false;
    target_loc_ = m_pcRobotScheduler->getDesiredPose();
}

State& GoToProcPlant::transition()
{
    // transition to HAULER_PARK_AT_HOPPER once completed 
    if(!(navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    {
        return *this;
    }
    return getState(HAULER_PARK_AT_HOPPER);
    // return *this;
}

void GoToProcPlant::step()
{
    // go to the processing plant
    if(first_)
    {
        navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
        navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_NAV_AND_NAV_VISION;
        navigation_vision_goal_.goal_loc = target_loc_;
        navigation_vision_client_->sendGoal(navigation_vision_goal_);
        // navigation_vision_client_->waitForResult();
        // return (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
        first_ = false;
        ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: going to processing plant");
    }

    // Updating done and succeeded in the same time step to avoid false failure detections
    current_state_done_ = navigation_vision_client_->getState().isDone();
    if(current_state_done_ && !(first_))
      last_state_succeeded_ = (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);     
}

bool GoToProcPlant::isDone() 
{
   return current_state_done_;
} 

bool GoToProcPlant::hasSucceeded() 
{
   return last_state_succeeded_;
}

void GoToProcPlant::exitPoint()
{
    // cleanup (cancel goals)
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Hauler arrived at processing plant, switching to HAULER_PARK_AT_HOPPER");
    navigation_vision_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O _ T O _ S C O U T   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HaulerGoToScout::entryPoint()
{
    // declare entrypoint variables
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  State Machine: Going to Scout (High Level Goal)");
    // pose of the excavator, supposed to be provided by scheduler
    target_loc_ = m_pcRobotScheduler->getDesiredPose();
    current_state_done_ = false;
    last_state_succeeded_ = false;
    first_ = true;
}

void HaulerGoToScout::step()
{
    // go to excavator using planner+vision goal
    if(first_)
    {
        
        navigation_vision_goal_.mode = NAV_VISION_TYPE::V_NAV_AND_NAV_VISION;
        navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_SCOUT_CLASS;
        navigation_vision_goal_.goal_loc = target_loc_;
        navigation_vision_client_->sendGoal(navigation_vision_goal_);
        first_ = false;
        ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:Moving towards excavator");
    }
    current_state_done_ = navigation_vision_client_->getState().isDone();
    if(current_state_done_ && !(first_))
      last_state_succeeded_ = (navigation_vision_client_->getResult()->result == COMMON_RESULT::SUCCESS);
}

bool HaulerGoToScout::isDone() 
{
   return current_state_done_;
} 

bool HaulerGoToScout::hasSucceeded() 
{
   if(last_state_succeeded_)
      ROS_WARN_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  Go to Scout Completed Successfully");
   return last_state_succeeded_;
}

void HaulerGoToScout::exitPoint()
{
    // clean up (cancel goals)
    navigation_vision_client_->cancelGoal();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  Reached scout, preparing to park");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// P A R K _ A T _ H O P P E R   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ParkAtHopper::entryPoint()
{
    // set entry variables
    first_ = true;
    current_state_done_ = false;
    last_state_succeeded_ = false;
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Parking to hopper");
}

State& ParkAtHopper::transition()
{
    // transition to reset odometry state when complete
    if(!(park_robot_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
        return *this;
    }
    return getState(HAULER_RESET_ODOM);
    // return *this;
}

void ParkAtHopper::step()
{
    if(first_)
    {
        // park at the hopper using vnav
        park_robot_goal_.hopper_or_excavator = OBJECT_DETECTION_HOPPER_CLASS;
        park_robot_client_->sendGoal(park_robot_goal_);
        // park_robot_client_->waitForResult();
        first_ = false;
        ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  Parking at hopper");
    }
   current_state_done_ = park_robot_client_->getState().isDone();
   if(current_state_done_ && !(first_))
      last_state_succeeded_ = (park_robot_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   
}

bool ParkAtHopper::isDone() 
{
   return current_state_done_;
} 

bool ParkAtHopper::hasSucceeded() 
{
   return last_state_succeeded_;
}

void ParkAtHopper::exitPoint()
{
    // cleanup (cancel goal)
    park_robot_client_->cancelGoal();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  State Machine: Finished parking at hopper");
    // // reset hauler odometry after finishing parking at the hopper
    // reset_srv_.request.target_robot_name = robot_name_;
    // reset_srv_.request.use_ground_truth = true;
    // ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " Hauler odometry reset service called!");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// U N D O C K _ H O P P E R   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void UndockHopper::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   first_ = true;
   ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  entrypoint of undock");
   reset_succeeded_ = false;
   undock_done_ = false;
   current_state_done_ = false;
   last_state_succeeded_ = false;
   // update the current status of the robot and publish it
   
   
}

bool UndockHopper::isDone()
{
   // update the status of current state
   return current_state_done_;
}

bool UndockHopper::hasSucceeded()
{
   // update the status of current state
   // last_state_succeeded_ = (navigation_client_->getState() == actionlib::status::SUCCESS);
//    last_state_succeeded_ = (navigation_vision_result_.result == COMMON_RESULT::SUCCESS);
   return last_state_succeeded_;
}

void UndockHopper::step()
{
   if(first_)
   {
      ROS_INFO_STREAM(robot_name_ << " State Machine: Undocking from volatile");
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
      navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_UNDOCK;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      first_ = false; 
      ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Undock stepping, first_ = false now");
   }
   // check if undocking is finished before reseting odometry
   undock_done_ = (navigation_vision_client_->getResult()->result == COMMON_RESULT::SUCCESS);
   current_state_done_ = undock_done_;
   last_state_succeeded_ = undock_done_;
   
   if(undock_done_ && !first_){
       resetOdom();
   }
}

void UndockHopper::exitPoint() 
{
   ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: exitpoint of undock, cancelling undock goal");
   navigation_vision_client_->cancelGoal();
}


void UndockHopper::resetOdom()
{
   ros::ServiceClient resetOdometryClient = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
   maploc::ResetOdom srv;
   srv.request.target_robot_name = robot_name_;
   srv.request.at_hopper = true;
   reset_succeeded_ = resetOdometryClient.call(srv);
//    micro_state = SCOUT_IDLE;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O _ T O _ E X C A V A T O R   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GoToExcavator::entryPoint()
{
    // declare entrypoint variables
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: State Machine: Going Back to Excavator (High Level Goal)");
    // pose of the excavator, supposed to be provided by scheduler
    target_loc_ = m_pcRobotScheduler->getDesiredPose();
    current_state_done_ = false;
    last_state_succeeded_ = false;
    first_ = true;
}

State& GoToExcavator::transition()
{
    // transition to parking at the excavator
    if(!(navigation_vision_result_.result == COMMON_RESULT::SUCCESS))
    {
        return *this;
    }
    return getState(HAULER_PARK_AT_EXCAVATOR);
}

void GoToExcavator::step()
{
    // go to excavator using planner+vision goal
    if(first_)
    {
        
        navigation_vision_goal_.mode = NAV_VISION_TYPE::V_NAV_AND_NAV_VISION;
        navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
        navigation_vision_goal_.goal_loc = target_loc_;
        navigation_vision_client_->sendGoal(navigation_vision_goal_);
        // navigation_vision_client_->waitForResult();
        first_ = false;
        ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Moving towards excavator");
    }
   current_state_done_ = navigation_vision_client_->getState().isDone();
   if(current_state_done_ && !(first_))
      last_state_succeeded_ = (navigation_vision_client_->getResult()->result == COMMON_RESULT::SUCCESS);    
}

bool GoToExcavator::isDone() 
{
   return current_state_done_;
} 

bool GoToExcavator::hasSucceeded() 
{
   return last_state_succeeded_;
}

void GoToExcavator::exitPoint()
{
    // clean up (cancel goals)
    navigation_vision_client_->cancelGoal();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  Reached excavator, preparing to park");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// P A R K _ A T _ E X C A V A T O R   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ParkAtExcavator::entryPoint()
{
    // set entry variables
    first_ = true;
    current_state_done_ = false;
    last_state_succeeded_ = false;
    park_robot_goal_.hopper_or_excavator = OBJECT_DETECTION_EXCAVATOR_CLASS;
    excavator_name_ = m_pcRobotScheduler->getTargetRobotName();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  State Machine: Parking at Excavator");
}

State& ParkAtExcavator::transition()
{
    // transition to reset odometry state when complete
    if(!(park_robot_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
        return *this;
    }
    return getState(HAULER_UNDOCK_EXCAVATOR);
    // return *this;
}

void ParkAtExcavator::step()
{
    if(first_)
    {
        // park at the excavator using vnav
        park_robot_goal_.hopper_or_excavator = excavator_name_;
        park_robot_client_->sendGoal(park_robot_goal_);
        // park_robot_client_->waitForResult();
        first_ = false;
        ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Parking at excavator");
    }

   current_state_done_ = park_robot_client_->getState().isDone();
   if(current_state_done_ && !(first_))
      last_state_succeeded_ = (park_robot_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool ParkAtExcavator::isDone() 
{
   return current_state_done_;
} 

bool ParkAtExcavator::hasSucceeded() 
{
   return last_state_succeeded_;
}

void ParkAtExcavator::exitPoint()
{
    // cleanup (cancel goal)
    park_robot_client_->cancelGoal();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: State Machine: Finished parking at excavator");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// U N D O C K  E X C A V A T O R  S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void UndockExcavator::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   first_ = true;
   ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  entrypoint of undock");
   current_state_done_ = false;
   last_state_succeeded_ = false;
   
   // update the current status of the robot and publish it
   
}

bool UndockExcavator::isDone()
{
   return current_state_done_;
}

bool UndockExcavator::hasSucceeded()
{

   return last_state_succeeded_;
}

void UndockExcavator::step()
{
   if(first_)
   {
      ROS_INFO_STREAM(robot_name_ << " State Machine: Undocking from volatile");
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
      navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_HARDCODED_UNDOCK;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      first_ = false; 
      ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  Undock stepping, first_ = false now");
   }

   current_state_done_ = navigation_vision_client_->getState().isDone();
   if(current_state_done_ && !(first_))
      last_state_succeeded_ = (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

void UndockExcavator::exitPoint() 
{
   ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  exitpoint of undock, cancelling undock goal");
   navigation_vision_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// D U M P   V O L A T I L E ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DumpVolatile::entryPoint()
{
    // set entry variables
    first_ = true;
    current_state_done_ = false;
    last_state_succeeded_ = false;
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: State Machine: Dumping Volatile");
}

State& DumpVolatile::transition()
{
    // transition to WHATEVER when state is complete
    if(!(hauler_client_->getState().isDone())){ 
        return *this;
    }
    return getState(HAULER_UNDOCK_HOPPER);  //Change this
    // return *this;
}

void DumpVolatile::step()
{
    if(first_)
    {   
        hauler_goal_.desired_state = true;
        hauler_client_->sendGoal(hauler_goal_);
        // hauler_client_->waitForResult();
        first_ = false;
        ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  Dumping volatile");
    }

   current_state_done_ = hauler_client_->getState().isDone();
   if(current_state_done_ && !(first_))
      last_state_succeeded_ = (hauler_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool DumpVolatile::isDone() 
{
   return current_state_done_;
} 

bool DumpVolatile::hasSucceeded() 
{
   return last_state_succeeded_;
}

void DumpVolatile::exitPoint()
{
    // cleanup (cancel goal)
    hauler_client_->cancelGoal();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: State Machine: Finished dumping volatile");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O _ T O _ L O C   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HaulerGoToLoc::entryPoint()
{
    // declare entrypoint variables
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: State Machine: Go To Loc");
    // pose of the excavator, supposed to be provided by scheduler
    target_loc_ = m_pcRobotScheduler->getDesiredPose();    
    first_ = true;
    last_state_succeeded_ = false;
}

void HaulerGoToLoc::step()
{
    // go to excavator using planner+vision goal
    if(first_)
    {
        navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
        navigation_action_goal_.pose = target_loc_;
        navigation_client_->sendGoal(navigation_action_goal_);
        first_ = false;
    }
   current_state_done_ = navigation_client_->getState().isDone();
   if(current_state_done_ && !(first_))
      last_state_succeeded_ = (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerGoToLoc::isDone() 
{
   return current_state_done_;
} 

bool HaulerGoToLoc::hasSucceeded() 
{
   if(last_state_succeeded_)
      ROS_WARN_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  Go to Scout Completed Successfully");
   return last_state_succeeded_;
}

void HaulerGoToLoc::exitPoint()
{
    // clean up (cancel goals)
    navigation_client_->cancelGoal();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  Reached scout, preparing to park");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  R E S E T _ O D O M   C L A S S ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void DumpVolatileAtHopper::entryPoint()
{
   // we assume we are near the volatile
   first_GTPP = true;
   first_PAH = true;
   first_UFH = true;
   first_GTR = true;
   first_GTRR = true;
   first_DV = true;
   second_GTRR = true;
   first_GTLL = true;
   resetOdomDone_ = false;
   micro_state = GO_TO_PROC_PLANT;
   macro_state_succeeded = false;
   macro_state_done = false;
   state_done =false;

   // Setting poses
   hardcoded_pose_ = (robot_name_ == COMMON_NAMES::HAULER_1_NAME) ? HAULER_1_LOOKOUT_LOC : HAULER_2_LOOKOUT_LOC;
   
   // Currently not caring about orientations
   GTPP_pose_ = hauler_pose_;
   GTPP_pose_.pose.position.x += 10.0;
}

bool DumpVolatileAtHopper::isDone()
{
   // switch states once completed with locating the volatile
   current_state_done_ = macro_state_done;
   return current_state_done_;
}

bool DumpVolatileAtHopper::hasSucceeded()
{
   // state succeeded once rover is parked on top of volatile
   // last_state_succeeded_ = ((resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && near_volatile_);
   last_state_succeeded_ = macro_state_succeeded;
   return last_state_succeeded_;
}

void DumpVolatileAtHopper::step()
{
   switch (micro_state)
   {
   case GO_TO_PROC_PLANT: 
      goToProcPlant();
      break;
   case PARK_AT_HOPPER:
      parkAtHopper();
      break;
   case DUMP_VOLATILE:
      dumpVolatile();
      break;
   case UNDOCK_FROM_HOPPER:
      undockFromHopper();
      break;
   case RESET_ODOM_AT_HOPPER:
      resetOdom();
      break;
   case GO_TO_LOOKOUT_LOCATION:
      goToLookoutLocation();
      break;
   case GO_TO_PROC_PLANT_RECOVERY:
      goToProcPlantRecovery();
      break;
   case HAULER_IDLE:
      idleHauler();
      break;
   default:
      break;
   }
}

void DumpVolatileAtHopper::goToProcPlant()
{
   if (first_GTPP)
   {
      operations::NavigationVisionGoal navigation_vision_goal;
      navigation_vision_goal.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
      navigation_vision_goal.mode = V_NAV_AND_NAV_VISION;
      navigation_vision_goal_.goal_loc = HAULER_RETURN_LOC;
      navigation_vision_client_->sendGoal(navigation_vision_goal);
      ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Going to Processing Plant vision goal sent");
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

void DumpVolatileAtHopper::goToProcPlantRecovery()
{
   //Send for centering first. 
   if(first_GTPPR)
   {
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_REPAIR_STATION_CLASS;
      navigation_vision_goal_.mode = V_CENTER;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Centering to Repair Station vision goal sent");  
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
          ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Travelling to right of Repair Station : GOAL : " << GTPP_pose_);
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

void DumpVolatileAtHopper::parkAtHopper()
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
         micro_state = DUMP_VOLATILE;  
      }
   }
}

void DumpVolatileAtHopper::dumpVolatile()
{
    if (first_DV)
    {
        hauler_goal_.desired_state = true;
        hauler_client_->sendGoal(hauler_goal_);
        first_DV = false;
    }
       
    bool is_done = (hauler_client_->getState().isDone());
    if (is_done)
    {
        if (park_robot_client_->getResult()->result == COMMON_RESULT::SUCCESS)
            micro_state = UNDOCK_FROM_HOPPER;
        else
        {
            micro_state = DUMP_VOLATILE;
        }
    }
}

void DumpVolatileAtHopper::undockFromHopper()
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

void DumpVolatileAtHopper::resetOdom()
{
   ros::ServiceClient resetOdometryClient = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
   maploc::ResetOdom srv;
   srv.request.target_robot_name = robot_name_;
   srv.request.at_hopper = true;
   resetOdomDone_ = resetOdometryClient.call(srv);
   // macro_state_succeeded = resetOdometryClient.call(srv);
   // macro_state_done = true;
   micro_state = GO_TO_LOOKOUT_LOCATION;
   return;
}

void DumpVolatileAtHopper::goToLookoutLocation() 
{
   if(first_GTLL)
   {
   navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
   navigation_action_goal_.pose = hardcoded_pose_;
   navigation_client_->sendGoal(navigation_action_goal_);
   ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << "]:  Going to Lookout Location : " << hardcoded_pose_);
   first_GTLL = false;
   }
   
   bool is_done = (navigation_client_->getState().isDone());
   if (is_done)
   {
      macro_state_done = true;
      macro_state_succeeded = true;
      micro_state = HAULER_IDLE;
      // Dont find a reason it should fail,
   }
}

void DumpVolatileAtHopper::exitPoint()
{
   // none at the moment
   navigation_client_->cancelGoal();
   navigation_vision_client_->cancelGoal();
   park_robot_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  G O  TO  R E P A I R  S T A T I O N  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HaulerGoToRepairStation::entryPoint()
{
   first_GTR  = true;
   first_GTRR = true;
   second_GTRR = true;
   macro_state_done_ = false;
   macro_state_succeeded_ = false;
   GTRL_pose_ = HAULER_RETURN_LOC;
   GTRR_pose_ = hauler_pose_;
   GTRR_pose_.pose.position.x -= 10.0;
   micro_state = GO_TO_REPAIR;
   ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Hauler entering goToRepairStation state");
}

bool HaulerGoToRepairStation::isDone()
{
   current_state_done_ = macro_state_done_;
   return current_state_done_;
}

bool HaulerGoToRepairStation::hasSucceeded()
{
   last_state_succeeded_ = macro_state_succeeded_;
   return last_state_succeeded_;
}

void HaulerGoToRepairStation::step()
{
   switch (micro_state)
   {
   case GO_TO_REPAIR: 
      goToRepair();
      break;
   case GO_TO_REPAIR_RECOVERY:
      goToRepairRecovery();
      break;
   case HAULER_IDLE:
      idleHauler();
      break;
   default:
      break;
   }
}

void HaulerGoToRepairStation::goToRepair()
{
   if(first_GTR)
   {
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_REPAIR_STATION_CLASS;
      navigation_vision_goal_.mode = V_NAV_AND_NAV_VISION;
      navigation_vision_goal_.goal_loc = GTRL_pose_;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Going to repair station vision goal sent");  
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
         macro_state_done_ = true;
         macro_state_succeeded_ = true;
      }     
   }
}

void HaulerGoToRepairStation::goToRepairRecovery()
{
   //Send for centering first. 
   if(first_GTRR)
   {
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_PROCESSING_PLANT_CLASS;
      navigation_vision_goal_.mode = V_CENTER;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Centering to Proc Plant vision goal sent");  
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
         ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Travelling to right of Processing Plant : GOAL : " << GTRR_pose_);
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

void HaulerGoToRepairStation::exitPoint()
{
   // none at the moment
   navigation_vision_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  G O  TO  E X C A V A T O R  R E C O V E R Y  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief:
 * After creating poses, call searchForScout() with first pose index. 
 * If done, update pose index, reset first. 
 * If searches exhausted || scout found ==> DONE
 * If scout_found ==> SUCCEEDED. 
 * */

void GoToExcavatorRecovery::entryPoint()
{
   ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: State Machine: Entrypoint of GoToExcavatorRecovery.");
   // define the offset distance for generating poses
   search_offset_ = 10.0;
   // set up the four recovery poses
   createPoses();
   // reset the pose that is going to be checked
   pose_index_ = 0;

   search_done_ = false;
   first_ = true;
   excavator_found_ = false;
   macro_state_done_ = false;
   macro_state_succeeded_ = false;
}

void GoToExcavatorRecovery::step() 
{
   // Send on search whenever previous search is done [AND] searches are not exhausted [AND] The goal hasn't been sent yet [AND] scout hasn't been found yet.
   searchForExcavator(pose_index_);
   // Check if searches are exhausted.
   searches_exhausted_ = (pose_index_ > 3);
   // Reset flags to enable searching on a new pose, gven conditions mentioned above
   if(search_done_ && !(searches_exhausted_) && !(excavator_found_)) 
   {
      first_ = true;
      ++pose_index_;
   }
   macro_state_done_ = (searches_exhausted_ || excavator_found_);
   macro_state_succeeded_ = excavator_found_;
}

// If the excavator finds the scout or all the recovery_poses_ are exhausted, this recovery state is done.
bool GoToExcavatorRecovery::isDone()
{
   // scout_found_ is updated in searchForScout()
   current_state_done_ = macro_state_done_;
   return current_state_done_;
}

bool GoToExcavatorRecovery::hasSucceeded()
{
   // succeeds if scout is found successfully 
   last_state_succeeded_ = macro_state_succeeded_;
   return last_state_succeeded_;
}

void GoToExcavatorRecovery::exitPoint()
{
   navigation_vision_client_->cancelGoal();
}

// Excavator searches for scout once it gets to one of those offset poses. 
void GoToExcavatorRecovery::searchForExcavator(int index) 
{
   if(first_)
   {
      ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: State Machine: Searching for Excavator at Pose # : " << index);
      navigation_vision_goal_.desired_object_label = COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS;
      navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_NAV_AND_NAV_VISION;
      target_loc_ = recovery_poses_[index];
      navigation_vision_goal_.goal_loc = target_loc_;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      first_ = false;
   }

   search_done_ = navigation_vision_client_->getState().isDone();
   if(search_done_)
      excavator_found_ = (navigation_vision_client_->getResult()->result == COMMON_RESULT::SUCCESS);
}

// When the excavator's Recovery method is triggered, it creates 4 offset poses from its current location and stores them in recovery_poses_
void GoToExcavatorRecovery::createPoses()
{
   ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: State Machine: Creating poses.");
   
   recovery_pose_ = hauler_pose_;
   recovery_pose_.pose.position.x += search_offset_;
   recovery_poses_[0] = recovery_pose_;

   recovery_pose_ = hauler_pose_;
   recovery_pose_.pose.position.x -= search_offset_;
   recovery_poses_[1] = recovery_pose_;

   recovery_pose_ = hauler_pose_;
   recovery_pose_.pose.position.y += search_offset_;
   recovery_poses_[2] = recovery_pose_;

   recovery_pose_ = hauler_pose_;
   recovery_pose_.pose.position.y -= search_offset_;
   recovery_poses_[3] = recovery_pose_;

   // ROS_INFO_STREAM("[STATE_MACHINES | excavator_state_machine.cpp | " << robot_name_ << " Poses created: [1] " << recovery_poses_[0] << 
   //                                                                                                    "\n[2] " << recovery_poses_[1] << 
   //                                                                                                    "\n[3] " << recovery_poses_[2] << 
   //                                                                                                    "\n[4] " << recovery_poses_[3] );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  G O  TO  L O O K O U T  L O C A T I O N  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void GoToLookoutLocation::entryPoint()
{
   ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: State Machine: Entrypoint of GoToLookoutLocation.");
   hardcoded_pose_ = (robot_name_ == COMMON_NAMES::HAULER_1_NAME) ? HAULER_1_LOOKOUT_LOC : HAULER_2_LOOKOUT_LOC;
   first_ = true;
   current_state_done_ = false;
   last_state_succeeded_ = false;
}
void GoToLookoutLocation::step() 
{
   if(first_)
   {
   navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
   navigation_action_goal_.pose = hardcoded_pose_;
   navigation_client_->sendGoal(navigation_action_goal_);
   ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  Going to Lookout Location : " << hardcoded_pose_);
   first_ = false;
   }

   current_state_done_ = navigation_client_->getState().isDone();
   if(current_state_done_ && !(first_))
      last_state_succeeded_ = (navigation_client_->getResult()->result == COMMON_RESULT::SUCCESS);
}

bool GoToLookoutLocation::isDone()
{
   return current_state_done_;
}

bool GoToLookoutLocation::hasSucceeded()
{
   return last_state_succeeded_;
}

void GoToLookoutLocation::exitPoint()
{
   navigation_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  B A L L E T  D A N C I N G  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HaulerBalletDancing::entryPoint() 
{
   first_ = true;
   current_state_done_ = false;
   last_state_succeeded_ = false;
}

void HaulerBalletDancing::step() 
{
   if(first_)
   {
      // Move forward 0.3 metres 
      navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;
      navigation_action_goal_.forward_velocity = -0.6;
      navigation_action_goal_.angular_velocity = 0;
      ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << " backing away from excavator");
      navigation_client_->sendGoal(navigation_action_goal_);
      ros::Duration(1).sleep();
      // brake wheels
      navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;
      navigation_action_goal_.forward_velocity = 0.0;   
      navigation_action_goal_.angular_velocity = 0;
      ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << " finished backing away from excavator");
      navigation_client_->sendGoal(navigation_action_goal_);
      ros::Duration(0.5).sleep();
      first_ = false;
   }
   current_state_done_ = navigation_client_->getState().isDone();
   if(current_state_done_ && !(first_))
      last_state_succeeded_ = (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool HaulerBalletDancing::isDone() {
   return current_state_done_;
} 

bool HaulerBalletDancing::hasSucceeded() {
   return last_state_succeeded_;
}

void HaulerBalletDancing::exitPoint()
{
    // clean up (cancel goals)
    navigation_client_->cancelGoal();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  I D L E  S T A T E  C L A S S ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// void IdleState::entryPoint() {
//   hauler_client_->cancelGoal();
//   navigation_vision_client_ ->cancelGoal();
//   navigation_client_->cancelGoal();
//   park_robot_client_->cancelGoal();
// }