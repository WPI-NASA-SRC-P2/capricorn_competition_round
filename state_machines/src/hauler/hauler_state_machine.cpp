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

/** TODO: Check if the object detection callback is needed*/

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
   //  else
}

bool GoToProcPlant::isDone() 
{
   current_state_done_ = navigation_vision_client_->getState().isDone();
   return current_state_done_;
} 

bool GoToProcPlant::hasSucceeded() 
{
   last_state_succeeded_ = (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
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
    // else   
}

bool HaulerGoToScout::isDone() 
{
   current_state_done_ = navigation_vision_client_->getState().isDone();
   return current_state_done_;
} 

bool HaulerGoToScout::hasSucceeded() 
{
   last_state_succeeded_ = (navigation_vision_result_.result == COMMON_RESULT::SUCCESS);
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
   //  else
}

bool ParkAtHopper::isDone() 
{
   current_state_done_ = park_robot_client_->getState().isDone();
   return current_state_done_;
} 

bool ParkAtHopper::hasSucceeded() 
{
   last_state_succeeded_ = (park_robot_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
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
   // update the current status of the robot and publish it
   
}

bool UndockHopper::isDone()
{
   // update the status of current state
   current_state_done_ = reset_succeeded_;

   return current_state_done_;
}

bool UndockHopper::hasSucceeded()
{
   // update the status of current state
   // last_state_succeeded_ = (navigation_client_->getState() == actionlib::status::SUCCESS);
//    last_state_succeeded_ = (navigation_vision_result_.result == COMMON_RESULT::SUCCESS);
   last_state_succeeded_ = reset_succeeded_;
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
   undock_done_ = (navigation_vision_result_.result == COMMON_RESULT::SUCCESS);
   
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
   //  else   
}

bool GoToExcavator::isDone() 
{
   current_state_done_ = navigation_vision_client_->getState().isDone();
   return current_state_done_;
} 

bool GoToExcavator::hasSucceeded() 
{
   last_state_succeeded_ = (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
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
    // else
}

bool ParkAtExcavator::isDone() 
{
   current_state_done_ = park_robot_client_->getState().isDone();
   return current_state_done_;
} 

bool ParkAtExcavator::hasSucceeded() 
{
   last_state_succeeded_ = (park_robot_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
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
   
   // update the current status of the robot and publish it
   
}

bool UndockExcavator::isDone()
{
   // update the status of current state
   current_state_done_ = navigation_vision_client_->getState().isDone();

   return current_state_done_;
}

bool UndockExcavator::hasSucceeded()
{
   // update the status of current state
   // last_state_succeeded_ = (navigation_client_->getState() == actionlib::status::SUCCESS);
   last_state_succeeded_ = (navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
   
   return last_state_succeeded_;
}

void UndockExcavator::step()
{
   if(first_)
   {
      ROS_INFO_STREAM(robot_name_ << " State Machine: Undocking from volatile");
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
      navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_UNDOCK;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      first_ = false; 
      ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  Undock stepping, first_ = false now");
   }
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
   //  else
}

bool DumpVolatile::isDone() 
{
   current_state_done_ = hauler_client_->getState().isDone();
   return current_state_done_;
} 

bool DumpVolatile::hasSucceeded() 
{
   last_state_succeeded_ = (hauler_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
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
}

bool HaulerGoToLoc::isDone() 
{
   current_state_done_ = navigation_client_->getState().isDone();
   return current_state_done_;
} 

bool HaulerGoToLoc::hasSucceeded() 
{
   last_state_succeeded_ = (navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
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
/** TODO: should be reseting at processing plant instead */
void DumpVolatileAtHopper::entryPoint()
{
   // we assume we are near the volatile
   first_GTPP = true;
   first_PAH = true;
   first_UFH = true;
   first_DV = true;
   first_ROH = true;
   first_GTRS = true;
   micro_state = GO_TO_PROC_PLANT;
   macro_state_succeeded = false;
   macro_state_done = false;
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
   case GO_TO_REPAIR_STATION:
      goToRepairStation();
      break;
   case HAULER_IDLE:
      idleScout();
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
      if (park_robot_client_->getResult()->result == COMMON_RESULT::SUCCESS)
         micro_state = DUMP_VOLATILE;
      else
      {
         first_PAH = true;
         micro_state = PARK_AT_HOPPER;
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
            first_DV = true;
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
   bool is_done;
   if(first_ROH) 
   {
   ros::ServiceClient resetOdometryClient = nh_.serviceClient<maploc::ResetOdom>(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY);
   maploc::ResetOdom srv;
   srv.request.target_robot_name = robot_name_;
   srv.request.at_hopper = true;
   is_done = resetOdometryClient.call(srv);
   if(!is_done)
      ROS_WARN_STREAM("[STATE_MACHINES | hauler_state_machine ]: Failed to reset odom for " << robot_name_);
   micro_state = GO_TO_REPAIR_STATION;
   first_ROH = false;
   return;
   }

   
}

void DumpVolatileAtHopper::goToRepairStation() 
{
   if(first_GTRS)
   {
   navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_REPAIR_STATION_CLASS;
   navigation_vision_goal_.mode = V_REACH;
   navigation_vision_client_->sendGoal(navigation_vision_goal_);
   first_GTRS = false;
   }
   
   macro_state_succeeded = (navigation_vision_client_->getResult()->result == COMMON_RESULT::SUCCESS);
   macro_state_done = navigation_vision_client_->getState().isDone();
   micro_state = HAULER_IDLE;  
}

void DumpVolatileAtHopper::exitPoint()
{
   // none at the moment
   navigation_vision_client_->cancelGoal();
   park_robot_client_->cancelGoal();
   hauler_client_->cancelGoal();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  G O  TO  R E P A I R  S T A T I O N  S T A T E  C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HaulerGoToRepairStation::entryPoint()
{
   first_ = true;
}

bool HaulerGoToRepairStation::isDone()
{
   current_state_done_ = navigation_vision_client_->getState().isDone();
   return current_state_done_;
}

bool HaulerGoToRepairStation::hasSucceeded()
{
   last_state_succeeded_ = (navigation_vision_result_.result == COMMON_RESULT::SUCCESS);
   if(last_state_succeeded_)
      ROS_WARN_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]:  Scout Go to Repair Station Completed Successfully");
   return last_state_succeeded_;
}

void HaulerGoToRepairStation::step()
{

   if (first_)
   {
      navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_REPAIR_STATION_CLASS;
      navigation_vision_goal_.mode = V_REACH;
      // navigation_vision_goal_.target_loc = target_loc_;
      navigation_vision_client_->sendGoal(navigation_vision_goal_);
      first_ = false;
      ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: Going to repair station Step Function!");
   }
}

void HaulerGoToRepairStation::exitPoint()
{
   // none at the moment
   navigation_vision_client_->cancelGoal();
}

