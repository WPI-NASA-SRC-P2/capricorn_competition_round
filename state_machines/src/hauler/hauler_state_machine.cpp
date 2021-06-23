#include <ros/ros.h>
#include <state_machines/hauler_state_machine.h>
#include <state_machines/RobotStateMachineTaskAction.h>
#include <actionlib/server/simple_action_server.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// H A U L E R   B A S E   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

HaulerState::HaulerState(uint32_t un_id, uint32_t un_max_count) :
    State(un_id, ToString("mystate_", un_id)),
    m_unMaxCount(un_max_count)
{
  /** @todo: FIX SO THAT CAN USE ANY EXCAVATOR NAME, SO NO NAMESPACE ISSUES */
  robot_name_ = COMMON_NAMES::HAULER_1;
  navigation_vision_client_ = new NavigationVisionClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + robot_name_ + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
  navigation_client_ = new NavigationClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);
  hauler_client_ = new HaulerClient(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/" + COMMON_NAMES::HAULER_ACTIONLIB, true);
  park_robot_client_ = new ParkRobotClient(COMMON_NAMES::CAPRICORN_TOPIC +  robot_name_ + "/" + robot_name_ + COMMON_NAMES::PARK_HAULER_ACTIONLIB, true); 
  /** TODO:fix the common names PARK_HAULER, ALso 2 actions being published, 1 of them is running but not working.*/ 

  ROS_INFO("Waiting for the hauler action servers...");
  navigation_vision_client_->waitForServer();
  navigation_client_->waitForServer(); 
  hauler_client_->waitForServer(); 
  park_robot_client_->waitForServer(); 
  resetHaulerOdometryClient_.waitForExistence();
  ROS_INFO("All hauler action servers started!");

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
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Going to Processing Plant");
   
    target_loc_.pose.position.x = -6.0;
    target_loc_.pose.position.y = 7.0;
    target_loc_.pose.position.z = 1.6;
    target_loc_.header.frame_id = "map";
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
    }
    else
        ROS_INFO("going to processing plant");
}

void GoToProcPlant::exitPoint()
{
    // cleanup (cancel goals)
    ROS_INFO("Hauler arrived at processing plant, switching to HAULER_PARK_AT_HOPPER");
    navigation_vision_client_->cancelGoal();
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
    }
    else
        ROS_INFO("Parking at hopper");
}

void ParkAtHopper::exitPoint()
{
    // cleanup (cancel goal)
    park_robot_client_->cancelGoal();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Finished parking at hopper");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// R E S E T _ O D O M   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ResetOdom::entryPoint()
{
    // set entry variables
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Reseting odom with GT");
    first_ = true;
}

State& ResetOdom::transition()
{
    // transition to reset odometry state when complete
    if(first_)
        return *this;
    return getState(HAULER_DUMP_VOLATILE);
}

void ResetOdom::step()
{
    // reset odometry (at the hopper) using reset odom service
    if(first_)
    {
        reset_srv_.request.target_robot_name = robot_name_;
        reset_srv_.request.use_ground_truth = true;
        resetHaulerOdometryClient_.call(reset_srv_);
        first_ = false;
    }
}

void ResetOdom::exitPoint()
{
    // cleanup (cancel goal)
    ROS_WARN_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << "Hauler odometry has been reset");
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// U N D O C K  H O P P E R  S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void UndockHopper::entryPoint()
{
    // set entry variables
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Undocking from Hopper");
    // target_loc_.header.frame_id = robot_name_ + ROBOT_BASE;
    // target_loc_.pose.position.x = -5;
    // target_loc_.pose.orientation.w = 1;
    begin_ = ros::Time::now().toSec();
    current_ = ros::Time::now().toSec();
    // first_ = true;
}

State& UndockHopper::transition()
{
    // transition to 
    if(current_ <= (begin_ + 4.0))
        return *this;
    // return getState(HAULER_UNDOCK_EXCAVATOR);
    return getState(HAULER_GO_BACK_TO_EXCAVATOR);
}

void UndockHopper::step()
{
    // undock at the hopper using vnav
    // if(first_)
    // {      
            // first_ = false;
    // }
    // else
    navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;
    navigation_action_goal_.forward_velocity = -0.6;
    navigation_client_->sendGoal(navigation_action_goal_);
    navigation_client_->waitForResult();
    current_ = ros::Time::now().toSec();
    ROS_INFO("Undocking from hopper");
    
}

void UndockHopper::exitPoint()
{
    // cleanup (cancel goal)
    navigation_client_->cancelGoal();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Finished undocking from hopper");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// R E S E T _ O D O M _ A T _ H O P P E R   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ResetOdomMacro::entryPoint()
{
    // declare entrypoint variables
    ROS_INFO("RESET ODOM AT HOPPER MACROSTATE");
}

State& ResetOdomMacro::transition()
{
    // transition to first state of reset odom at hopper (HAULER_PARK_AT_HOPPER)
    return getState(HAULER_GO_TO_PROC_PLANT);
}

void ResetOdomMacro::step()
{
    // I don't think there is a step here actually
    ROS_INFO("MACROSTEPPING");
}


void ResetOdomMacro::exitPoint()
{
    ROS_INFO("MOVING TO MICROSTATES");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// G O _ T O _ E X C A V A T O R   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GoToExcavator::entryPoint()
{
    // declare entrypoint variables
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Going Back to Excavator (High Level Goal)");
    // pose of the excavator, supposed to be provided by scheduler
    target_loc_.pose.position.x = 10.0;
    target_loc_.pose.position.y = 25.0;
    target_loc_.pose.position.z = 1.6;
    target_loc_.header.frame_id = "map";
    
    first_ = true;
}

State& GoToExcavator::transition()
{
    // transition to parking at the excavator
    if(!(navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
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
    }
    else   
        ROS_INFO("Moving towards excavator");
}

void GoToExcavator::exitPoint()
{
    // clean up (cancel goals)
    navigation_vision_client_->cancelGoal();
    ROS_INFO("Reached excavator, preparing to park");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// P A R K _ A T _ E X C A V A T O R   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ParkAtExcavator::entryPoint()
{
    // set entry variables
    first_ = true;
    park_robot_goal_.hopper_or_excavator = OBJECT_DETECTION_EXCAVATOR_CLASS;
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Parking at Excavator");
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
        park_robot_client_->sendGoal(park_robot_goal_);
        park_robot_client_->waitForResult();
        first_ = false;
    }
    else
        ROS_INFO("Parking at excavator");
}

void ParkAtExcavator::exitPoint()
{
    // cleanup (cancel goal)
    park_robot_client_->cancelGoal();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Finished parking at excavator");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// U N D O C K  E X C A V A T O R  S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void UndockExcavator::entryPoint()
{
    // set entry variables
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Undocking from Excavator");
    // navigation_vision_goal_.desired_object_label = OBJECT_DETECTION_EXCAVATOR_CLASS;
    // navigation_vision_goal_.mode = COMMON_NAMES::NAV_VISION_TYPE::V_UNDOCK;
    begin_ = ros::Time::now().toSec();
    current_ = ros::Time::now().toSec();
    first_ = true;
}

State& UndockExcavator::transition()
{
    // transition to 
    // if(!(navigation_vision_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
    if(current_ <= (begin_ + 4.0))
        return *this;
        
    return getState(HAULER_GO_TO_PROC_PLANT);
}

void UndockExcavator::step()
{
    // undock at the excavator using vnav
    // if(first_)
    // {      
    //     first_ = false;
    //     navigation_vision_client_->sendGoal(navigation_vision_goal_);
    //     navigation_vision_client_->waitForResult();
    //     ROS_INFO("Undocking from excavator");
        
    // }
    // else
    //     ROS_INFO("Nothing can be done about this :(");
    navigation_action_goal_.drive_mode = NAV_TYPE::MANUAL;
    navigation_action_goal_.forward_velocity = -0.6;
    navigation_client_->sendGoal(navigation_action_goal_);
    navigation_client_->waitForResult();
    current_ = ros::Time::now().toSec();
    ROS_INFO("Undocking from excavator");
}

void UndockExcavator::exitPoint()
{
    // cleanup (cancel goal)
    navigation_client_->cancelGoal();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Finished undocking from excavator");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// D U M P   V O L A T I L E ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DumpVolatile::entryPoint()
{
    // set entry variables
    first_ = true;
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Dumping Volatile");
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
        hauler_client_->waitForResult();
        first_ = false;
    }
    else
        ROS_INFO("Dumping volatile");
}

void DumpVolatile::exitPoint()
{
    // cleanup (cancel goal)
    park_robot_client_->cancelGoal();
    ROS_INFO_STREAM("[STATE_MACHINES | hauler_state_machine.cpp | " << robot_name_ << "]: " << robot_name_ << " State Machine: Finished dumping volatile");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// F O L L O W _ E X C A V A T O R   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void FollowExcavator::entryPoint()
// {
//     // declare entrypoint variables
// }

// State& FollowExcavator::transition()
// {
//     // transition to next state
// }

// void FollowExcavator::step()
// {
//     // do the thing
// }

// void FollowExcavator::exitPoint()
// {
//     // clean up 
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// F O L L O W _ E X C A V A T O R   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void FollowExcavator::entryPoint()
// {
//     // declare entrypoint variables
// }

// State& FollowExcavator::transition()
// {
//     // transition to next state
// }

// void FollowExcavator::step()
// {
//     // do the thing
// }

// void FollowExcavator::exitPoint()
// {
//     // clean up 
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// M A I N ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


// int main(int argc, char** argv)
// {
//    ros::init(argc, argv, "hauler_state_machine");
//    ros::NodeHandle nh;

//    try {
//       HaulerScheduler cSchd(700);
      
//       cSchd.addState(new GoToProcPlant());
//       cSchd.addState(new ParkAtHopper());
//       cSchd.addState(new ResetOdom());
//       cSchd.addState(new UndockHopper());
//       cSchd.addState(new ResetOdomMacro());
//       cSchd.addState(new GoToExcavator());
//       cSchd.addState(new ParkAtExcavator());
//       cSchd.addState(new UndockExcavator());
//       cSchd.addState(new DumpVolatile());
//       cSchd.setInitialState(HAULER_PARK_AT_HOPPER);
//       cSchd.exec();
//       return 0;
//    }
//    catch(StateMachineException& ex) {
//       std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
//    }
// }