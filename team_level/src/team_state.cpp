#include <team_level/team_state.h>


TeamState::TeamState(uint32_t un_id, const std::string& str_name, ros::NodeHandle nh) :
    //   m_pcRobotScheduler(nullptr), 
      m_unId(un_id), m_strName(str_name) 
{
    robot_status = new RobotStatus(nh);

    scout_1_service_client = nh.serviceClient<state_machines::set_robot_state>(CAPRICORN_TOPIC + SCOUT_1_NAME + SET_ROBOT_STATE_SRV);
    scout_2_service_client = nh.serviceClient<state_machines::set_robot_state>(CAPRICORN_TOPIC + SCOUT_2_NAME + SET_ROBOT_STATE_SRV);
    scout_3_service_client = nh.serviceClient<state_machines::set_robot_state>(CAPRICORN_TOPIC + SCOUT_3_NAME + SET_ROBOT_STATE_SRV);
    excavator_1_service_client = nh.serviceClient<state_machines::set_robot_state>(CAPRICORN_TOPIC + EXCAVATOR_1_NAME + SET_ROBOT_STATE_SRV);
    excavator_2_service_client = nh.serviceClient<state_machines::set_robot_state>(CAPRICORN_TOPIC + EXCAVATOR_2_NAME + SET_ROBOT_STATE_SRV);
    excavator_3_service_client = nh.serviceClient<state_machines::set_robot_state>(CAPRICORN_TOPIC + EXCAVATOR_3_NAME + SET_ROBOT_STATE_SRV);
    hauler_1_service_client = nh.serviceClient<state_machines::set_robot_state>(CAPRICORN_TOPIC + HAULER_1_NAME + SET_ROBOT_STATE_SRV);
    hauler_2_service_client = nh.serviceClient<state_machines::set_robot_state>(CAPRICORN_TOPIC + HAULER_2_NAME + SET_ROBOT_STATE_SRV);
    hauler_3_service_client = nh.serviceClient<state_machines::set_robot_state>(CAPRICORN_TOPIC + HAULER_3_NAME + SET_ROBOT_STATE_SRV);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// S E A R C H   S T A T E   C L A S S ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void STANDBY::entryPoint()
{
   //Set to true to avoid repeatedly giving the goal.
   ROS_INFO("entrypoint of undock");
   if(scout == NONE)
      ROS_ERROR_STREAM("SCOUT IS UNSET, BUT STILL ENTRY POINT HAS BEEN CALLED!");

   ROS_INFO("Making sure service client is connected");

   if(ROBOT_ENUM_SERVICE_MAP.find(scout) != ROBOT_ENUM_SERVICE_MAP.end()) {
      ROBOT_ENUM_SERVICE_MAP[scout].waitForExistence();
      ROS_INFO("Scout server is connected");
   }
   else {
      ROS_ERROR_STREAM("Set Scout "<<scout<<" doesn't exist on the ROBOT_ENUM_SERVICE_MAP");
   }
}

bool STANDBY::isDone()
{
   return robot_status->isDone(scout);
}

void STANDBY::step()
{
   state_machines::set_robot_state robot_state_srv;
   robot_state_srv.request.robot_state = SCOUT_SEARCH_VOLATILE;
   ROBOT_ENUM_SERVICE_MAP[scout].call(robot_state_srv);
}

void STANDBY::exitPoint() 
{
   ROS_INFO("exitpoint of undock, cancelling undock goal");

   state_machines::set_robot_state robot_state_srv;
   robot_state_srv.request.robot_state = SCOUT_STOP_SEARCH;
   ROBOT_ENUM_SERVICE_MAP[scout].call(robot_state_srv);
}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////// S E A R C H   S T A T E   C L A S S ////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void Search::entryPoint()
// {
//    /** @TODO: Add no objects in vision functionality */
//    // start off with spiraling
//    srv.request.resume_spiral_motion = true;
//    near_volatile_ = false;
//    ROS_INFO("entering scout_search state");
// }

// bool Search::isDone()
// {
//    // if no transition, stay in current state
//    if(!near_volatile_) {
//       // stay in SCOUT_SEARCH_VOLATILE
//       return *this;
//    }
//    // if near the volatile, switch to scout_locate_volatile state
//    ROS_INFO("volatile detected, transitioning to scout_undock state");
//    return getState(SCOUT_LOCATE_VOLATILE);
// }

// void Search::step()
// {
//    // execute spiral motion
//    ROS_INFO("Executing spiral motion");
//    spiralClient_.call(srv);
// }

// void Search::exitPoint()
// {
//    // cancel spiral motion 
//    srv.request.resume_spiral_motion = false;
//    spiralClient_.call(srv);
//    ROS_INFO("Exited spiral search.");
// }

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////  L O C A T E  S T A T E  C L A S S ////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void Locate::entryPoint()
// {
//    // we assume we are near the volatile
//    near_volatile_ = true;
//    first_ = true;
// }

// bool Locate::isDone()
// {
//    // switch states once completed with locating the volatile
//    if(!(resource_localiser_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && near_volatile_) {
//       return *this;
//    }
//    ROS_WARN("Volatile found, moving to undock state");
//    return getState(SCOUT_UNDOCK);
// }

// void Locate::step()
// {
//    if(first_)
//    {
//       resource_localiser_client_->sendGoal(goal);
//       ROS_INFO_STREAM("Sending resource localization goal");
//       first_ = false;
//    }
//    ROS_INFO_STREAM("Locating Step Function!");
// }

// void Locate::exitPoint()
// {
//    // none at the moment
//    resource_localiser_client_->cancelGoal();
//    near_volatile_ = false;
// }

// //////////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////// M A I N ////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////


// int main(int argc, char** argv)
// {
//    ros::init(argc, argv, "scout_state_machine");
//    ros::NodeHandle nh;

//    try {
//       ScoutScheduler cSchd(700);
//       cSchd.addState(new Search());
//       cSchd.addState(new STANDBY());
//       cSchd.addState(new Locate());
//       cSchd.setInitialState(SCOUT_SEARCH_VOLATILE);
//       // cSchd.setInitialState(SCOUT_UNDOCK);
//       cSchd.exec();
//       return 0;
//    }
//    catch(StateMachineException& ex) {
//       std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
//    }
// }
