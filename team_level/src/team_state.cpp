#include <team_level/team_state.h>


TeamState::TeamState(uint32_t un_id, const std::string& str_name, ros::NodeHandle nh) :
    //   m_pcRobotScheduler(nullptr), 
      m_unId(un_id), m_strName(str_name) 
{

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// U N D O C K   S T A T E   C L A S S ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void STANDBY::entryPoint()
// {
//    //Set to true to avoid repeatedly giving the goal.
//    first_ = true;
//    ROS_INFO("entrypoint of undock");
// }

// State& Undock::transition()
// {
//    if(!(navigation_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)) {
//       ROS_INFO("remaining in undock");
//       return *this;
//    }
//    ROS_INFO("transitioning out of undock");
//    return getState(SCOUT_SEARCH_VOLATILE); // should be SCOUT_SEARCH_VOLATILE
// }

// void Undock::step()
// {
//    if(first_)
//    {
//       ROS_INFO_STREAM(robot_name_ << " State Machine: Undocking from volatile");
//       geometry_msgs::PoseStamped pt;
//       pt.header.frame_id = robot_name_ + ROBOT_BASE;
//       pt.pose.position.x = -6;
//       pt.pose.orientation.w = 1;
      
//       navigation_action_goal_.drive_mode = NAV_TYPE::GOAL;
//       navigation_action_goal_.pose = pt;
//       navigation_client_->sendGoal(navigation_action_goal_);
//       ROS_INFO_STREAM("Goal sent : " << navigation_action_goal_);
//       // navigation_client_->waitForResult();
//       first_ = false;
//    }   
//    else
//       ROS_INFO_STREAM("Undock stepping, first_ = false now");
// }

// void Undock::exitPoint() 
// {
//    ROS_INFO("exitpoint of undock, cancelling undock goal");
//    navigation_client_->cancelGoal();
// }

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

// State& Search::transition()
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

// State& Locate::transition()
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
//       cSchd.addState(new Undock());
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
