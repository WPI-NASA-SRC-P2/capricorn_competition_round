#include <team_level/robot_status.h>

RobotStatus::RobotStatus(ros::NodeHandle nh)
{
   robot_state_subscriber = nh.subscribe(ROBOTS_CURRENT_STATE_TOPIC, 1000, &RobotStatus::robotStateCB, this);
}

void RobotStatus::robotStateCB(state_machines::robot_state_status msg)
{
   std::string robot_name = msg.robot_name;
   if(ROBOT_NAME_TO_ENUM_MAP.find(robot_name) != ROBOT_NAME_TO_ENUM_MAP.end()) {
      ROBOTS_ENUM robot_enum = ROBOT_NAME_TO_ENUM_MAP[robot_name];
      robot_isDone_hasFailed_map[robot_enum] = std::pair<bool,bool>(msg.current_state_done, msg.has_failed_last_state);
   }
   else {
      ROS_ERROR_STREAM(ROBOTS_CURRENT_STATE_TOPIC<<" published with an irregular robot name. Provided name: "<<msg.robot_name);
   }
}

bool RobotStatus::hasFailed(ROBOTS_ENUM robot){
   if(robot_isDone_hasFailed_map.find(robot) != robot_isDone_hasFailed_map.end()) {
      return robot_isDone_hasFailed_map[robot].second;
   }
   else {
      ROS_WARN_STREAM("Haven't received any message for the robot enum"<< robot <<" yet");
      return false;
   }
}

bool RobotStatus::isDone(ROBOTS_ENUM robot){
   if(robot_isDone_hasFailed_map.find(robot) != robot_isDone_hasFailed_map.end()) {
      return robot_isDone_hasFailed_map[robot].first;
   }
   else {
      ROS_WARN_STREAM("Haven't received any message for the robot enum"<< robot <<" yet");
      return false;
   }
}

// int main(int argc, char** argv)
// {
//    ros::init(argc, argv, "scout_state_machine");
//    ros::NodeHandle nh;

//    RobotStatus robot_status(nh);
//    while(ros::ok())
//    {
//       ROS_INFO_STREAM("Robot state done:"<<robot_status.isDone(SCOUT_1)<<"\tLast state failed:"<<robot_status.hasFailed(SCOUT_1));
//       ros::Duration(1).sleep();
//       ros::spinOnce();
//    }
// }
