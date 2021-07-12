#include <team_level/robot_state_register.h>

RobotStateRegister::RobotStateRegister(ros::NodeHandle nh)
{
   robot_state_subscriber = nh.subscribe(CAPRICORN_TOPIC + ROBOTS_CURRENT_STATE_TOPIC, 1000, &RobotStateRegister::robotStateCB, this);

   robot_state_publisher = nh.advertise<state_machines::robot_desired_state>(COMMON_NAMES::CAPRICORN_TOPIC + ROBOTS_DESIRED_STATE_TOPIC, 1, true);
}

void RobotStateRegister::robotStateCB(state_machines::robot_state_status msg)
{
   std::string robot_name = msg.robot_name;
   if(ROBOT_NAME_TO_ENUM_MAP.find(robot_name) != ROBOT_NAME_TO_ENUM_MAP.end()) 
   {
      ROBOTS_ENUM robot_enum = ROBOT_NAME_TO_ENUM_MAP[robot_name];
      robot_isDone_hasSucceeded_map[robot_enum] = std::pair<bool,bool>(msg.current_state_done, msg.last_state_succeeded);
      robot_state_map[robot_enum] = (STATE_MACHINE_TASK)msg.robot_current_state;
   }
   else 
   {
      ROS_ERROR_STREAM("[TEAM_LEVEL | robot_state_register.cpp ]: " << ROBOTS_CURRENT_STATE_TOPIC << " published with an irregular robot name. Provided name: " << msg.robot_name);
   }
}

bool RobotStateRegister::hasSucceeded(ROBOTS_ENUM robot)
{
   if(robot_isDone_hasSucceeded_map.find(robot) != robot_isDone_hasSucceeded_map.end()) 
   {
      return robot_isDone_hasSucceeded_map[robot].second;
   }
   else 
   {
      // ROS_INFO_STREAM("Haven't received any message for the robot enum"<< robot <<" yet");
      return false;
   }
}

bool RobotStateRegister::isDone(ROBOTS_ENUM robot)
{
   if(robot_isDone_hasSucceeded_map.find(robot) != robot_isDone_hasSucceeded_map.end()) 
   {
      return robot_isDone_hasSucceeded_map[robot].first;
   }
   else 
   {
      // ROS_INFO_STREAM("Haven't received any message for the robot enum"<< robot <<" yet");
      return false;
   }
}

STATE_MACHINE_TASK RobotStateRegister::currentState(ROBOTS_ENUM robot)
{
   if(robot_state_map.find(robot) != robot_state_map.end()) 
   {
      return (STATE_MACHINE_TASK)robot_state_map[robot];
   }
   else 
   {
      // ROS_INFO_STREAM("Haven't received any message for the robot enum"<< robot <<" yet");
      return ROBOT_IDLE_STATE;
   }
}

void RobotStateRegister::setRobotState(ROBOTS_ENUM robot, STATE_MACHINE_TASK desired_task)
{
   if(ROBOT_ENUM_NAME_MAP.find(robot) != ROBOT_ENUM_NAME_MAP.end()) 
   {
    state_machines::robot_desired_state desired_state_msg;
    desired_state_msg.robot_name = ROBOT_ENUM_NAME_MAP[robot];
    desired_state_msg.robot_desired_state = desired_task;
    robot_state_publisher.publish(desired_state_msg);
   //  ROS_ERROR_STREAM("[ TEAM_LEVEL | robot_state_register ]: "<< desired_state_msg.robot_name <<"  "<<desired_state_msg.robot_desired_state);
    ros::Duration(0.01).sleep();
   }
   else
   {
      ROS_ERROR_STREAM("[TEAM_LEVEL | robot_state_register.cpp ]: Robot enum " << robot << " not found in map ROBOT_ENUM_NAME_MAP");
   }
}

void RobotStateRegister::setRobotState(ROBOTS_ENUM robot, ROBOTS_ENUM target_robot, STATE_MACHINE_TASK desired_task)
{
   if((ROBOT_ENUM_NAME_MAP.find(robot) != ROBOT_ENUM_NAME_MAP.end()) && 
      (ROBOT_ENUM_NAME_MAP.find(target_robot) != ROBOT_ENUM_NAME_MAP.end())) {
    state_machines::robot_desired_state desired_state_msg;
    desired_state_msg.robot_name = ROBOT_ENUM_NAME_MAP[robot];
    desired_state_msg.target_robot_name = ROBOT_ENUM_NAME_MAP[target_robot];
    desired_state_msg.robot_desired_state = desired_task;
    robot_state_publisher.publish(desired_state_msg);
   //  ROS_ERROR_STREAM("[ TEAM_LEVEL | robot_state_register ]: "<< desired_state_msg.robot_name <<"  "<<desired_state_msg.robot_desired_state);
    ros::Duration(0.01).sleep();
   }
   else{
      ROS_ERROR_STREAM("Robot enum "<<robot<<" or enum "<<target_robot<<" not found in map ROBOT_ENUM_NAME_MAP");
   }
}

void RobotStateRegister::setRobotState(ROBOTS_ENUM robot, STATE_MACHINE_TASK desired_task, geometry_msgs::PoseStamped target_pose)
{
   if(ROBOT_ENUM_NAME_MAP.find(robot) != ROBOT_ENUM_NAME_MAP.end()) 
   {
    state_machines::robot_desired_state desired_state_msg;
    desired_state_msg.robot_name = ROBOT_ENUM_NAME_MAP[robot];
    desired_state_msg.robot_desired_state = desired_task;
    desired_state_msg.goal_pose = target_pose;
    robot_state_publisher.publish(desired_state_msg);
   //  ROS_ERROR_STREAM("[ TEAM_LEVEL | robot_state_register ]: "<< desired_state_msg.robot_name <<"  "<<desired_state_msg.robot_desired_state);
    ros::Duration(0.01).sleep();
   }
   else
   {
      ROS_ERROR_STREAM("[TEAM_LEVEL | robot_state_register.cpp ]: Robot enum " << robot << " not found in map ROBOT_ENUM_NAME_MAP");
   }
}