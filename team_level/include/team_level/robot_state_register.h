#pragma once
#include <utils/common_names.h>
#include <map>
#include "ros/ros.h"
#include <state_machines/robot_state_status.h>
#include <state_machines/robot_desired_state.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"

using namespace COMMON_NAMES;

#define DEBUG false

class RobotStateRegister
{
   public:
      RobotStateRegister(ros::NodeHandle nh);
      bool hasSucceeded(ROBOTS_ENUM robot);
      bool isDone(ROBOTS_ENUM robot);
      STATE_MACHINE_TASK currentState(ROBOTS_ENUM robot);

      void setRobotState(ROBOTS_ENUM robot, STATE_MACHINE_TASK desired_task);
      void setRobotState(ROBOTS_ENUM robot, ROBOTS_ENUM target_robot, STATE_MACHINE_TASK desired_task);
      void setRobotState(ROBOTS_ENUM robot, STATE_MACHINE_TASK desired_task, geometry_msgs::PoseStamped target_pose);

      bool isRobotOutOfCommission(ROBOTS_ENUM robot);
      
   private:
      ros::Subscriber robot_state_subscriber, robot_out_of_commission_subscriber;
      ros::Publisher robot_state_publisher;
      void robotStateCB(state_machines::robot_state_status msg);
      void robotOutOfCommissionCB(std_msgs::String msg);
      
      std::map<ROBOTS_ENUM, std::pair<bool, bool>> robot_isDone_hasSucceeded_map;
      std::map<ROBOTS_ENUM, STATE_MACHINE_TASK> robot_state_map;
      
      std::map<ROBOTS_ENUM, bool> robot_out_of_commission_map = {
                                                      {SCOUT_1, false},
                                                      {SCOUT_2, false},
                                                      {SCOUT_3, false},
                                                      {EXCAVATOR_1, false},
                                                      {EXCAVATOR_2, false},
                                                      {EXCAVATOR_3, false},
                                                      {HAULER_1, false},
                                                      {HAULER_2, false},
                                                      {HAULER_3, false}};

      std::map<std::string, ROBOTS_ENUM> ROBOT_NAME_TO_ENUM_MAP = {
                                                      {SCOUT_1_NAME, SCOUT_1},
                                                      {SCOUT_2_NAME, SCOUT_2},
                                                      {SCOUT_3_NAME, SCOUT_3},
                                                      {EXCAVATOR_1_NAME, EXCAVATOR_1},
                                                      {EXCAVATOR_2_NAME, EXCAVATOR_2},
                                                      {EXCAVATOR_3_NAME, EXCAVATOR_3},
                                                      {HAULER_1_NAME, HAULER_1},
                                                      {HAULER_2_NAME, HAULER_2},
                                                      {HAULER_3_NAME, HAULER_3}};


      std::map<ROBOTS_ENUM, std::string> ROBOT_ENUM_NAME_MAP{
                                    {SCOUT_1, SCOUT_1_NAME},
                                    {SCOUT_2, SCOUT_2_NAME},
                                    {SCOUT_3, SCOUT_3_NAME},
                                    {EXCAVATOR_1, EXCAVATOR_1_NAME},
                                    {EXCAVATOR_2, EXCAVATOR_2_NAME},
                                    {EXCAVATOR_3, EXCAVATOR_3_NAME},
                                    {HAULER_1, HAULER_1_NAME},
                                    {HAULER_2, HAULER_2_NAME},
                                    {HAULER_3, HAULER_3_NAME},
                                    {NONE, "NONE"}};
};

