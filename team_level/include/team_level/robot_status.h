#include <utils/common_names.h>
#include <map>
#include "ros/ros.h"
#include <state_machines/robot_state_status.h>

using namespace COMMON_NAMES;

class RobotStatus
{
   public:
      RobotStatus(ros::NodeHandle nh);
      bool hasFailed(ROBOTS_ENUM robot);
      bool isDone(ROBOTS_ENUM robot);
   private:
      ros::Subscriber robot_state_subscriber;
      void robotStateCB(state_machines::robot_state_status msg);
      std::map<ROBOTS_ENUM, std::pair<bool, bool>> robot_isDone_hasFailed_map;
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
};

