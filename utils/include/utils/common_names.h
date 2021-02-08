#include <string.h>

enum RobotWheel
{
  FRONT_LEFT = 0,
  FRONT_RIGHT,
  BACK_LEFT,
  BACK_RIGHT
};

enum RobotDriveMode
{
  DRIVE = 0,
  ROTATE,
  CIRCULAR_TURN,
  GOAL_POINT,
  GOAL_POSE  
};

namespace COMMON_NAMES
{
  /****** ROBOTS ******/
  const std::string SCOUT_1 = "small_scout_1/";
  const std::string SCOUT_2 = "small_scout_2/";
  const std::string SCOUT_3 = "small_scout_3/";

  const std::string EXCAVATOR_1 = "small_excavator_1/";
  const std::string EXCAVATOR_2 = "small_excavator_2/";
  const std::string EXCAVATOR_3 = "small_excavator_3/";

  const std::string HAULER_1 = "small_hauler_1/";
  const std::string HAULER_2 = "small_hauler_2/";
  const std::string HAULER_3 = "small_hauler_3/";
  

  /****** WHEELS ******/
  const std::string FRONT_LEFT_WHEEL = "front_left_wheel/";
  const std::string FRONT_RIGHT_WHEEL = "front_right_wheel/";
  const std::string BACK_RIGHT_WHEEL = "back_right_wheel/";
  const std::string BACK_LEFT_WHEEL = "back_left_wheel/";

  /****** VELOCITY ******/
  const std::string VELOCITY_TOPIC = "drive/command/velocity";
  const std::string STEERING_TOPIC = "steer/command/position";

  /****** VELOCITY ******/
  const std::string NAVIGATION_ACTIONLIB = "navigation";

} // namespace CAPRICORN_COMMON_NAMES
