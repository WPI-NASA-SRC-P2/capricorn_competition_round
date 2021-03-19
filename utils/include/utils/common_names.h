namespace COMMON_NAMES
{

  /****** ROBOTS ******/
  const std::string SCOUT_1 = "small_scout_1";
  const std::string SCOUT_2 = "small_scout_2";
  const std::string SCOUT_3 = "small_scout_3";

  const std::string EXCAVATOR_1 = "small_excavator_1";
  const std::string EXCAVATOR_2 = "small_excavator_2";
  const std::string EXCAVATOR_3 = "small_excavator_3";

  const std::string HAULER_1 = "small_hauler_1";
  const std::string HAULER_2 = "small_hauler_2";
  const std::string HAULER_3 = "small_hauler_3";

  /****** ROBOT FRAMES ******/
  const std::string MAP = "map";  
  const std::string ODOM = "odom";  
  const std::string ROBOT_BASE = "base_footprint";  

  /****** WHEELS ******/
  const std::string FRONT_LEFT_WHEEL = "/front_left_wheel";
  const std::string FRONT_RIGHT_WHEEL = "/front_right_wheel";
  const std::string BACK_RIGHT_WHEEL = "/back_right_wheel";
  const std::string BACK_LEFT_WHEEL = "/back_left_wheel";

  /****** VELOCITY ******/
  const std::string VELOCITY_TOPIC = "/drive/command/velocity";
  const std::string STEERING_TOPIC = "/steer/command/position";

  /****** ACTIONLIBS ******/
  const std::string NAVIGATION_ACTIONLIB = "navigation";

  /****** GAZEBO ******/
  const std::string HEIGHTMAP = "heightmap";
  const std::string MODEL_STATE_QUERY = "/gazebo/get_model_state";

  /****** RTABMAP ******/
  const std::string RESET_POSE_CLIENT = "/camera/reset_odom_to_pose";

  /****** ROS NODE NAMES ******/
  const std::string INITALIZE_ODOM_NODE_NAME = "_odom_initialize";
  const std::string CHEAT_ODOM_PUB_NODE_NAME = "_cheat_odom_publisher";
  const std::string ODOM_ERROR_NODE_NAME = "_odom_errorr";
  const std::string NOISY_IMAGE_NODE_NAME = "_noisy_image_eliminate";
  const std::string HORIZON_TRACKING_NODE_NAME = "_horzion_tracking";

  /****** TOPIC NAMES ******/
  const std::string CAPRICORN_TOPIC = "/capricorn/";
  const std::string POSE_ERROR_TOPIC = "/pose_error";
  const std::string CHEAT_ODOM_TOPIC = "/cheat_odom";
  const std::string RIGHT_IMAGE_RAW_TOPIC = "/camera/right/image_raw";
  const std::string LEFT_IMAGE_RAW_TOPIC = "/camera/left/image_raw";
  const std::string RIGHT_CAMERAINFO_TOPIC = "/camera/right/camera_info";
  const std::string LEFT_CAMERAINFO_TOPIC = "/camera/left/camera_info";
  const std::string SET_SENSOR_PITCH_TOPIC = "/sensor/pitch/command/position";

  /****** HAULER NAMES ******/
  const std::string SET_BIN_POSITION = "/bin/command/position";
  
} // namespace CAPRICORN_COMMON_NAMES
