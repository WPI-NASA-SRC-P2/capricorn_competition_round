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
  const std::string DESIRED_VELOCITY = "/desired_velocity";
  const std::string CURRENT_SPEED = "/current_speed";
  const std::string BRAKE_ROVER = "/brake_rover";

  /****** ACTIONLIBS ******/
  const std::string NAVIGATION_ACTIONLIB = "navigation";
  const std::string RESOURCE_LOCALISER_ACTIONLIB = "resource_localiser_actionlib";

  /****** GAZEBO ******/
  const std::string HEIGHTMAP = "heightmap";
  const std::string PROCESSING_PLANT_GAZEBO = "processing_plant";
  const std::string REPAIR_STATION_GAZEBO = "repair_station";
  const std::string PROCESSING_PLANT_LINK_GAZEBO = "processing_plant_link";
  const std::string REPAIR_STATION_LINK_GAZEBO = "repair_station_link";
  const std::string MODEL_STATE_QUERY = "/gazebo/get_model_state";
  const std::string LINK_STATE_QUERY = "/gazebo/get_link_state";
  const std::string SENSOR_BAR_GAZEBO = "_sensor_bar";

  /****** RTABMAP ******/
  const std::string RESET_POSE_CLIENT = "/camera/reset_odom_to_pose";

  /****** ACTIONLIB NAMES ******/
  const std::string FIND_PP_RS_ACTIONLIB_NAME = "_find_pp_rs";

  /****** ROS NODE NAMES ******/
  const std::string PR_DATASET_NODE_NAME = "_pr_dataset";
  const std::string GROUND_TRUTH_PR_NODE_NAME = "_ground_truth_pr";
  const std::string PR_LOCALIZATION_NODE_NAME = "_pr_localization";
  const std::string INITALIZE_ODOM_NODE_NAME = "_odom_initialize";
  const std::string CHEAT_ODOM_PUB_NODE_NAME = "_cheat_odom_publisher";
  const std::string ODOM_ERROR_NODE_NAME = "_odom_errorr";
  const std::string NOISY_IMAGE_NODE_NAME = "_noisy_image_eliminate";
  const std::string HORIZON_TRACKING_NODE_NAME = "_horzion_tracking";
  const std::string FIND_PP_RS_SERVER_NODE_NAME = "_find_pp_rs_server";

  /****** TOPIC NAMES ******/
  const std::string CAPRICORN_TOPIC = "/capricorn/";
  const std::string POSE_ERROR_TOPIC = "/pose_error";
  const std::string GROUND_TRUTH_TOPIC = "/ground_truth";
  const std::string PR_GROUND_TRUTH_TOPIC = "/pr_ground_truth";
  const std::string CHEAT_ODOM_TOPIC = "/cheat_odom";
  const std::string RIGHT_IMAGE_RAW_TOPIC = "/camera/right/image_raw";
  const std::string LEFT_IMAGE_RAW_TOPIC = "/camera/left/image_raw";
  const std::string RIGHT_CAMERAINFO_TOPIC = "/camera/right/camera_info";
  const std::string LEFT_CAMERAINFO_TOPIC = "/camera/left/camera_info";
  const std::string SET_SENSOR_PITCH_TOPIC = "/sensor/pitch/command/position";
  const std::string WHEEL_PID = "/wheel_pid";
  const std::string SET_SENSOR_YAW_TOPIC = "/sensor/yaw/command/position";
  const std::string OBJECT_DETECTION_OBJECTS_TOPIC = "/object_detection/objects";
  const std::string VOLATILE_SENSOR_TOPIC = "/volatile_sensor";

  /****** OBJECT DETECTION CLASS NAMES ******/
  const std::string OBJECT_DETECTION_PROCESSING_PLANT_CLASS = "processingPlant";
  const std::string OBJECT_DETECTION_REPAIR_STATION_CLASS = "repairStation";

  /****** NAVIGATION ACTION RESULT ENUM ******/
  enum NAV_RESULT
  {
    FAILED = 0,
    SUCCESS = 1,
    INTERRUPTED = 2
  };

  
} // namespace CAPRICORN_COMMON_NAMES
