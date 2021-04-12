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
  const std::string ROBOT_CHASSIS = "_small_chassis";

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
  const std::string HAULER_ACTIONLIB = "hauler_bin";
  const std::string EXCAVATOR_ACTIONLIB = "excavator";
  const std::string PARK_HAULER_ACTIONLIB = "park_hauler";
  const std::string FIND_PP_RS_ACTIONLIB = "_find_pp_rs";
  const std::string NAVIGATION_VISION_ACTIONLIB = "_navigation_vision";

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

  /****** ROS NODE NAMES ******/
  const std::string NAVIGATION_VISION_SERVER_NODE_NAME = "_navigation_vision_server";
  const std::string NAVIGATION_VISION_CLIENT_NODE_NAME = "_navigation_vision_server";
  const std::string PR_DATASET_NODE_NAME = "_pr_dataset";
  const std::string GROUND_TRUTH_PR_NODE_NAME = "_ground_truth_pr";
  const std::string PR_LOCALIZATION_NODE_NAME = "_pr_localization";
  const std::string INITALIZE_ODOM_NODE_NAME = "_odom_initialize";
  const std::string CHEAT_ODOM_PUB_NODE_NAME = "_cheat_odom_publisher";
  const std::string ODOM_ERROR_NODE_NAME = "_odom_errorr";
  const std::string NOISY_IMAGE_NODE_NAME = "_noisy_image_eliminate";
  const std::string HORIZON_TRACKING_NODE_NAME = "_horzion_tracking";
  const std::string FIND_PP_RS_SERVER_NODE_NAME = "_find_pp_rs_server";
  const std::string PARK_HAULER_HOPPER_SERVER_NODE_NAME = "_park_hauler_server_hopper";
  const std::string PARK_HAULER_HOPPER_CLIENT_NODE_NAME = "_park_hauler_vision_hopper";

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
  const std::string VOLATILE_LOCATION_TOPIC = "/volatile_location";
  const std::string SCHEDULER_TOPIC = "/scheduler";

  /****** HAULER NAMES ******/
  const std::string SET_BIN_POSITION = "/bin/command/position";

  /****** EXCAVATOR NAMES ******/
  const std::string SET_ELBOW_PITCH_POSITION = "/arm/elbow_pitch/command/position";
  const std::string SET_SHOULDER_PITCH_POSITION = "/arm/shoulder_pitch/command/position";
  const std::string SET_SHOULDER_YAW_POSITION = "/arm/shoulder_yaw/command/position";
  const std::string SET_WRIST_PITCH_POSITION = "/arm/wrist_pitch/command/position";

  const std::string WHEEL_PID = "/wheel_pid";
  const std::string SET_SENSOR_YAW_TOPIC = "/sensor/yaw/command/position";
  const std::string OBJECT_DETECTION_OBJECTS_TOPIC = "/object_detection/objects";
  const std::string VOLATILE_SENSOR_TOPIC = "/volatile_sensor";

  // Used to communicate between excavators and scouts when the excavator is ready to move in to pick up a volatile
  // TODO: Choose a real message type for this topic, instead of std_msgs::Empty
  const std::string EXCAVATOR_ARRIVED_TOPIC = "/excavator_arrived";

  /****** OBJECT DETECTION CLASS NAMES ******/
  const std::string OBJECT_DETECTION_PROCESSING_PLANT_CLASS = "processingPlant";
  const std::string OBJECT_DETECTION_REPAIR_STATION_CLASS = "repairStation";
  const std::string OBJECT_DETECTION_EXCAVATOR_CLASS = "excavator";
  const std::string OBJECT_DETECTION_EXCAVATOR_ARM_CLASS = "excavatorArm";
  const std::string OBJECT_DETECTION_SCOUT_CLASS = "scout";
  const std::string OBJECT_DETECTION_HAULER_CLASS = "hauler";
  const std::string OBJECT_DETECTION_FURNACE_CLASS = "furnace";
  const std::string OBJECT_DETECTION_HOPPER_CLASS = "hopper";
  const std::string OBJECT_DETECTION_ROBOT_ANTENNA_CLASS = "robotAntenna";
  const std::string OBJECT_DETECTION_PP_SMALL_THRUSTER_CLASS = "ppSmallThruster";
  const std::string OBJECT_DETECTION_ROCK_CLASS = "rock";

  /****** NAVIGATION ENUMS ******/
  enum NAV_TYPE
  {
    MANUAL,  // Manual driving
    GOAL,    // Trajectory generation with the planner from a goal
    REVOLVE, // Revolve the robot around a fixed point
    SPIRAL,  // Archimedean spiral (scout finding volatiles)
    FOLLOW,  // Follow an object in frame
  };
  
  enum NAV_RESULT
  {
    FAILED,
    SUCCESS,
    INTERRUPTED
  };

} // namespace CAPRICORN_COMMON_NAMES
