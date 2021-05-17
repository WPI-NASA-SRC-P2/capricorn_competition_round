#pragma once

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
  const std::string ROBOT_BASE = "_base_footprint";
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
  const std::string STATE_MACHINE_ACTIONLIB = "_state_machine";

  /****** SERVICES ******/
  const std::string SCOUT_SEARCH_SERVICE = "scout_search";

  /****** GAZEBO ******/
  const std::string HEIGHTMAP = "heightmap";
  const std::string PROCESSING_PLANT_GAZEBO = "processing_plant";
  const std::string REPAIR_STATION_GAZEBO = "repair_station";
  const std::string PROCESSING_PLANT_LINK_GAZEBO = "processing_plant_link";
  const std::string REPAIR_STATION_LINK_GAZEBO = "repair_station_link";
  const std::string MODEL_STATE_QUERY = "/gazebo/get_model_state";
  const std::string LINK_STATE_QUERY = "/gazebo/get_link_state";
  const std::string SENSOR_BAR_GAZEBO = "_sensor_bar";

  /****** ROBOT LINKS ******/
  const std::string LEFT_CAMERA_ROBOT_LINK = "_left_camera_optical";

  /****** RTABMAP ******/
  const std::string RESET_POSE_CLIENT = "/camera/reset_odom_to_pose";
  const std::string TRUE_POSE_SRV = "/get_true_pose";
  const std::string RESET_ODOMETRY = "reset_rover_odom_srv";
  const std::string RTAB_ODOM_TOPIC = "/camera/odom";

  /****** ROS NODE NAMES ******/
  const std::string NAVIGATION_VISION_SERVER_NODE_NAME = "_navigation_vision_server";
  const std::string NAVIGATION_VISION_CLIENT_NODE_NAME = "_navigation_vision_client";
  const std::string PR_DATASET_NODE_NAME = "_pr_dataset";
  const std::string GROUND_TRUTH_PR_NODE_NAME = "_ground_truth_pr";
  const std::string PR_LOCALIZATION_NODE_NAME = "_pr_localization";
  const std::string INITALIZE_ODOM_NODE_NAME = "_odom_initialize";
  const std::string CHEAT_ODOM_PUB_NODE_NAME = "_cheat_odom_publisher";
  const std::string ODOM_ERROR_NODE_NAME = "_odom_errorr";
  const std::string NOISY_IMAGE_NODE_NAME = "_noisy_image_eliminate";
  const std::string HORIZON_TRACKING_NODE_NAME = "_horzion_tracking";
  const std::string FIND_PP_RS_SERVER_NODE_NAME = "_find_pp_rs_server";
  const std::string PARK_HAULER_HOPPER_SERVER_NODE_NAME = "_park_hauler_server";
  const std::string PARK_HAULER_HOPPER_CLIENT_NODE_NAME = "_park_hauler_client";
  const std::string SCOUT_SEARCH_NODE_NAME = "_scout_search";
  const std::string STATE_MACHINE_SERVER_NODE_NAME = "_sm_server";

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
  const std::string HAULER_FILLED = "/hauler_filled";
  const std::string LOOKOUT_LOCATION_TOPIC = "/lookout_location";
  const std::string PARK_HAULER = "/park_hauler";
  const std::string HAULER_PARKED_TOPIC = "/hauler_parked";

  /****** HAULER NAMES ******/
  const std::string SET_BIN_POSITION = "/bin/command/position";

  /****** EXCAVATOR NAMES ******/
  const std::string SET_ELBOW_PITCH_POSITION = "/arm/elbow_pitch/command/position";
  const std::string SET_SHOULDER_PITCH_POSITION = "/arm/shoulder_pitch/command/position";
  const std::string SET_SHOULDER_YAW_POSITION = "/arm/shoulder_yaw/command/position";
  const std::string SET_WRIST_PITCH_POSITION = "/arm/wrist_pitch/command/position";
  const std::string SCOOP_INFO = "/scoop_info";

  const std::string WHEEL_PID = "/wheel_pid";
  const std::string SET_SENSOR_YAW_TOPIC = "/sensor/yaw/command/position";
  const std::string OBJECT_DETECTION_OBJECTS_TOPIC = "/object_detection/objects";
  const std::string VOLATILE_SENSOR_TOPIC = "/volatile_sensor";

  // Used to communicate between excavators and scouts when the excavator is ready to move in to pick up a volatile
  // TODO: Choose a real message type for this topic, instead of std_msgs::Empty
  const std::string EXCAVATOR_ARRIVED_TOPIC = "/excavator_arrived";
  const std::string HAULER_ARRIVED_TOPIC = "/hauler_arrived";

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

  /****** NAVIGATION VISION ENUMS ******/
  enum NAV_VISION_TYPE
  {
    V_REACH = 0,         // Reach the goal and stop
    V_FOLLOW = 1,        // Follow an object
    V_CENTER = 2,        // Centers the robot to a given class
    V_UNDOCK = 3,        // Undocks from given class
    V_OBS_GOTO_GOAL = 4, // Uses go to goal with obstacle avoidance
  };

  /****** NAVIGATION ENUMS ******/
  enum NAV_TYPE
  {
    MANUAL,  // Manual driving
    GOAL,    // Trajectory generation with the planner from a goal
    REVOLVE, // Revolve the robot around a fixed point
    SPIRAL,  // Archimedean spiral (scout finding volatiles)
    FOLLOW,  // Follow an object in frame
  };

  /****** COMMON RESULTS ENUMS (This is used by every actionlibrary)******/
  enum COMMON_RESULT
  {
    FAILED,
    SUCCESS,
    INTERRUPTED,
    INVALID_GOAL,
  };

  /****** STATE MACHINE ENUM ******/
  enum STATE_MACHINE_TASK
  {
    /**************SCOUT STATES**************/
    SCOUT_SEARCH_VOLATILE = 0, // Execute spiral motion to search for the volatiles.
    SCOUT_STOP_SEARCH = 1,     // Stop executing the search algorithm.
    SCOUT_LOCATE_VOLATILE = 2, // Pinpoint the location of the volatile
    SCOUT_UNDOCK = 3,          // Move the Scout away from the Excavator

    /**************EXCAVATOR STATES**************/
    EXCAVATOR_GO_TO_LOC = 4,             // Takes Excavator to a location from which it will
                                         // be quicker to get to the digging location
    EXCAVATOR_GO_TO_SCOUT = 5,           // Get close to the volatile when it is detected
    EXCAVATOR_PARK_AND_PUB = 6,          // Publish a message that excavator has reached,
                                         // And park where the scout was located.
    EXCAVATOR_DIG_AND_DUMP_VOLATILE = 7, // Takes care of digging, and dumping
                                         // the volatile in hauler if volatile is found
    EXCAVATOR_GOTO_DEFAULT_ARM_POSE = 8, // Moves excavator's arm to a default position used for object detection

    /**************HAULER STATES**************/
    HAULER_GO_TO_LOC = 9,                    // Takes Hauler to a location
    HAULER_DUMP_VOLATILE_TO_PROC_PLANT = 10, // Undocks hauler from excavator, goes to processing plant,
                                             // parks hauler to processing plant, dumps volatile and
                                             // undocks hauler from hopper
    HAULER_GO_BACK_TO_EXCAVATOR = 11,        // Takes hauler from any location to excavator and parks
    HAULER_PARK_AT_EXCAVATOR = 12,           // Hauler parks at excavator
    HAULER_FOLLOW_EXCAVATOR = 13,            // Hauler follows excavator

    // redundant modes for hauler (everything is taken care by above modes)
    HAULER_GO_TO_PROC_PLANT = 14, // Hauler goes to processing plant
    HAULER_PARK_AT_HOPPER = 16,   // Parks hauler wrt hopper
    HAULER_DUMP_VOLATILE = 17,    // Empty hauler's bin
    HAULER_UNDOCK_EXCAVATOR = 18, // undock from excavator (basically backward motion from excavator)
    HAULER_UNDOCK_HOPPER = 19,    // undock from hopper (backward motion from hopper)
  };

} // namespace CAPRICORN_COMMON_NAMES

/****** EXCAVATOR TASK ENUM ******/
enum EXCAVATOR_ARM_TASK
{
  START_DIGGING = 1,
  START_UNLOADING = 2,
  GO_TO_DEFAULT = 3,
};
