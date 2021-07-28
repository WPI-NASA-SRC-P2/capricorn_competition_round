#pragma once
#include <sstream>

namespace COMMON_NAMES
{
  /****** ROBOTS ******/
  const std::string SCOUT_1_NAME = "small_scout_1";
  const std::string SCOUT_2_NAME = "small_scout_2";
  const std::string SCOUT_3_NAME = "small_scout_3";

  const std::string EXCAVATOR_1_NAME = "small_excavator_1";
  const std::string EXCAVATOR_2_NAME = "small_excavator_2";
  const std::string EXCAVATOR_3_NAME = "small_excavator_3";

  const std::string HAULER_1_NAME = "small_hauler_1";
  const std::string HAULER_2_NAME = "small_hauler_2";
  const std::string HAULER_3_NAME = "small_hauler_3";

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
  const std::string SOLAR_RECHARGE_ACTIONLIB = "solar_recharging";


  /****** SERVICES ******/
  const std::string SCOUT_SEARCH_SERVICE = "scout_search";
  const std::string SET_ROBOT_STATE_SRV = "/set_robot_state";

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
  const std::string LOCALIZATION_ODOM_TOPIC = "/odom/localization";
  const std::string RESET_LOCALIZATION_POSE = "/set_pose";
  // const std::string RTAB_ODOM_TOPIC = "/camera/odom/localization";

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
  const std::string SOLAR_CHARGING_CLIENT_NODE_NAME = "_solar_charging_client";


  /****** TOPIC NAMES ******/
  const std::string CLOCK_TOPIC = "/clock";
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
  const std::string NAV_TYPE_TOPIC = "/nav_type_topic";
  const std::string RAMP_DONE_TOPIC = "/ramp_done";
  const std::string ROBOTS_CURRENT_STATE_TOPIC = "robot_state_status";
  const std::string ROBOTS_DESIRED_STATE_TOPIC = "robot_desired_state";
  const std::string REPLAN_TRAJECTORY = "/replan_trajectory";
  const std::string ROBOTS_OUT_OF_COMMISSION_TOPIC = "robots_out_of_commission";
  const std::string IMU_TOPIC = "/imu";
  const std::string IMU_FILTERED_TOPIC = "/imu_filtered";
  const std::string SPIRAL_WAYPOINT_PUBLISHER = "/covered_waypoints";
  const std::string MAP_RESET_TOPIC = "/odom_reset_map_reset";

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
  const std::string SCOUT_LOC_TOPIC = "/scout_loc";

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
    V_NAV_AND_NAV_VISION = 5, // Navigation which switches to vision based navigation when robot can see target and target location is near the robot
    V_HARDCODED_UNDOCK = 6,   // Hardcoded undock for resetting odom at Proc Plant
    V_PPM = 7,   // Hardcoded undock for resetting odom at Proc Plant
  };

  /****** NAVIGATION ENUMS ******/
  enum NAV_TYPE
  {
    MANUAL,      // Manual driving
    GOAL_OLD,    // Trajectory generation with the planner from a goal (using turn-in-place)
    GOAL,        // Follow a path, but do it smoothly instead of turn-in-place
    REVOLVE,     // Revolve the robot around a fixed point
    SPIRAL,      // Archimedean spiral (scout finding volatiles)
    FOLLOW,      // Follow an object in frame
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
    SCOUT_SEARCH_VOLATILE, // Execute spiral motion to search for the volatiles.
    SCOUT_GO_TO_LOC,       // Sends scout to the given location
    SCOUT_STOP_SEARCH,     // Stop executing the search algorithm.
    SCOUT_LOCATE_VOLATILE, // Pinpoint the location of the volatile
    SCOUT_UNDOCK,          // Move the Scout away from the Excavator
    SCOUT_FACE_PROCESSING_PLANT, // Rotate the Scout using nav vision centering to proc plant
    SCOUT_SYNC_ODOM,       // synchronize odometry of scout with excavator 
    SCOUT_RESET_ODOM,      // reset odometry state for scout (calls the reset service)
    SCOUT_RESET_ODOM_GROUND_TRUTH, // reset odometry of scout w.r.t. its starting position in sim
    SCOUT_GOTO_REPAIR_STATION,   // Scout goes to repair station
    SCOUT_PARK_REPAIR_STATION,   // Scout parks at repair station
    SCOUT_VISUAL_RESET_ODOM,

    /**************EXCAVATOR STATES**************/  
    //11
    EXCAVATOR_GO_TO_LOC,             // Takes Excavator to a location from which it will
                                         // be quicker to get to the digging location
    EXCAVATOR_GO_TO_SCOUT,           // Get close to the volatile when it is detected
    EXCAVATOR_PARK_AND_PUB,          // Publish a message that excavator has reached,
                                         // And park where the scout was located.
    EXCAVATOR_PRE_HAULER_PARK_MANEUVER,     // Centering the hauler for ease of parking
    EXCAVATOR_DIG_AND_DUMP_VOLATILE, // Takes care of digging, and dumpin g
                                         // the volatile in hauler if volatile is found
    EXCAVATOR_GOTO_DEFAULT_ARM_POSE, // Moves excavator's arm to a default position used for object detection
    EXCAVATOR_RESET_ODOM_GROUND_TRUTH, // reset odometry of excavator w.r.t. its starting position in sim
    EXCAVATOR_RESET_ODOM,             // reset excavator odometry using reset odom service
    EXCAVATOR_SYNC_ODOM,              // reset excavator odometry w.r.t. hauler odometry 
    EXCAVATOR_FACE_PROCESSING_PLANT,  //Face processing plant using NAV_VISION::V_CENTER
    EXCAVATOR_GO_TO_REPAIR,           //Go to repair station using NAV_VISION
    EXCAVATOR_RESET_ODOM_AT_HOPPER,    // Excavator resets odometry at hopper
    EXCAVATOR_GO_TO_SCOUT_RECOVERY,
    EXCAVATOR_VOLATILE_RECOVERY,
    EXCAVATOR_GO_TO_INIT_LOCATION,
    EXCAVATOR_GO_TO_LOOKOUT_LOCATION,
    EXCAVATOR_BALLET_DANCING,
    EXCAVATOR_VISUAL_RESET_ODOM,
    EXCAVATOR_PRE_PARK_MANEUVER_RECOVERY,
    
    /**************HAULER STATES**************/
    //25
    HAULER_GO_TO_LOC,                    // Takes Hauler to a location
    HAULER_DUMP_VOLATILE_TO_PROC_PLANT, // Undocks hauler from excavator, goes to processing plant,
                                             // parks hauler to processing plant, dumps volatile and
                                             // undocks hauler from hopper
    HAULER_GO_BACK_TO_EXCAVATOR,        // Takes hauler from any location to excavator and parks
    HAULER_PARK_AT_EXCAVATOR,           // Hauler parks at excavator
    HAULER_FOLLOW_EXCAVATOR,            // Hauler follows excavator
    HAULER_RESET_ODOM,                  // Hauler reset odometry service call state
    HAULER_GO_TO_INIT_LOC,

    // redundant modes for hauler (everything is taken care by above modes)
    HAULER_GO_TO_PROC_PLANT, // Hauler goes to processing plant
    HAULER_PARK_AT_HOPPER,   // Parks hauler wrt hopper
    HAULER_DUMP_VOLATILE,    // Empty hauler's bin
    HAULER_UNDOCK_EXCAVATOR, // undock from excavator (basically backward motion from excavator)
    HAULER_UNDOCK_HOPPER,    // undock from hopper (backward motion from hopper)
    HAULER_RESET_ODOM_AT_HOPPER, //face processing plant, park at hopper and then reset odom with ground truth
    HAULER_FACE_PROCESSING_PLANT, //face the processinf plant using NAV_VISION_TYPE::V_CENTER
    HAULER_GO_TO_SCOUT,      // Hauler goes to the scout using NAV_VISION_TYPE::V_NAV_AND_NAV_VISION
    HAULER_GOTO_REPAIR_STATION, // Hauler goes to the repair station using NAV_VISION_TYPE::V_REACH to recharge
    HAULER_GO_TO_EXCAVATOR_RECOVERY,
    HAULER_GO_TO_LOOKOUT_LOCATION,
    HAULER_BALLET_DANCING,
    HAULER_VISUAL_RESET_ODOM,
    HAULER_INITIAL_RESET,
    HAULER_DO_NOTHING,
    ROBOT_IDLE_STATE,

    /***************ROBOTS MACRO STATES***************/
    //41
    SCOUT_MACRO_UNDOCK,
    EXCAVATOR_MACRO_GO_TO_SCOUT,
    EXCAVATOR_MACRO_DIG,
    HAULER_MACRO_DUMP
  };

  enum ROBOTS_ENUM
  {
    NONE,
    SCOUT_1,
    SCOUT_2,
    SCOUT_3,
    EXCAVATOR_1,
    EXCAVATOR_2,
    EXCAVATOR_3,
    HAULER_1,
    HAULER_2,
    HAULER_3
  };

} // namespace CAPRICORN_COMMON_NAMES

/****** EXCAVATOR TASK ENUM ******/
enum EXCAVATOR_ARM_TASK
{
  START_DIGGING = 1,
  CHECK_VOLATILE = 2,
  START_UNLOADING = 3,
  GO_TO_DEFAULT = 4,
  RECOVERY = 5,
};


template<typename T>
std::string ToString(T& t_arg) {
   std::ostringstream oss;
   oss << t_arg;
   return oss.str();
}

template<typename T, typename... Ts>
std::string ToString(T& t_arg, Ts... t_args) {
   std::ostringstream oss;
   oss << t_arg << ToString(t_args...);
   return oss.str();
}