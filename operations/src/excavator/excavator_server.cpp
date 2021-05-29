#include <operations/ExcavatorAction.h>
#include <actionlib/server/simple_action_server.h>
#include <string.h>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <srcp2_msgs/ExcavatorScoopMsg.h>
#include <utils/common_names.h>
#include <geometry_msgs/Point.h> // To get target point in order to orient shoulder joint
#include <math.h>                // used in findShoulderAngle() for atan2()
#include <mutex>

#include <perception/ObjectArray.h>
#include <perception/Object.h>
#include <operations/navigation_algorithm.h>

#include <tf/transform_listener.h>

using namespace COMMON_NAMES;

// Initialization for joint angle publishers
typedef actionlib::SimpleActionServer<operations::ExcavatorAction> Server;
ros::Publisher excavator_shoulder_yaw_publisher_;
ros::Publisher excavator_shoulder_pitch_publisher_;
ros::Publisher excavator_elbow_pitch_publisher_;
ros::Publisher excavator_wrist_pitch_publisher_;

// The global variables to save the last values passed to the joints
float curr_sh_yaw = 0;
float curr_sh_pitch = 0;
float curr_elb_pitch = 0;
float curr_wrt_pitch = 0;
bool volatile_found = false; // flag to store value received from scoop_info topic
std::mutex excavator_cancel_goal_mutex;

int SLEEP_DURATION = 5; // The sleep duration

// Variables for object detection of hauler with respect to excavator
perception::ObjectArray objectArray;
std::string desired_label = OBJECT_DETECTION_HAULER_CLASS;
std::mutex objects_mutex;
bool objects_received = 0;


/**
 * @brief Initializing the publisher here
 * 
 * @param nh nodeHandle
 *
 * @param robot_name Passed in the terminal/launch file to target a particular rover
 */
void initExcavatorPublisher(ros::NodeHandle &nh, const std::string &robot_name)
{
  excavator_shoulder_yaw_publisher_ = nh.advertise<std_msgs::Float64>("/" + robot_name + SET_SHOULDER_YAW_POSITION, 1000);
  excavator_shoulder_pitch_publisher_ = nh.advertise<std_msgs::Float64>("/" + robot_name + SET_SHOULDER_PITCH_POSITION, 1000);
  excavator_elbow_pitch_publisher_ = nh.advertise<std_msgs::Float64>("/" + robot_name + SET_ELBOW_PITCH_POSITION, 1000);
  excavator_wrist_pitch_publisher_ = nh.advertise<std_msgs::Float64>("/" + robot_name + SET_WRIST_PITCH_POSITION, 1000);
}

/**
 * @brief The call back for scoop info topic
 * 
 * @param msg  The scoop info message
 */
void scoopCallback(const srcp2_msgs::ExcavatorScoopMsg::ConstPtr &msg)
{
  volatile_found = msg->volatile_clod_mass;
}

/**
 * @brief Callback function which subscriber to Objects message published from object detection
 * 
 * @param objs 
 */
void objectsCallback(const perception::ObjectArray &objs)
{
    const std::lock_guard<std::mutex> lock(objects_mutex);
    objects_received = true;
    objectArray = objs;
}

/**
 * @brief Calculates the orientation of the shoulder joint based on the passed target point
 * 
 * @param target the x, y coordinates of the volatiles in the body frame of the excavator
 * @param shoulder the x, y coordinates of the shoulder joint in the body frame of excavator
 * @return atan2((target.y - shoulder.y), (target.x - shoulder.x)) is the required yaw angle of the shoulder joint
 */
float findShoulderAngle(const geometry_msgs::Point &target)
{
  geometry_msgs::Point shoulder_wrt_base;
  shoulder_wrt_base.x = 0.7;
  shoulder_wrt_base.y = 0.000001;
  shoulder_wrt_base.z = 0.100000;

  return atan2((target.y - shoulder_wrt_base.y), (target.x - shoulder_wrt_base.x));
}

/**
 * @brief Prints a point to the terminal with description - FOR DEBUGGING ONLY
 * 
 * @param description descrition of the point
 * @param point point values
 */
void printPoint(std::string const &description, geometry_msgs::Point &point)
{
    ROS_INFO_STREAM(description << ": [" << point.x << ", " << point.y << ", " << point.z << "]");
}

/**
 * @brief Get the hauler pose in camera frame through object detection
 * 
 * @param stamped_point the point to store the information in
 * @return geometry_msgs::PoseStamped 
 */
geometry_msgs::PoseStamped getHaulerPose(geometry_msgs::PointStamped stamped_point)
{

  geometry_msgs::PoseStamped currentHaulerPoseStamped;
  currentHaulerPoseStamped.pose.position = stamped_point.point; 
  perception::ObjectArray objects = objectArray;

  bool pointDetected = false;

   for(int i = 0; i < objects.number_of_objects; i++) 
    {   
        perception::Object object = objects.obj.at(i);
        if(object.label == desired_label) {
            // Store the object's location
            currentHaulerPoseStamped= object.point;
            pointDetected = true;
            break;
        }
    }

    if (!pointDetected)
      ROS_INFO_STREAM("Hauler not detected, using default hauler position ([0, 0, 2] in left camera optical frame)");
    
    return currentHaulerPoseStamped;
}

/**
 * @brief Get the dumping point location in base footprint frame - CODE DUPLICATION OF NAVIGATION ALGORITHM -> TRANSFORM POINT
 * 
 * @param tries Number of times transform should be attempted
 * @return geometry_msgs::Point The dumping point loaction in base footprint frame
 */
float getDumpAngleInBase(int tries)
{
  tf::TransformListener tf_listener_; // For transformation from camera frame to shoulder frame

  std::string base_frame = "small_excavator_1" + ROBOT_BASE;
  std::string left_camera_frame = "small_excavator_1" + LEFT_CAMERA_ROBOT_LINK;

  bool transformSet = false; // flag to keep track of when valid frames are found
  int countTries = 0; // counter to keep track of tries
  
  geometry_msgs::PointStamped initial_point_stamped; // object to store detected hauler point in base frame
  initial_point_stamped.point.z = 2.0; // Default assumed location is [0, 0, 2] in left camera frame in case object detection fails

  printPoint("Default point in left camera frame", initial_point_stamped.point);

  initial_point_stamped.point = getHaulerPose(initial_point_stamped).pose.position; // try to get hauler location from object detection

  printPoint("Detected point in left camera frame", initial_point_stamped.point);

  geometry_msgs::PointStamped final_point_stamped; // point stamped object to store transformation of point from camera frame to base frame

  /*
    Set the frames for point stamped objects, required for transformation
  */
  initial_point_stamped.header.frame_id = left_camera_frame;
  final_point_stamped.header.frame_id = base_frame;

  while(countTries<tries && !transformSet) {
        try{
            tf_listener_.transformPoint(base_frame, initial_point_stamped, final_point_stamped);
            transformSet = true;
        }
        catch (tf::TransformException &ex) {
            ROS_INFO_STREAM("Could not transform hauler detection from " << left_camera_frame << " to " << base_frame);
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        countTries++;
    }

  printPoint("Transformed point in base frame", final_point_stamped.point);

  return findShoulderAngle(final_point_stamped.point);
}


/**
 * @brief This function publishes the joint angles to the publishers
 * 
 * @param shoulder_yaw the desired shoulder yaw joint value
 * @param shoulder_pitch the desired shoulder pitch joint value
 * @param elbow_pitch the desired elbow pitch joint value
 * @param wrist_pitch the desired wrist pitch joint value
 * @param steps the number of simulation steps, higher value corresponds to slower movement
 * @param dig flag for dig or dump task to adjust the speed
 */
void publishAngles(float shoulder_yaw, float shoulder_pitch, float elbow_pitch, float wrist_pitch)
{
  std_msgs::Float64 shoulder_yaw_msg;
  std_msgs::Float64 shoulder_pitch_msg;
  std_msgs::Float64 elbow_pitch_msg;
  std_msgs::Float64 wrist_pitch_msg;

  shoulder_yaw_msg.data = shoulder_yaw;
  shoulder_pitch_msg.data = shoulder_pitch;
  elbow_pitch_msg.data = elbow_pitch;
  wrist_pitch_msg.data = wrist_pitch;

  excavator_shoulder_yaw_publisher_.publish(shoulder_yaw_msg);
  excavator_shoulder_pitch_publisher_.publish(shoulder_pitch_msg);
  excavator_elbow_pitch_publisher_.publish(elbow_pitch_msg);
  excavator_wrist_pitch_publisher_.publish(wrist_pitch_msg);
}

/**
 * @brief publishes the excavator angles to the rostopics small_excavator_1/arm/*joint_name/position
 * 
 * @param task the excavator task to be accomplished
 * @param target the target x, y coordinates in terms of the body frame
 * @param shoulder the fixed coordinates of the shoulder joint in body frame
 */
bool publishExcavatorMessage(int task, const geometry_msgs::Point &target, const geometry_msgs::Point &shoulder)
{
  float dumpAngle = getDumpAngleInBase(3);

  // float theta = findShoulderAngle(target, shoulder);
  float theta = -1.57;
  static float last_vol_loc_angle = -1.57;
  float yaw_angle;

  if (target.x == 1)
      last_vol_loc_angle = theta;

  std::string scoop_value;
  if (task == START_DIGGING) // digging angles
  {
    publishAngles(0, -2, 1, 0);     // Move the arm up
    publishAngles(last_vol_loc_angle, -2, 1, 0); // Step for safe trajectory to not bump into camera
    ros::Duration(2).sleep();
    publishAngles(last_vol_loc_angle, 1, 1, -2); // This set of values move the scoop under the surface
    ros::Duration(5).sleep();

    scoop_value = volatile_found ? "Volatile found" : "Volatile not found"; // Prints to the terminal if volatiles found
    ROS_INFO_STREAM("Scoop info topic returned: " + scoop_value + "\n");

    while (!volatile_found && last_vol_loc_angle < 1.2) // Logic for panning the shoulder yaw angle to detect volatiles with scoop info under the surface
    {
      // move the shoulder yaw joint from right to left under the surface
      publishAngles(last_vol_loc_angle, 1, 1, -0.6);
      ros::Duration(3).sleep();
      last_vol_loc_angle += 0.2;
      ROS_INFO_STREAM(std::to_string(last_vol_loc_angle));
      scoop_value = volatile_found ? "Volatile found" : "Volatile not found";
      ROS_INFO_STREAM("Scoop info topic returned: " + scoop_value + "\n");
    }

    yaw_angle = last_vol_loc_angle;

    if (yaw_angle < 0.785 && yaw_angle > -0.785) // If digging happens towards the front of excavator
    {
      if (volatile_found) // If volatiles found towards the center, move to the rightmost position and raise the arm
      {
        publishAngles(-0.785, 1, 1, -2.6); // Set of values moves the arm to the right while inside the surface
        ros::Duration(2).sleep();
        publishAngles(-0.785, -0.5, 1, -1.1); // Intermediate set of values for smooth motion
        ros::Duration(2).sleep();
        publishAngles(-0.785, -2, 1, 0.4); // This set of values moves the scoop over the surface
      }
    }
    else // Else raise the arm where volatiles were found or drop regolith to the left
    {
      if (volatile_found)
      {
        publishAngles(yaw_angle, 1, 1, -2.6); // Set of values moves the scoop to not drop volatiles
        ros::Duration(2).sleep();
        publishAngles(yaw_angle, -0.5, 1, -1.1); // Intermediate set of values to raise the arm above the surface
        ros::Duration(2).sleep();
        publishAngles(yaw_angle, -2, 1, 0.4); // This set of values moves the arm over the surface
      }
      else // Else raise the arm and dump the regolith in the left
      {
        publishAngles(1.57, -2, 1, 0.4); // This set of values moves the arm to the left and above the surface
        ros::Duration(SLEEP_DURATION).sleep();
        publishAngles(1.57, -2, 1, 1.5); // This set of values moves the scoop to drop regolith on the ground
        ros::Duration(SLEEP_DURATION).sleep();
        last_vol_loc_angle = theta;
        return false;
      }
    }
  }
  else if (task == START_UNLOADING) // dumping angles
  {
    // previous shoulder yaw was 0.15
    publishAngles(dumpAngle, -2, 1, 0.4); // This set of values moves the scoop towards the hauler
    ros::Duration(SLEEP_DURATION).sleep();
    publishAngles(dumpAngle, -2, 1, 1.5); // This set of values moves the scoop to deposit volatiles in the hauler bin
    ros::Duration(SLEEP_DURATION).sleep();
    publishAngles(dumpAngle, -2, 1, -0.7786); // This set of values moves the scoop to the front center
    ros::Duration(3).sleep();
  }
  else if (task == GO_TO_DEFAULT) // dumping angles
  {
    publishAngles(-1, -1, 1.5792, -0.7786);
  }
  else
  {
    ROS_ERROR("Unhandled state encountered in Excavator actionlib server");
  }
  return true;
}

/**
 * @brief This is where the action to dig or dump are executed
 * 
 * @param goal The desired excavator task
 * @param action_server The server object for excavator action
 */
void execute(const operations::ExcavatorGoalConstPtr &goal, Server *action_server) //const operations::ExcavatorFeedbackConstPtr& feedback,
{
  geometry_msgs::Point shoulder_wrt_base_footprint; // Transforming from robot base frame to shoulder joint frame
  shoulder_wrt_base_footprint.x = 0.7;              // Value from tf topic from the simulation, all values in meters
  shoulder_wrt_base_footprint.y = 0.0;
  shoulder_wrt_base_footprint.z = 0.1;

  bool dig_dump_result = publishExcavatorMessage(goal->task, goal->target, shoulder_wrt_base_footprint);
  ros::Duration(SLEEP_DURATION).sleep();
  // action_server->working(); // might use for feedback

  // SUCCEESS or FAILED depending upon if it could find volatile
  dig_dump_result ? action_server->setSucceeded() : action_server->setAborted();
}

/**
 * @brief Function called when the goal is cancelled
 * 
 */
void cancelGoal()
{
  std_msgs::Float64 shoulder_yaw_msg;
  shoulder_yaw_msg.data = -1.57;

  ROS_INFO("Cancelled Excavator Goal");
  const std::lock_guard<std::mutex> lock(excavator_cancel_goal_mutex);
  ROS_INFO("Done Cancelling");
  excavator_shoulder_yaw_publisher_.publish(shoulder_yaw_msg);
  ros::Duration(SLEEP_DURATION).sleep();
  publishAngles(-1.57, -2, 1, 1.5); // This set of values moves the scoop to deposit volatiles in the hauler bin
  ros::Duration(SLEEP_DURATION).sleep();
}

/**
 * @brief The main method for excavator server
 * 
 * @param argc The number of arguments passed
 * @param argv The array of arguments passed
 * @return int 
 */
int main(int argc, char **argv)
{

  // Check if the node is being run through roslauch, and have one parameter of RobotName_Number
  if (argc != 2 && argc != 4)
  {
    // Displaying an error message for correct usage of the script, and returning error.
    ROS_ERROR_STREAM("This Node needs an argument as <RobotName_Number>");
    return -1;
  }
  else
  {
    std::string robot_name = (std::string)argv[1];
    ROS_INFO_STREAM(robot_name + "\n");
    std::string node_name = robot_name + "_excavator_action_server";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    initExcavatorPublisher(nh, robot_name);
    ros::Subscriber sub = nh.subscribe("/" + robot_name + SCOOP_INFO, 1000, scoopCallback); // scoop info subscriber
    Server server(nh, EXCAVATOR_ACTIONLIB, boost::bind(&execute, _1, &server), false);
    // server.registerPreemptCallback(&cancelGoal);
    server.start();
    ROS_INFO("STARTED EXCAVATOR SERVER");
    ros::spin();

    return 0;
  }
}