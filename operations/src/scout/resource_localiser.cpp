#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <operations/ResourceLocaliserAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <srcp2_msgs/VolSensorMsg.h>
#include <utils/common_names.h>

// typedef for the Action Server and Client
typedef actionlib::SimpleActionServer<operations::ResourceLocaliserAction> ResourceLocaliserServer;
typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient_;
NavigationClient_ *navigation_client_;

using namespace COMMON_NAMES;

static const double ROTATION_VELOCITY = 0.2;
static const double DRIVING_VELOCITY = 0.4;
static const double MAX_DETECT_DIST = 2.0;
static const double VOLATILE_DISTANCE_THRESHOLD = 0.005;
static const int FLIP_ROTATION_COUNT_MAX = 3;
static const int REPEAT_COUNT_MAX = 2;
static const float VOLATILE_SENSOR_TO_BASE_DISTANCE = 0.670;
static const int MAX_COUNT_FOR_OUT_OF_VOLATILE_RANGE = 100; // If the volaile depletes or due to some unforeseen cases,
                                                               // scout no longer finds the volatile, to keep it from getting
                                                               // stuck there, we are using this counter. 
                                                               // Will wait for 10 seconds

bool near_volatile_ = false;
bool new_message_received = false;
double volatile_distance_;
std::string robot_name_;

/**
 * @brief enum for rotation direction
 *        When clockwise, send direction as it is
          When Anticlockwise, flip the direction with -1
 * 
 */
enum DrivingDirection
{
  POSITIVE = 1,
  NEGATIVE = -1
};

enum DrivingMode
{
  ROTATE_ROBOT = 0,
  DRIVE_ROBOT_STRAIGHT
};

/**
 * @brief Rotate the robot in the given direction
 * 
 * @param rotate_direction   direction of rotation
 */
void rotateRobot(const DrivingDirection rotate_direction, const float rotational_velocity_multiplier)
{
  // ROS_INFO("Rotating the robot");
  operations::NavigationGoal goal;

  // Manual driving
  goal.drive_mode = NAV_TYPE::MANUAL;

  goal.forward_velocity = 0;
  goal.angular_velocity = rotate_direction * ROTATION_VELOCITY * rotational_velocity_multiplier;
  
  navigation_client_->sendGoal(goal);
  ros::Duration(0.1).sleep();
}

/**
 * @brief Stop the robot
 * 
 *
 */
void stopRobot()
{
  operations::NavigationGoal goal;

  // Manual driving
  goal.drive_mode = NAV_TYPE::MANUAL;

  goal.forward_velocity = 0;
  goal.angular_velocity = 0;

  navigation_client_->sendGoal(goal);
  navigation_client_->sendGoal(goal);
  navigation_client_->sendGoal(goal);
  navigation_client_->sendGoal(goal);
  ros::Duration(0.5).sleep();
}

void getOnTopOfVolatile()
{
  operations::NavigationGoal goal;

  goal.drive_mode = NAV_TYPE::MANUAL;
  goal.forward_velocity = 0.2;   
  goal.angular_velocity = 0;
  navigation_client_->sendGoal(goal);
  navigation_client_->sendGoal(goal);
  navigation_client_->sendGoal(goal);
  navigation_client_->sendGoal(goal);
  
  float starting_volatile_distance = volatile_distance_;

  float volatile_to_base_distance = (std::hypot(starting_volatile_distance, VOLATILE_SENSOR_TO_BASE_DISTANCE));
  while(ros::ok() && volatile_distance_ < volatile_to_base_distance)
    ros::Duration(0.1).sleep();

  stopRobot();
}

/**
 * @brief Rotate the robot to the absolute yaw 
 * 
 * @param orientation yaw of the robot 
 *										 ref frame is 'map'
 */
void driveRobotStraight(DrivingDirection rotate_direction, const float rotational_velocity_multiplier)
{
  // ROS_INFO("Driving the robot");
  operations::NavigationGoal goal;

  // Manual driving
  goal.drive_mode = NAV_TYPE::MANUAL;
  
  goal.forward_velocity = rotate_direction * DRIVING_VELOCITY * rotational_velocity_multiplier;
  goal.angular_velocity = 0;
  // ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Driving robot straight");

  navigation_client_->sendGoal(goal);
  ros::Duration(0.1).sleep();
}

/**
 * @brief Get the Best Pose object
 *        Currently only adjusts the orientation
 *        Should be generalised for position as well
 * 
 * @return geometry_msgs::Pose  Pose which minimises the volatile distance
 */
void getBestPose()
{
  DrivingDirection driving_direction = POSITIVE;
  DrivingMode driving_mode = ROTATE_ROBOT;
  bool rotate_robot = true;
  float flip_rotation_count = 0;
  int repeat_count = 0;

  double last_volatile_distance = MAX_DETECT_DIST + 1; // To make sure any detected
                                                       // distance is less than this
  double best_volatile_distance = last_volatile_distance;

  // Start rotating the robot to minimise distance
  // Sometimes the goal is skipped, and robot doesn't move. So sending consequitive orders.
  rotateRobot(driving_direction, 1.0);
  rotateRobot(driving_direction, 1.0);
  DrivingMode driving_execution;
  int volatile_unseen_counter = 0;

  while (rotate_robot && ros::ok())
  {
    if (new_message_received)
    {
      new_message_received = false;

      // If the distance is decreasing
      if ((volatile_distance_ - best_volatile_distance) < 0)
      {
        // if (volatile_distance_ < best_volatile_distance)
        // {
        ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Best distance updated from " << best_volatile_distance << " to " << volatile_distance_);
        best_volatile_distance = volatile_distance_;
        // }
      }

      // If the distance is increasing
      else if ((volatile_distance_ - best_volatile_distance) > VOLATILE_DISTANCE_THRESHOLD)
      {
        best_volatile_distance =  MAX_DETECT_DIST + 1;
        // ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Going far");
        if (flip_rotation_count < FLIP_ROTATION_COUNT_MAX)
        {
          ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Flipping Direction");
          driving_direction = (driving_direction == POSITIVE) ? NEGATIVE : POSITIVE;
          if (driving_mode == ROTATE_ROBOT)
            // rotateRobot(driving_direction, 1 / (flip_rotation_count + 1));
            driving_execution = ROTATE_ROBOT;
          else
            // driveRobotStraight(driving_direction, 1 / (flip_rotation_count + 1));
            driving_execution = DRIVE_ROBOT_STRAIGHT;
          flip_rotation_count++;
        }
        else
        {
          ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Flipped Enough");
          if (repeat_count < REPEAT_COUNT_MAX)
          {
            flip_rotation_count = 0;
            best_volatile_distance = MAX_DETECT_DIST + 1;
            driving_direction = POSITIVE;
            if (driving_mode == ROTATE_ROBOT)
            {
              ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Now linear optimisation");
              driving_mode = DRIVE_ROBOT_STRAIGHT;
              // driveRobotStraight(driving_direction, 1);
              driving_execution = DRIVE_ROBOT_STRAIGHT;
              repeat_count++;
            }
            else
            {
              ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Now Rotational optimisation");
              driving_mode = ROTATE_ROBOT;
              // rotateRobot(driving_direction, 1);
              driving_execution = ROTATE_ROBOT;
            }
          }
          else
          {
            ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "DONE EVERYTHING");
            rotate_robot = false;
            near_volatile_ = false;
            break;
          }
        }
      }
      
      if (volatile_distance_ == MAX_DETECT_DIST + 1)
        volatile_unseen_counter++;
      else
        volatile_unseen_counter = 0;
      
      if (driving_execution == ROTATE_ROBOT)
        rotateRobot(driving_direction, 1 / (flip_rotation_count + 1));
      else 
        driveRobotStraight(driving_direction, 1 / (flip_rotation_count + 1));
      
      last_volatile_distance = volatile_distance_;
    }
    else
    {
      ROS_WARN_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "No new message found");
      // TODO: What is new message is not received?
      // This case may not arise normally, but can arise during battery low situation
      // as volatile sensor stops working in battery low mode
    }
    if(volatile_unseen_counter > MAX_COUNT_FOR_OUT_OF_VOLATILE_RANGE)
      break;
    ros::Duration(0.1).sleep();
    // ROS_INFO_STREAM("Volatile Counter = "<<volatile_unseen_counter);
  }
  return;
}

/**
 * @brief Actionlib callback
 * 
 * @param localiser_goal 
 * @param server 
 */
void localiseResource(const operations::ResourceLocaliserGoalConstPtr &localiser_goal, ResourceLocaliserServer *server)
{
  ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Starting locating volatile sequence");
  if (near_volatile_)
  {
    stopRobot();
    getBestPose();

    ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Driving on top of volatile");

    operations::ResourceLocaliserResult res;
    if(volatile_distance_ != MAX_DETECT_DIST + 1)
    {
      getOnTopOfVolatile();
      res.result = COMMON_RESULT::SUCCESS;
    }
    else
    {
      ROS_ERROR_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Resource localiser finished ending up outside of volatile range");
      res.result = COMMON_RESULT::FAILED;
    }
    stopRobot();
    server->setSucceeded(res);
  }
  else
  {
    ROS_ERROR_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Resourse localisation called, but rover not near volatile");
    operations::ResourceLocaliserResult res;
    res.result = COMMON_RESULT::FAILED;
    server->setSucceeded(res);
  }
}

/**
 * @brief Callback for sensor topic
 * 
 * @param msg 
 */
void updateSensorData(const srcp2_msgs::VolSensorMsg::ConstPtr &msg)
{
  new_message_received = true;
  if (msg->distance_to == -1) // When there is no volatile nearby,
                              // Distance is returned as -1
  {
    near_volatile_ = false;
    volatile_distance_ = MAX_DETECT_DIST + 1; // To make sure any detected
                                              // distance is less than this
  }
  else
  {
    near_volatile_ = true;
    volatile_distance_ = msg->distance_to;
  }
}

int main(int argc, char **argv)
{
  // Ensure the robot name is passed in
  if (argc != 2 && argc != 4)
  {
    // Displaying an error message for correct usage of the script, and returning error.
    ROS_ERROR_STREAM("Not enough arguments! Please pass in robot name with number.");
    return -1;
  }
  else
  {
    // Robot Name from argument
    robot_name_ = std::string(argv[1]);
    std::string node_name = robot_name_ + "_resource_localiser_action_server";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ros::Subscriber subscriber = nh.subscribe("/" + robot_name_ + VOLATILE_SENSOR_TOPIC, 1000, updateSensorData);

    ResourceLocaliserServer resource_localiser_server(nh, RESOURCE_LOCALISER_ACTIONLIB, boost::bind(&localiseResource, _1, &resource_localiser_server), false);
    resource_localiser_server.start();

    ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Connecting to nav server...");

    navigation_client_ = new NavigationClient_(CAPRICORN_TOPIC + robot_name_ + "/" + NAVIGATION_ACTIONLIB, true);
    navigation_client_->waitForServer();

    ROS_INFO_STREAM("[OPERATIONS | resource_localiser.cpp | " << robot_name_ << "]: " << "Connected. Waiting for a localization request.");

    ros::spin();

    delete navigation_client_;

    return 0;
  }
}