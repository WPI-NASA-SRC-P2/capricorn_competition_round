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
NavigationClient_* navigation_client_;


using namespace COMMON_NAMES;


double ROTATION_VELOCITY = 0.2;
double MAX_DETECT_DIST = 2.0;
double VOLATILE_DISTANCE_THRESHOLD = 0.02;
int FLIP_ROTATION_COUNT_MAX = 2;
bool near_volatile_ = false;
bool new_message_received = false;
double volatile_distance_;

geometry_msgs::PoseStamped robot_pose_;


/**
 * @brief enum for rotation direction
 *        When clockwise, send direction as it is
          When Anticlockwise, flip the direction with -1
 * 
 */
enum RotationDirection
{
  CLOCKWISE = 1,
  COUNTERCLOCKWISE = -1
};


/**
 * @brief Rotate the robot in the given direction
 * 
 * @param rotate_direction   direction of rotation
 */
void rotateRobot(const RotationDirection rotate_direction)
{
	operations::NavigationGoal goal;
	
	// Manual driving
	goal.drive_mode = NAV_TYPE::MANUAL;
	
	goal.forward_velocity = 0;
	goal.angular_velocity = rotate_direction * ROTATION_VELOCITY;
  
	navigation_client_->sendGoal(goal);
	ros::Duration(0.5).sleep();
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
	// ros::Duration(0.5).sleep();
}

/**
 * @brief Rotate the robot to the absolute yaw 
 * 
 * @param orientation yaw of the robot 
 *										 ref frame is 'map'
 */
void navigateRobot(const geometry_msgs::Pose target_pose)
{
	operations::NavigationGoal goal;
	
	// Auto driving
	goal.drive_mode = NAV_TYPE::GOAL;
	
	goal.pose.header.frame_id = MAP;
	goal.pose.pose = target_pose;

	navigation_client_->sendGoal(goal);
	// ros::Duration(0.1).sleep();
}

/**
 * @brief Get the Best Pose object
 *        Currently only adjusts the orientation
 *        Should be generalised for position as well
 * 
 * @return geometry_msgs::Pose  Pose which minimises the volatile distance
 */
geometry_msgs::Pose getBestPose()
{
  RotationDirection rotate_direction = CLOCKWISE;
  bool rotate_robot = true;
  int flip_rotation_count = 0;

  double last_volatile_distance = MAX_DETECT_DIST + 1;	// To make sure any detected 
                                                        // distance is less than this
  double best_volatile_distance = last_volatile_distance;
  
  geometry_msgs::Pose current_robot_pose = robot_pose_.pose;
  geometry_msgs::Pose best_robot_pose = current_robot_pose;

  // Start rotating the robot to minimise distance
  rotateRobot(rotate_direction);

  while (rotate_robot && ros::ok())
  {
    if(new_message_received)
    {
      new_message_received = false;
      current_robot_pose = (robot_pose_.pose);

      // If the distance is decreasing
      if ((last_volatile_distance - volatile_distance_)>VOLATILE_DISTANCE_THRESHOLD)
      {
        if (volatile_distance_ < best_volatile_distance)
        {
          best_volatile_distance = volatile_distance_;
          ROS_INFO("Best distance updated");
          best_robot_pose = current_robot_pose;
        }
      }

      // If the distance is increasing
      else if ((last_volatile_distance - volatile_distance_)<-VOLATILE_DISTANCE_THRESHOLD
                || !near_volatile_)
      {
        if(flip_rotation_count < FLIP_ROTATION_COUNT_MAX)
        {
          ROS_INFO("Flipping Direction");
          rotate_direction = (rotate_direction == CLOCKWISE) ? COUNTERCLOCKWISE : CLOCKWISE;
          rotateRobot(rotate_direction);
          flip_rotation_count++;
        }
        else
        {
          ROS_INFO("Flipped Enough");
          rotate_robot = false;
          near_volatile_ = false;
          break;
        }
      }
      
      last_volatile_distance = volatile_distance_;
    }
  }
  return best_robot_pose;
}

/**
 * @brief Actionlib callback
 * 
 * @param localiser_goal 
 * @param server 
 */
void localiseResource(const operations::ResourceLocaliserGoalConstPtr& localiser_goal, ResourceLocaliserServer* server)
{
	ROS_INFO("Starting locating volatile sequence");
	if (near_volatile_)
	{
    geometry_msgs::Pose best_robot_pose = getBestPose();
		
    ROS_INFO("Setting to best");
		navigateRobot(best_robot_pose);		
	  server->setSucceeded();
	}
  else
  {
    ROS_ERROR("Resourse localisation called, but rover not near volatile");
    server->setAborted();
  }
}


/**
 * @brief Callback for sensor topic
 * 
 * @param msg 
 */
void updateSensorData(const srcp2_msgs::VolSensorMsg::ConstPtr& msg)
{
	new_message_received = true;
	if(msg->distance_to == -1)  // When there is no volatile nearby, 
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

/**
 * @brief Subscribes to an odometry topic, and updates the global robot_pose_
 * 
 * @param msg The odometry message to process
 */
void updateRobotPose(const nav_msgs::Odometry::ConstPtr &msg)
{
  // guard is bad?
	robot_pose_.header = msg->header;
	robot_pose_.pose = msg->pose.pose;
}

int main(int argc, char** argv)
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
    std::string robot_name(argv[1]);
    std::string node_name = robot_name + "_resource_localiser_action_server";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    
    ros::Subscriber subscriber = nh.subscribe("/" + robot_name + VOLATILE_SENSOR_TOPIC, 1000, updateSensorData);
    ros::Subscriber update_current_robot_pose_ = nh.subscribe(CAPRICORN_TOPIC + robot_name + CHEAT_ODOM_TOPIC, 1000, updateRobotPose);

    ResourceLocaliserServer resource_localiser_server(nh, RESOURCE_LOCALISER_ACTIONLIB, boost::bind(&localiseResource, _1, &resource_localiser_server), false);
    resource_localiser_server.start();

    navigation_client_ = new NavigationClient_(NAVIGATION_ACTIONLIB, true);
    
    ros::spin();
    
    delete navigation_client_;
    
    return 0;
  }
}