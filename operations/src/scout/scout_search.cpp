/*
Author BY: Ashay Aswale, Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

Command Line Arguments Required:
1. robot_name: eg. small_scout_1, small_excavator_2
*/
#include <mutex>
#include <operations/NavigationAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <operations/obstacle_avoidance.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <operations/navigation_algorithm.h>
#include <operations/Spiral.h>

#define UPDATE_HZ 10

typedef actionlib::SimpleActionClient<operations::NavigationAction> Client;

Client *g_client;

operations::NavigationGoal g_nav_goal;
perception::ObjectArray g_objects;

const float INIT_VALUE = -100.00, FORWARD_VELOCITY = 0.8;
std::mutex g_objects_mutex;
int g_lost_detection_times = 0, g_true_detection_times = 0, g_revolve_direction = -1;

geometry_msgs::PoseStamped g_robot_pose;
bool cb_init = false; // Used to check if the callback has been initiated yet or not
static std::vector<geometry_msgs::PointStamped> g_spiral_points;

double g_last_dist = 0.0;
bool g_going_to_goal = false;
bool g_new_trajectory = false;
bool resume_spiral = false;
bool new_stop_call = false;
const double CHECKPOINT_THRESHOLD = 2.0;
bool was_driving = false;

/**
 * @brief Callback to the robot pose topic
 * 
 * @param msg 
 */
void updateRobotPose(const nav_msgs::Odometry::ConstPtr &msg)
{
  g_robot_pose.header = msg->header;
  g_robot_pose.pose = msg->pose.pose;
  cb_init = true;
}

/**
 * @brief Get the Orientation object
 *        The tangential direction of a point is 
 *        approximated by the segment joining the previous and the next point
 * 
 * @param p0  Point previous to the current point of interest
 * @param p2  Point next to the current point of interest
 * @return geometry_msgs::Quaternion orientation tangential to the spiral
 */
geometry_msgs::Quaternion getOrientation(const geometry_msgs::Point &p0, const geometry_msgs::Point &p2)
{
  float d_x = p2.x - p0.x;
  float d_y = p2.y - p0.y;

  double yaw = atan2(d_y, d_x);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, yaw);

  geometry_msgs::Quaternion quat_msg;
  tf2::convert(myQuaternion, quat_msg);
  return quat_msg;
}

/**
 * @brief Get the Center Of Rotation object
 *        Returns the center of the circle formed by the first three points in spiral
 * 
 * @param spiral_points   Points forming the spiral curvature
 * @return geometry_msgs::PointStamped  Center of the circle of first three points
 */
geometry_msgs::PointStamped getCenterOfRotation(const std::vector<geometry_msgs::PointStamped> &spiral_points)
{
  std::vector<geometry_msgs::PointStamped> temp_points;
  temp_points.push_back(spiral_points.at(0));
  temp_points.push_back(spiral_points.at(1));
  temp_points.push_back(spiral_points.at(2));

  double radius = NavigationAlgo::getRadiusOfThreePointsCircle(temp_points);
  geometry_msgs::PointStamped center_of_rot;
  center_of_rot.point.y = radius;
  return center_of_rot;
}

/**
 * @brief Callback function which subscriber to Objects message published from object detection
 * 
 * @param objs 
 */
void objectsCallback(const perception::ObjectArray &objs)
{
  const std::lock_guard<std::mutex> lock(g_objects_mutex);
  g_objects = objs;
}

/**
 * @brief Actual algorithm for driving the spiral motion
 * 
 */
void driveSprial()
{
  if (g_spiral_points.size() >= 3)
  {
    double dist = NavigationAlgo::changeInPosition(g_robot_pose, g_spiral_points.at(1));
    bool done_driving = g_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    ROS_INFO("Get Ready");
    if (g_going_to_goal && !done_driving)
    {
      ROS_INFO("I think I'm going to goal");
      g_last_dist = dist;
      ros::Duration(0.1).sleep();
      g_going_to_goal = true;
      return;
    }
    else
      g_going_to_goal = false;

    if (dist < CHECKPOINT_THRESHOLD)
    {
      geometry_msgs::PointStamped center_of_rot = getCenterOfRotation(g_spiral_points);
      g_spiral_points.erase(g_spiral_points.begin());
      g_nav_goal.drive_mode = NAV_TYPE::REVOLVE;
      g_nav_goal.point = center_of_rot;
      g_nav_goal.forward_velocity = 2.0;
      ROS_INFO_STREAM("Circular Motion radius: " << center_of_rot.point.y << "\tdist: " << dist);
      g_last_dist = 100; // Just to make it big for the next iteration
      g_new_trajectory = true;
      // g_going_to_goal = true;
      was_driving = false;
    }
    else if (dist > g_last_dist || was_driving)
    {
      geometry_msgs::Point point_0, point_2;
      point_0 = g_spiral_points.at(0).point;
      point_2 = g_spiral_points.at(2).point;

      ROS_INFO_STREAM("Going to goal dist:" << dist);

      g_nav_goal.drive_mode = NAV_TYPE::GOAL;
      g_nav_goal.pose.pose.position = g_spiral_points.at(1).point;
      g_nav_goal.pose.pose.orientation = getOrientation(point_0, point_2);
      g_nav_goal.pose.header.frame_id = MAP;
      // g_client->sendGoal(g_nav_goal);
      g_going_to_goal = true;
      g_new_trajectory = true;
      was_driving = true;
    }
    else
    {
      ROS_INFO_STREAM("Continuing spiral" << dist << g_last_dist);
      g_last_dist = dist;
      g_new_trajectory = false;
    }
  }
}

/**
 * @brief Function for navigating a robot near to an object detection based class
 * 
 */
void spiralSearch()
{
  const std::lock_guard<std::mutex> lock(g_objects_mutex);
  perception::ObjectArray objects = g_objects;

  std::vector<perception::Object> obstacles;

  for (int i = 0; i < objects.number_of_objects; i++)
    obstacles.push_back(objects.obj.at(i));

  float direction = checkObstacle(obstacles);

  if (abs(direction) > 0.0)
  {
    g_nav_goal.drive_mode = NAV_TYPE::MANUAL;
    g_nav_goal.forward_velocity = FORWARD_VELOCITY;
    g_nav_goal.direction = direction;
    g_nav_goal.angular_velocity = 0;
    g_new_trajectory = true;
    // g_going_to_goal = true;
    ROS_INFO("Avoiding Obstacle");
    return;
  }
  else
  {
    driveSprial();
  }
}

/**
 * @brief Function which gets at the start
 */
void execute()
{
  ros::Rate update_rate(UPDATE_HZ);

  while (ros::ok())
  {
    if (resume_spiral)
    {
      spiralSearch();
      if (g_new_trajectory)
      {
        g_client->sendGoal(g_nav_goal);
        g_new_trajectory = false;
      }
    }
    else if (new_stop_call)
    {
      g_nav_goal.drive_mode = NAV_TYPE::MANUAL;
      g_nav_goal.forward_velocity = 0;
      g_nav_goal.direction = 0;
      g_nav_goal.angular_velocity = 0;
      g_client->sendGoal(g_nav_goal);
      new_stop_call = false;
      g_going_to_goal = false;
    }

    update_rate.sleep();
    ros::spinOnce();
  }
}

bool serviceCB(operations::Spiral::Request &req,
               operations::Spiral::Response &res)
{
  resume_spiral = req.resume_spiral_motion;

  // Whever interrupted, it assumes that this location has volatiles,
  // and robots will park. So it should continue from the next point after that.
  if (!resume_spiral)
  {
    g_spiral_points.erase(g_spiral_points.begin());
    new_stop_call = true;
  }
  return true;
}

int main(int argc, char **argv)
{
  if (argc != 2 && argc != 4)
  {
    ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");
    return -1;
  }

  std::string robot_name(argv[1]);
  ros::init(argc, argv, robot_name + COMMON_NAMES::SCOUT_SEARCH_NODE_NAME);
  ros::NodeHandle nh;

  bool odom_flag;
	nh.getParam("cheat_odom", odom_flag);

  ros::Subscriber odom_sub;

	if (odom_flag)
	{
		odom_sub = nh.subscribe(CAPRICORN_TOPIC + robot_name + CHEAT_ODOM_TOPIC, 1000, updateRobotPose);
		ROS_INFO("Currently using cheat odom from Gazebo\n");
	}
	else
	{
		odom_sub = nh.subscribe("/" + robot_name + RTAB_ODOM_TOPIC, 1000, updateRobotPose);
		ROS_INFO("Currently using odom from rtabmap\n");
	}

  g_client = new Client(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + "/" + COMMON_NAMES::NAVIGATION_ACTIONLIB, true);
  g_client->waitForServer();
  g_nav_goal.drive_mode = COMMON_NAMES::NAV_TYPE::MANUAL;

  geometry_msgs::PointStamped zero_point;
  zero_point.header.frame_id = MAP;
  g_spiral_points = NavigationAlgo::getNArchimedeasSpiralPoints(zero_point, 400, 17);

  while (ros::ok() && !cb_init)
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objectsCallback);

  ros::ServiceServer service = nh.advertiseService(COMMON_NAMES::SCOUT_SEARCH_SERVICE, serviceCB);
  ROS_INFO_STREAM("Starting Searching - " << robot_name);
  execute();
  return 0;
}