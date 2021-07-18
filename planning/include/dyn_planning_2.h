#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
#include <math.h>
#include <utils/common_names.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>




class DynamicPlanning2
{
private:

    
public:

 
/**
 * @brief 
 * 
 * @param wp1 
 * @param wp2 
 * @return unordered_map<char, float> 
 */
 static std::unordered_map<std::string, float> LineEquation(geometry_msgs::Point wp1, geometry_msgs::Point wp2);


/**
 * @brief 
 * 
 * @param centroid 
 * @param radius 
 * @return std::unordered_map<std::string, float> 
 */
 static std::unordered_map<std::string, float> CircleEquation(geometry_msgs::Point centroid, float radius);


/**
 * @brief 
 * 
 * @param lineParameters 
 * @param radius 
 * @return float 
 */
static float mIntersect(std::unordered_map<std::string, float> lineParameters, float radius);

 
 /**
  * @brief 
  * 
  * @param obstacleCentroids 
  * @return float 
  */
 static float ObstacleRadius(float width);
 
 /**
  * @brief 
  * 
  * @param parameters 
  * @param centroidOfObstacle 
  * @return float 
  */
 static float PerpendicularDistance(std::unordered_map<std::string, float> parameters, geometry_msgs::Point centroidOfObstacle);

/**
 * @brief 
 * 
 * @param path 
 * @return nav_msgs::Path 
 */
static nav_msgs::Path getPathInMapFrame(nav_msgs::Path path);

/**
 * @brief 
 * 
 * @param pose 
 * @param frame 
 * @param tf_buffer 
 * @param duration 
 * @param tries 
 * @return true 
 * @return false 
 */
static bool transformPose(geometry_msgs::PoseStamped& pose, const std::string& frame, const tf2_ros::Buffer& tf_buffer, float duration = 0.1, int tries = 10);

/**
 * @brief 
 * 
 * @param centroid 
 * @param mi 
 * @param lineParameters 
 * @return std::vector<std::pair<float, float>> 
 */
static std::vector<float> PointsofIntersection(geometry_msgs::Point centroid, float mi, std::unordered_map<std::string, float>  lineParameters, float radius);

static std::pair<float,float> solveQuadraticEquation(float a, float b, float c);

/**
 * @brief 
 * 
 * @param lineParameters 
 * @param iPoints 
 * @return true 
 * @return false 
 */
static bool CheckIfBewtweenWaypoints(std::unordered_map<std::string, float> lineParameters, std::vector<float> iPoints);

/**
 * @brief 
 * 
 * @param path 
 */
static void ReversePath(nav_msgs::Path& path);


/**
 * @brief 
 * 
 * @param obstacles 
 * @param path 
 * @return true 
 * @return false 
 */
static bool checkAllObstacles(perception::ObjectArray obstacles, nav_msgs::Path path, std::string robot_name, const tf2_ros::Buffer& tf_buffer);


/**
 * @brief 
 * 
 * @param obstacles 
 * @param path 
 * @param robot_name 
 * @param tf_buffer 
 * @return true 
 * @return false 
 */
static bool checkAllObstacles2(perception::ObjectArray obstacles, nav_msgs::Path path, std::string robot_name, const tf2_ros::Buffer& tf_buffer);


};
