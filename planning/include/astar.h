#pragma once

#include <vector>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <set>
#include <unordered_map>


class AStar
{
private:
  /**
   * @brief Get the Point object
   * 
   * @param x 
   * @param y 
   * @return geometry_msgs::Point 
   */
  static geometry_msgs::Point getPoint(double x, double y);

  /**
   * @brief 
   * 
   * @param ind1 
   * @param ind2 
   * @param width 
   * @return double 
   */
  static double distance(int ind1, int ind2, int width);

  /**
   * @brief Get the Neighbors Indicies Array object
   * 
   * @param pt 
   * @param width 
   * @param size 
   * @return std::array<int, 8> 
   */
  static std::array<int, 8> getNeighborsIndiciesArray(int pt, int width, int size);
  
  /**
   * @brief 
   * 
   * @param pt1 
   * @param pt2 
   * @param pt3 
   * @param width 
   * @return true 
   * @return false 
   */
  static bool collinear(int pt1, int pt2, int pt3, int width);

  /**
   * @brief 
   * 
   * @param ind 
   * @param oGrid 
   * @return geometry_msgs::PoseStamped 
   */
  static geometry_msgs::PoseStamped poseStampedFromIndex(int ind, nav_msgs::OccupancyGrid &oGrid);

  /**
   * @brief 
   * 
   * @param current 
   * @param last 
   * @param reverse_list 
   * @param oGrid 
   * @return nav_msgs::Path 
   */
  static nav_msgs::Path reconstructPath(int current, int last, std::unordered_map<int, int> &reverse_list, nav_msgs::OccupancyGrid &oGrid);


public:
  /**
     * @brief Calculated the shortest path using an occupancy grid based approach.
     * 
     * @param oGrid The occupancy grid, ranging 0 (unoccupied) to 100 (completely blocked)
     * @param target The target Point for the algorithm.
     * @param start The start point of the algorithm (usually the robot position)
     *
     * @return A ROS Path message containing the points in the shortest path.
    **/

  static nav_msgs::Path findPathOccGrid(nav_msgs::OccupancyGrid &oGrid, geometry_msgs::Point target, geometry_msgs::Point start);
};