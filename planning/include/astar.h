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
  static geometry_msgs::Point getPoint(double x, double y);
  static double distance(int ind1, int ind2, int width);
  static std::array<int, 8> getNeighborsIndiciesArray(int pt, int width, int size);
  static bool collinear(int pt1, int pt2, int pt3, int width);
  static geometry_msgs::PoseStamped poseStampedFromIndex(int ind, nav_msgs::OccupancyGrid &oGrid);
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

  static nav_msgs::Path findPathOccGrid(nav_msgs::OccupancyGrid oGrid, geometry_msgs::Point target, geometry_msgs::Point start);
};