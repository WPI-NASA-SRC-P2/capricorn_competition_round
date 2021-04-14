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
   * @brief Create a geometry_msgs::Point object
   * 
   * @param x X component
   * @param y Y component
   * @return A geometry_msgs::Point object
   */
  static geometry_msgs::Point getPoint(double x, double y);

  /**
   * @brief Get euclidean distance between two indexes on a grid
   * 
   * @param ind1 First point index
   * @param ind2 Second point index
   * @param width The width of the map that the indexes are on
   * @return The euclidean distance between the points (in map units)
   */
  static double distance(int ind1, int ind2, int width);

  /**
   * @brief Return the 8 surronding points on an index on a grid
   * 
   * @param pt The index of the point to get the neighbors of
   * @param width The width of the grid which the point is on
   * @param size The maximum point index, or the size of the 1D representation of the grid
   * @return An array of the index of the neighbors of the point, -1 if that neighbor is out of bounds of the grid.
   */
  static std::array<int, 8> getNeighborsIndiciesArray(int pt, int width, int size);

  /**
   * @brief Checks if the three points are collinear
   * 
   * @param pt1 The index of the first point
   * @param pt2 The index of the second point
   * @param pt3 The index of the third point
   * @param width The width of the grid that the points are on.
   * @return Whether the three points are collinear.
   */
  static bool collinear(int pt1, int pt2, int pt3, int width);

  /**
   * @brief Converts a 1D index on a grid to a 2D point
   * 
   * @param ind The index of the point
   * @param oGrid The occupancy grid that the point lies in
   * @return A 2d point conversion of the given index
   */
  static geometry_msgs::PoseStamped poseStampedFromIndex(int ind, nav_msgs::OccupancyGrid const &oGrid);

  /**
   * @brief Reconstructs the path from map of least costly nodes
   * @param current The node to start the reverse list of
   * @param last The target node
   * @param reverse_list The list of nodes and associated closest nodes
   * @param oGrid The occupancy grid (only for header and other metadata)
   * @return A Path message of the shortest path, including the current robot location and the target location.
   */
  static nav_msgs::Path reconstructPath(int current, int last, std::unordered_map<int, int> &reverse_list, const nav_msgs::OccupancyGrid &oGrid);

public:
  /**
     * @brief Calculated the shortest path using an occupancy grid based approach.
     * 
     * @param oGrid The occupancy grid, ranging 0 (unoccupied) to 100 (completely blocked)
     * @param target The target Point for the algorithm.
     * @param start The start point of the algorithm (usually the robot position
     * @param threshold The threshold above which we consider a node occupied. default = 50
     *
     * @return A ROS Path message containing the points in the shortest path, including the robot's current location.
    **/

  static nav_msgs::Path findPathOccGrid(const nav_msgs::OccupancyGrid &oGrid, geometry_msgs::Point target, geometry_msgs::Point start, int threshold = 50);
};