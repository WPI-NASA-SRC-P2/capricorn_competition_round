// header guards
// https://www.educative.io/edpresso/what-are--sharpifndef-and--sharpdefine-used-for-in-cpp



#ifndef PLANNER_CLASS_H
#define PLANNER_CLASS_H

#include <ros/ros.h>
#include <set>
#include <algorithm>
#include <tuple>
#include <iostream>
#include <vector>


// Class names must be CamelCased and has a capital letter for each new word
// http://wiki.ros.org/CppStyleGuide
class pathPlanner {

/**
 * @brief  Declare the private variables, functions and methods here.
 *         Variables, functions and methods here cannot be accessed from outside the class
 * 
 */
private:

	// Private variable
	// Variable names must be 


/**
 * @brief Declare the public variables, functions and methods here.
 *        Things here can be accessed from outside the class.
 * 
 */
public:
	



	

	static std::vector<geometry_msgs::Point> findAllNeighbors(std::vector <geometry_msgs::Point>& grid, int width, std::vector <geometry_msgs::Point> toSearch, int r);

	static std::vector <int> cSpace(std::vector <geometry_msgs::Point> grid, int width, std::vector <int> probibility, int threshold, int radius);



};

#endif // Planner_CLASS_H