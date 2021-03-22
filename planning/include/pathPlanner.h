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
class pathPlanner
{

/**
 * @brief  Declare the private variables, functions and methods here.
 *         Variables, functions and methods here cannot be accessed from outside the class
 * 
 */
private:

  // Private variable
  // Variable names must be 
  std::string team_name;


/**
 * @brief Declare the public variables, functions and methods here.
 *        Things here can be accessed from outside the class.
 * 
 */
public:
  /**
   * @brief Construct a new pathPlanner Class object
   * 
   */
  pathPlanner();

  /**
   * @brief Destroy the pathPlanner Class object
   *        Pointers, threads etc used in the class must be delted in the destructor
   * 
   */
  ~pathPlanner();

  struct cord 
  {
    int x;
    int y;

    cord& operator=(const cord& a)
    {
      x = a.x;
      y = a.x;
      return *this;
    }

    bool operator==(const cord& a) const 
    {
      return (x == a.x && y == a.y);
    }

    
  };
  

  

  std::vector<pathPlanner::cord> findAllNeighbors(std::vector <pathPlanner::cord> grid, int width, std::vector <pathPlanner::cord> toSearch, int r);

  int grid_to_index(std::vector <pathPlanner::cord>, pathPlanner::cord);
  pathPlanner::cord index_to_grid(std::vector <pathPlanner::cord>, int);
  std::vector <pathPlanner::cord> eightNeighbors(std::vector <pathPlanner::cord>, pathPlanner::cord, int width);

  // Functions and Methods should be camelCased with first letter lower-cased 
  // Getters and Setters for interacting with the private variables/attributes
  // https://www.w3schools.com/cpp/cpp_encapsulation.asp
  
  /**
   * @brief Get the Team Name from the class object
   * 
   * @return std::string returns the team name
   */
  std::string getTeamName();

  /**
   * @brief Set the team name
   * 
   * @param string Input which will set the Team Name
   *               Notice that the variable is passed by reference and is const.
   *               https://www.learncpp.com/cpp-tutorial/passing-arguments-by-reference/
   */
  void setTeamName(const std::string& input_string);

  std::vector <int> cSpace(std::vector <pathPlanner::cord> grid, int width, std::vector <int> probibility, int threshold, int radius);



};

#endif // Planner_CLASS_H