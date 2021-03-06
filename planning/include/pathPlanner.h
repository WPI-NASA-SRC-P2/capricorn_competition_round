// header guards
// https://www.educative.io/edpresso/what-are--sharpifndef-and--sharpdefine-used-for-in-cpp
#ifndef TEMPLATE_CLASS_H
#define TEMPLATE_CLASS_H

#include <ros/ros.h>
#include <iostream>

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
   * @brief Constructor
   * 
   */
    pathPlanner();

    /**
    * @brief Destroy the Template Class object
    *        Pointers, threads etc used in the class must be delted in the destructor
    * 
    */
    ~pathPlanner();

    // Functions and Methods should be camelCased with first letter lower-cased
    // Getters and Setters for interacting with the private variables/attributes
    // https://www.w3schools.com/cpp/cpp_encapsulation.asp

    /**
   * @brief Calculate the c-space from the occupancy grid
   * 
   * @return void  - publishes the updated occupancy grid to a "cspace" topic
   */
    void cSpace();

};

#endif // TEMPLATE_CLASS_H