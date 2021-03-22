#ifndef NAVIGATION_ALGO_H
#define NAVIGATION_ALGO_H

#include <vector>
#include <math.h>
#include <geometry_msgs/Point.h>
// #include <capricorn_common/robot_description.h>

class NavigationAlgo
{
private:
  static constexpr float arc_spiral_a = 0;   // Inner radius (starting radius of the spiral)
  static constexpr float arc_spiral_b = 2.7; // Incerement per rev
  static constexpr float arc_spiral_multi = 2;
  static constexpr float arc_spiral_incr = 3.9;

public:
  NavigationAlgo(/* args */);
  ~NavigationAlgo();

  static constexpr float wheel_sep_width_ = 1.0;
  static constexpr float wheel_sep_length_ = 1.076;

  /**
   * @brief **DEPRICATED** Get the Steering Angles for Making Radial Turn 
   *        Use override with geometry_msgs::Point instead
   * 
   * @param radius    Radius follows right hand rule from top
   *                  i.e. radius to the left is +ve
   *                                     right is -ve
   * 
   * @return std::vector<float>    output steering angles
   *                  The vector will be in order:
   *                  Clockwise from top, starting with FRONT_LEFT
   * 
   *          element 0: Front Left Wheel
   *          element 1: Front Right Wheel
   *          element 2: Back Right Wheel
   *          element 3: Back Left Wheel
   */
  static std::vector<float> getSteeringAnglesRadialTurn(const float radius);

  /**
   * @brief Get the Steering Angles for Making Radial Turn 
   *        Use override with geometry_msgs::Point instead
   * 
   * @param center_of_rotation    The center of rotation from the center of the robot.                                
   *                              The Center of Rotation follows right hand rule from top
   *                              i.e. Positive X is to the front of the robot 
   *                                   Positive Y is to the left of the robot
   * 
   * @return std::vector<float>   output steering angles
   *                              The vector will be in order:
   *                              Clockwise from top, starting with FRONT_LEFT
   *             
   *                      element 0: Front Left Wheel
   *                      element 1: Front Right Wheel
   *                      element 2: Back Right Wheel
   *                      element 3: Back Left Wheel
   */
  static std::vector<float> getSteeringAnglesRadialTurn(const geometry_msgs::Point center_of_rotation);

  /**
   * @brief Get the Driving Velocities for Making Radial Turn 
   * 
   * @param center_of_rotation    The center of rotation from the center of the robot.                                
   *                              The Center of Rotation follows right hand rule from top
   *                              i.e. Positive X is to the front of the robot 
   *                                   Positive Y is to the left of the robot
   * 
   * @param velocity              Desired Velocity at the center of the robot
   * 
   * @return std::vector<float>   Output steering angles
   *                              The vector will be in order:
   *                              Clockwise from top, starting with FRONT_LEFT
   * 
   *                      element 0: Front Left Wheel
   *                      element 1: Front Right Wheel
   *                      element 2: Back Right Wheel
   *                      element 3: Back Left Wheel
   */
  static std::vector<float> getDrivingVelocitiessRadialTurn(const geometry_msgs::Point center_of_rotation, const float velocity);

  /**
   * @brief **DEPRICATED** Get the Driving Efforts for Making Radial Turn 
   * 
   * @param radius    Radius follows right hand rule from top
   *                  i.e. radius to the left is +ve
   *                                     right is -ve
   * 
   * @param effort    Effort at the center of the robot
   * 
   * @return std::vector<float>    output steering angles
   *                  The vector will be in order:
   *                  Clockwise from top, starting with FRONT_LEFT
   * 
   *          element 0: Front Left Wheel
   *          element 1: Front Right Wheel
   *          element 2: Back Right Wheel
   *          element 3: Back Left Wheel
   */
  static std::vector<float> getDrivingVelocitiessRadialTurn(const float radius, const float effort);

  /**
   * @brief Get the radius of Osculating circle for Archimedean Spiral
   * 
   * @param time      How further are we into the spiral
   * 
   * @return float   outputs the radius at Arcimedean Spiral
   */
  static float getRadiusInArchimedeanSpiral(const float time);

  /**
   * @brief Retruns the N points lying in spiral path
   * 
   * @param init_location   Initial location of the robot in an environment
   * @param N               Number of points desired
   * @param init_theta      Pick up where left off
   * @return std::vector<geometry_msgs::Point> 
   */
  static std::vector<geometry_msgs::Point> getNArchimedeasSpiralPoints(const geometry_msgs::Point &init_location, const int N, int init_theta = 0);

  /**
   * @brief Get the Kinetic Energy object
   * 
   * @param vel           Velociry of the robot
   * @param robot_mass    Mass of the robot
   * @return float 
   */
  inline static float getKineticEnergy(const float vel, const float robot_mass)
  {
    return 0.5 * (robot_mass) * (std::pow(vel, 2));
  }

};
#endif