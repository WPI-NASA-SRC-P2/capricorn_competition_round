#include <ros/ros.h>
#include <iostream>

class TrajectoryMethods
{
public:
  /**
     * @brief Assigns the service response based on the output
     * of trajectory generation function 
     * @param req request section of a service message
     * @param res response section of a service message
     * @return true after successfully responding to a client request
     */
  static bool trajectoryGeneration(planning::trajectory::Request &req, planning::trajectory::Response &res);
};