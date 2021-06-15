#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include "srcp2_msgs/SystemMonitorMsg.h"



/**
 * @brief enum for rotation direction
 *        When clockwise, send direction as it is
          When Anticlockwise, flip the direction with -1
 * 
 */
enum DrivingDirection
{
  POSITIVE = 1,
  NEGATIVE = -1
};

class SolarChargingServer
{
private:
    bool solar_ok = false;
    float_t power_rate;
    bool power_saver = false;
    bool should_turn = false;
    bool is_turning = false;
    bool turnRobot();
    void rotateRobot(const DrivingDirection rotate_direction, const float rotational_velocity_multiplier);
    
    
public:
    void stopRobot(void);
    void solarChargeInitiate(operations::SolarChargeRequest &request, operations::SolarChargeResponse &response);
    void systemMonitorCB(srcp2_msgs::SystemMonitorMsg msg);
    void setPowerSaveMode(bool on);
    ros::Subscriber systemMonitor_subscriber;
    ros::ServiceClient powerMode_client; 
};