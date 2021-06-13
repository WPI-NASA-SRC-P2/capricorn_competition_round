#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include "srcp2_msgs/SystemMonitorMsg.h"

class SolarChargingServer
{
private:
    bool solar_ok = false;
    float32 power_rate;
    power_saver: false;
    bool should_turn = false;
    bool is_turning = false;
    bool turnRobot()

    
public:
    void stop_robot(void);
    void rotate_robot(void);
    void solarChargeInitiate(void);
    void systemMonitorCB(nav_msgs::OccupancyGrid oGrid);
    void setPowerSaveMode(bool on);
    ros::Subscriber systemMonitor_subscriber;
    ros::ServiceClient powerMode_client; 
};