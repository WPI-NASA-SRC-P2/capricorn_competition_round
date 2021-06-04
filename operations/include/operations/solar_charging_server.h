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
public:
    void stop_robot(void);
    void rotate_robot(void);
    void solarChargeInitiate(void);
    void systemMonitorCB(nav_msgs::OccupancyGrid oGrid);
    ros::Subscriber systemMonitor_subscriber;
};