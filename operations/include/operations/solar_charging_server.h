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

  //Indicates whether the robot is oriented correctly to charge
  bool solar_ok = false;  
  
  //Positive rate for charging, negative rate for discharging
  float_t power_rate;

  //Indicates whether the robot is in power saver mode. Power save mode reduced the power consumption of the robot while idle. 
  bool power_saver = false;
  
  //True if the robot has been given a command to enter solar charge mode and is not oriented correctly to charge. 
  bool should_turn = false;

  // //True if the robot has begun to turn
  // bool is_turning = false;

    /**
     * @brief roates the chassis 
     *        Is a call to the naviagtion actlib
     *        doesn't stop until another action/goal is sent to the naviagtion client
     * 
     */
    void rotateRobot(void);

    /**
     * @brief stops the chassis from rotating
     *        Is a call to the navigation actlib
     * 
     */
    void stopRobot(void);

    /**
     * @brief Set the Power Save Mode to on or off
     * 
     * @param state on or off
     */
    void setPowerSaveMode(bool state);
    
public:
    
    /**
     * @brief based on the request, either Acitivates Solar_Charge_Mode, or diactives it
     *        ALWAYS cancel solar recharge before beginning any other action
     * 
     * @param request solarChargeRequest 
     * @param reponse solarChargeResponse
     */
    bool solarChargeInitiate(operations::SolarCharge::Request& request, operations::SolarCharge::Response& response);

    /**
     * @brief stops the robot when solar_charge_mode is on and the robot is in desired position
     * 
     * @param msg system_monitor topic message
     */
    void systemMonitorCB(const srcp2_msgs::SystemMonitorMsg &msg);
    
    //subscriber that listens to the System Monitor topic 
    //used to get POwer Save Mode status
    ros::Subscriber systemMonitor_subscriber;

    //sends a request to turn power saver mode on and off
    // can see effect in rostopic system_monitor
    ros::ServiceClient powerMode_client; 

};