#include <operations/NavigationAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>
#include "operations/SolarCharge.h"



int main(int argc, char *argv[])
{
  //initialize node
  ros::init(argc, argv, "solar_charging_client");

  std::string robot_name(argv[1]);   

  //ROS Topic names
  std::string system_monitor_topic_ = "/capricorn/" + robot_name + "/system_monitor";

  //create a nodehandle
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<operations::SolarCharge>("solar_charge");

  operations::SolarChargeRequest req = true;
  operations::SolarChargeResponse res;
  client.call(req, res);

  // #TODO: not sure how best to exit solar charging mode
  // operations::SolarChargeRequest req = false;
  // operations::SolarChargeResponse res;
  // client.call(req, res);

  ros::Subscriber systemMonitor_subscriber = nh.subscribe(system_monitor_topic_, 1000, solarChargeInitiate_2);
  
  return 0;
}
