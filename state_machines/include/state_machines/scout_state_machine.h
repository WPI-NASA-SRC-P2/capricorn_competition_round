// #ifndef SCOUT_STATE_MACHINE_H
// #define SCOUT_STATE_MACHINE_H

#include <iostream>
#include <operations/NavigationAction.h>
#include <operations/NavigationVisionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <utils/common_names.h>
#include <state_machines/ScoutStateMachineTaskAction.h>
#include <operations/Spiral.h>
#include <operations/ResourceLocaliserAction.h>

using namespace COMMON_NAMES;

typedef actionlib::SimpleActionServer<state_machines::ScoutStateMachineTaskAction> SM_SERVER;

const std::set<STATE_MACHINE_TASK> SCOUT_TASKS = {
    STATE_MACHINE_TASK::SCOUT_SEARCH_VOLATILE,
    STATE_MACHINE_TASK::SCOUT_STOP_SEARCH,
    STATE_MACHINE_TASK::SCOUT_LOCATE_VOLATILE,
    STATE_MACHINE_TASK::SCOUT_UNDOCK,
};

class ScoutStateMachine
{
private:
  ros::NodeHandle nh_;

  std::string robot_name_;

  const double SLEEP_TIME = 0.5;
  const double ROTATION_SPEED = 0.5;

  ros::ServiceClient spiralClient_;
  
  typedef actionlib::SimpleActionClient<operations::ResourceLocaliserAction> ResourceLocaliserClient_;
  ResourceLocaliserClient_* resource_localiser_client_;

  typedef actionlib::SimpleActionClient<operations::NavigationAction> NavigationClient;
  NavigationClient *navigation_client_;
  operations::NavigationGoal navigation_action_goal_;

  typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> NavigationVisionClient;
  NavigationVisionClient *navigation_vision_client_;
  operations::NavigationVisionGoal navigation_vision_goal_;

  /**
   * @brief Start the searching algorithm
   * 
   */
  bool startSearchingVolatile();

  /**
   * @brief Start the searching algorithm
   * 
   */
  bool stopSearchingVolatile();

  /**
   * @brief Start the searching algorithm
   * 
   */
  bool resumeSearchingVolatile(const bool resume);

  /**
   * @brief Pinpoint the location of the volatile, and stand on it
   * 
   */
  bool locateVolatile();

  /**
   * @brief Move away from the excavator to continue on the trajectory
   * 
   */
  bool undockRobot();

public:
  ScoutStateMachine(ros::NodeHandle nh, const std::string &robot_name);

  ~ScoutStateMachine();

  friend void cancelGoal(ScoutStateMachine *sm);
  friend void execute(const state_machines::ScoutStateMachineTaskGoalConstPtr &goal, SM_SERVER *as, ScoutStateMachine *sm);
};

// #endif // SCOUT_STATE_MACHINE_H