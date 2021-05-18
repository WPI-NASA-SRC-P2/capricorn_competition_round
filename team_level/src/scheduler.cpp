#include <team_level/scheduler.h>

Scheduler::Scheduler(ros::NodeHandle nh, const int team_number): nh_(nh)
{
  initTeam(team_number);
}

Scheduler::~Scheduler()
{
  delete scout_client_;
  delete excavator_client_;
  delete hauler_client_;
}

void Scheduler::initTeam(const int team_number)
{
  SCOUT = team_number == 1 ? SCOUT_1 : SCOUT_2;
  EXCAVATOR = team_number == 1 ? EXCAVATOR_1 : EXCAVATOR_2;
  HAULER = team_number == 1 ? HAULER_1 : HAULER_2;
  
  initClients();
}

void Scheduler::initClients()
{
  scout_client_ = new RobotClient(CAPRICORN_TOPIC + SCOUT + "/" + SCOUT + STATE_MACHINE_ACTIONLIB, true);
  excavator_client_ = new RobotClient(CAPRICORN_TOPIC + EXCAVATOR + "/" + EXCAVATOR + STATE_MACHINE_ACTIONLIB, true);
  hauler_client_ = new RobotClient(CAPRICORN_TOPIC + HAULER + "/" + HAULER + STATE_MACHINE_ACTIONLIB, true);
}

void Scheduler::startScheduler()
{
  start_scheduler_ = true;
  schedulerLoop();
}

void Scheduler::schedulerLoop()
{
  init();
  ROS_INFO("All State machines connected!");

  startScout();
  
  while (ros::ok() && start_scheduler_)
  {
    updateRobotStatus();

    updateScout();
    updateExcavator();
    updateHauler();

    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
}

void Scheduler::stopScheduler()
{
  start_scheduler_ = false;
}


void Scheduler::init()
{
  scout_client_->waitForServer();
  excavator_client_->waitForServer();
  hauler_client_->waitForServer();

  // Default value is 0, which breaks a few things
  scout_goal_.task = -1;
  excavator_goal_.task = -1;
  hauler_goal_.task = -1;
}

void Scheduler::startSearching()
{
  state_machines::RobotStateMachineTaskGoal state_machine_goal;
}

void Scheduler::updateRobotStatus()
{
  scout_done_ = scout_client_-> getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
  excavator_done_ = excavator_client_-> getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
  hauler_done_ = hauler_client_-> getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void Scheduler::startScout()
{
  sendScoutGoal(SCOUT_SEARCH_VOLATILE);
}

void Scheduler::updateScout()
{
  if(excavator_goal_.task == EXCAVATOR_GO_TO_SCOUT && excavator_done_)
    sendScoutGoal(SCOUT_UNDOCK);
  else if(scout_goal_.task == SCOUT_UNDOCK && scout_done_)
    sendScoutGoal(SCOUT_SEARCH_VOLATILE);
}

void Scheduler::updateExcavator()
{
  if(scout_goal_.task == SCOUT_SEARCH_VOLATILE && scout_done_)
    sendExcavatorGoal(EXCAVATOR_GO_TO_SCOUT);
  else if(excavator_goal_.task == EXCAVATOR_GO_TO_SCOUT && excavator_done_)
    sendExcavatorGoal(EXCAVATOR_PARK_AND_PUB);
  else if(hauler_goal_.task == HAULER_PARK_AT_EXCAVATOR && hauler_done_)
    sendExcavatorGoal(EXCAVATOR_DIG_AND_DUMP_VOLATILE);  
}

void Scheduler::updateHauler()
{
  if(scout_goal_.task == SCOUT_SEARCH_VOLATILE && scout_done_)
    sendHaulerGoal(HAULER_FOLLOW_EXCAVATOR);
  else if(excavator_goal_.task == EXCAVATOR_PARK_AND_PUB && excavator_done_)
    sendHaulerGoal(HAULER_PARK_AT_EXCAVATOR);
  else if(excavator_goal_.task == EXCAVATOR_DIG_AND_DUMP_VOLATILE && excavator_done_)
    sendHaulerGoal(HAULER_DUMP_VOLATILE_TO_PROC_PLANT);  
  else if(hauler_goal_.task == HAULER_DUMP_VOLATILE_TO_PROC_PLANT && hauler_done_)
    sendHaulerGoal(HAULER_FOLLOW_EXCAVATOR);  
}

void Scheduler::sendScoutGoal(const STATE_MACHINE_TASK task)
{
  sendRobotGoal(scout_client_, scout_goal_, task);
}

void Scheduler::sendExcavatorGoal(const STATE_MACHINE_TASK task)
{
  sendRobotGoal(excavator_client_, excavator_goal_, task);
}

void Scheduler::sendHaulerGoal(const STATE_MACHINE_TASK task)
{
  sendRobotGoal(hauler_client_, hauler_goal_, task);
}

void Scheduler::sendRobotGoal(RobotClient* robot_client, state_machines::RobotStateMachineTaskGoal& robot_goal, const STATE_MACHINE_TASK task)
{
  if(robot_goal.task != task)
  {
    ROS_WARN_STREAM("Setting Task"<<task);
    robot_goal.task = task;
    robot_client->sendGoal(robot_goal);
  }
}