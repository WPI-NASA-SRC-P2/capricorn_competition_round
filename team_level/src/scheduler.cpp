#include <team_level/scheduler.h>

Scheduler::Scheduler(ros::NodeHandle nh, const int team_number): nh_(nh)
{
  initTeam(team_number);
}

Scheduler::~Scheduler()
{
  for(auto robot_client : map_of_clients_)
    delete robot_client.second;
}

void Scheduler::initTeam(const int team_number)
{
  if (team_number == 1)
    robots_in_team_ = std::vector<std::string>{SCOUT_1, EXCAVATOR_1, HAULER_1};
  else
    robots_in_team_ = std::vector<std::string>{SCOUT_2, EXCAVATOR_2, HAULER_2};

  initClients();
}

void Scheduler::initClients()
{
  for(auto robot : robots_in_team_)
    map_of_clients_[robot] = new RobotClient(robot + STATE_MACHINE_ACTIONLIB, true);
}

void Scheduler::startScheduler()
{
  start_scheduler_ = true;
  schedulerLoop();
}

void Scheduler::schedulerLoop()
{
  for (auto client : map_of_clients_)
    client.second->waitForServer();

  while (ros::ok() && start_scheduler_)
  {
    switch (robot_state_)
    {
      case(SCHEDULER_STATES::INIT):
        init();
        break;
        
      case(SCHEDULER_STATES::SEARCHING):
        startSearching();
        break;
        
      case(SCHEDULER_STATES::INIT_ODOM_AT_HOPPER):
        initOdomAtHopper();
        break;
        
      case(SCHEDULER_STATES::GO_TO_VOLATILE):
        goToVolatile();
        break;
        
      case(SCHEDULER_STATES::PARK_ROBOTS):
        parkRobots();
        break;
        
      case(SCHEDULER_STATES::EXCAVATE_VOLATILE):
        excavateVolatile();
        break;
        
      case(SCHEDULER_STATES::DEPOSIT_VOLATILE):
        depositVolatile();
        break;
        
      case(SCHEDULER_STATES::RESET_HAULER_ODOM):
        resetHaulerOdom();
        break;
        
      case(SCHEDULER_STATES::REUNITE_TEAM):
        reuniteTeam();
        break;
        
      default:
        ROS_ERROR("Incorrect Scheduler state provided!");
        break;
    }
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

}

void Scheduler::startSearching()
{

}

void Scheduler::initOdomAtHopper()
{

}

void Scheduler::goToVolatile()
{

}

void Scheduler::parkRobots()
{

}

void Scheduler::excavateVolatile()
{

}

void Scheduler::depositVolatile()
{

}

void Scheduler::resetHaulerOdom()
{

}

void Scheduler::reuniteTeam()
{

}
