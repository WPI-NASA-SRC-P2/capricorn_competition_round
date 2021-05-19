#include <team_level/scheduler.h>

Scheduler::Scheduler(ros::NodeHandle nh, const int team_number) : nh_(nh)
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

  scout_odom_sub_ = nh_.subscribe(CAPRICORN_TOPIC + SCOUT + CHEAT_ODOM_TOPIC, 1000, &Scheduler::updateScoutPose, this);
  excavator_odom_sub_ = nh_.subscribe(CAPRICORN_TOPIC + EXCAVATOR + CHEAT_ODOM_TOPIC, 1000, &Scheduler::updateExcavatorPose, this);
  hauler_odom_sub_ = nh_.subscribe(CAPRICORN_TOPIC + HAULER + CHEAT_ODOM_TOPIC, 1000, &Scheduler::updateHaulerPose, this);

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
  startExcavator();

  while (ros::ok() && start_scheduler_)
  {
    updateScout();
    updateExcavator();
    updateHauler();

    ros::Duration(0.5).sleep();
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
  scout_task_completed_ = scout_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
  excavator_task_completed_ = excavator_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
  hauler_task_completed_ = hauler_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void Scheduler::startExcavator()
{
  sendExcavatorGoal(EXCAVATOR_GOTO_DEFAULT_ARM_POSE);
}

void Scheduler::startScout()
{
  sendScoutGoal(SCOUT_SEARCH_VOLATILE);
}

void Scheduler::updateScout()
{
  updateRobotStatus();

  if (excavator_goal_.task == EXCAVATOR_GO_TO_SCOUT && excavator_task_completed_)
    sendScoutGoal(SCOUT_UNDOCK);
  else if (scout_goal_.task == SCOUT_UNDOCK && scout_task_completed_)
    sendScoutGoal(SCOUT_SEARCH_VOLATILE);
}

void Scheduler::updateExcavator()
{
  updateRobotStatus();

  static bool first_task = true;
  if (excavator_goal_.task == EXCAVATOR_GO_TO_LOC && excavator_task_completed_)
  {
    sendExcavatorGoal(EXCAVATOR_GO_TO_SCOUT);
  }
  else if (scout_goal_.task == SCOUT_SEARCH_VOLATILE && scout_task_completed_)
  {
    if ((excavator_goal_.task == EXCAVATOR_DIG_AND_DUMP_VOLATILE && excavator_task_completed_) || first_task)
    {
      sendExcavatorGoal(EXCAVATOR_GO_TO_LOC);
      first_task = false;
    }
  }
  else if (excavator_goal_.task == EXCAVATOR_GO_TO_SCOUT && excavator_task_completed_)
    sendExcavatorGoal(EXCAVATOR_PARK_AND_PUB);
  else if (hauler_goal_.task == HAULER_PARK_AT_EXCAVATOR && hauler_task_completed_)
    sendExcavatorGoal(EXCAVATOR_DIG_AND_DUMP_VOLATILE);
}

void Scheduler::updateHauler()
{
  updateRobotStatus();

  if (excavator_goal_.task == EXCAVATOR_DIG_AND_DUMP_VOLATILE && excavator_task_completed_)
    sendHaulerGoal(HAULER_DUMP_VOLATILE_TO_PROC_PLANT);
  else if (excavator_goal_.task == EXCAVATOR_PARK_AND_PUB && excavator_task_completed_)
  {
    if(hauler_goal_.task == HAULER_GO_TO_LOC && hauler_task_completed_)
    sendHaulerGoal(HAULER_PARK_AT_EXCAVATOR);
  }
  else if (hauler_goal_.task != HAULER_DUMP_VOLATILE_TO_PROC_PLANT)
  {
    bool excavator_going = excavator_goal_.task == EXCAVATOR_GO_TO_SCOUT || excavator_goal_.task == EXCAVATOR_GO_TO_LOC;
    bool excavator_waiting = (excavator_goal_.task == EXCAVATOR_PARK_AND_PUB);// || (excavator_goal_.task == EXCAVATOR_DIG_AND_DUMP_VOLATILE && excavator_task_completed_);
    if (excavator_going || excavator_waiting)
      sendHaulerGoal(HAULER_GO_TO_LOC);
  }
  // else if (hauler_goal_.task == HAULER_DUMP_VOLATILE_TO_PROC_PLANT && hauler_task_completed_)
  //   sendHaulerGoal(HAULER_PARK_AT_EXCAVATOR);
}

void Scheduler::sendScoutGoal(const STATE_MACHINE_TASK task)
{
  sendRobotGoal(SCOUT, scout_client_, scout_goal_, task);
}

void Scheduler::sendExcavatorGoal(const STATE_MACHINE_TASK task)
{
  if (task == EXCAVATOR_GO_TO_LOC)
  {
    std::lock_guard<std::mutex> lock(scout_pose_mutex);
    geometry_msgs::PoseStamped excavator_goal_pose;
    excavator_goal_pose.header.frame_id = MAP;
    excavator_goal_pose.pose =  NavigationAlgo::getPointCloserToOrigin(scout_pose_.pose, excavator_pose_.pose, 5.0);
    // ROS_INFO_STREAM("excavator Pose: "<<excavator_goal_pose);
    sendRobotGoal(EXCAVATOR, excavator_client_, excavator_goal_, task, excavator_goal_pose);
  }
  else
    sendRobotGoal(EXCAVATOR, excavator_client_, excavator_goal_, task);
}

void Scheduler::sendHaulerGoal(const STATE_MACHINE_TASK task)
{
  if (task == HAULER_GO_TO_LOC)
  {
    std::lock_guard<std::mutex> lock(excavator_pose_mutex);

    bool excavator_waiting = (excavator_goal_.task == EXCAVATOR_PARK_AND_PUB);// || (excavator_goal_.task == EXCAVATOR_DIG_AND_DUMP_VOLATILE && excavator_task_completed_);

    geometry_msgs::PoseStamped hauler_goal_pose;
    geometry_msgs::PoseStamped ref_pose = excavator_waiting ? excavator_pose_ : scout_pose_;
    hauler_goal_pose.header.frame_id = MAP;
    hauler_goal_pose.pose =  NavigationAlgo::getPointCloserToOrigin(ref_pose.pose, hauler_pose_.pose, -5.0);
    // ROS_INFO_STREAM("HAULER Pose: "<<hauler_goal_pose);
    sendRobotGoal(HAULER, hauler_client_, hauler_goal_, task, hauler_goal_pose);
  }
  else
  sendRobotGoal(HAULER, hauler_client_, hauler_goal_, task);
}

void Scheduler::sendRobotGoal(std::string robot_name, RobotClient *robot_client, state_machines::RobotStateMachineTaskGoal &robot_goal, const STATE_MACHINE_TASK task)
{
  if (robot_goal.task != task)
  {
    ROS_WARN_STREAM("SCHEDULER : Setting " << robot_name << " Task: " << task);
    if (task == EXCAVATOR_PARK_AND_PUB)
      ros::Duration(20.0f).sleep();

    robot_goal.task = task;
    robot_client->sendGoal(robot_goal);
  }
}

void Scheduler::sendRobotGoal(std::string robot_name, RobotClient *robot_client, state_machines::RobotStateMachineTaskGoal &robot_goal, const STATE_MACHINE_TASK task, const geometry_msgs::PoseStamped& goal_loc)
{
  if (robot_goal.task != task)
  {
    ROS_WARN_STREAM("SCHEDULER : Setting " << robot_name << " Task: " << task);

    robot_goal.task = task;
    robot_goal.goal_loc = goal_loc;
    robot_client->sendGoal(robot_goal);
  }
}


/**
 * @brief Callback to the robot pose topic
 * 
 * @param msg 
 */
void Scheduler::updateScoutPose(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(scout_pose_mutex);
  scout_pose_.header = msg->header;
  scout_pose_.pose = msg->pose.pose;
}


/**
 * @brief Callback to the robot pose topic
 * 
 * @param msg 
 */
void Scheduler::updateExcavatorPose(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(excavator_pose_mutex);
  excavator_pose_.header = msg->header;
  excavator_pose_.pose = msg->pose.pose;
}


/**
 * @brief Callback to the robot pose topic
 * 
 * @param msg 
 */
void Scheduler::updateHaulerPose(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(hauler_pose_mutex);
  hauler_pose_.header = msg->header;
  hauler_pose_.pose = msg->pose.pose;
}

