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

  //   scout_odom_sub_ = nh_.subscribe(CAPRICORN_TOPIC + SCOUT + CHEAT_ODOM_TOPIC, 1000, &Scheduler::updateScoutPose, this);
  //   excavator_odom_sub_ = nh_.subscribe(CAPRICORN_TOPIC + EXCAVATOR + CHEAT_ODOM_TOPIC, 1000, &Scheduler::updateExcavatorPose, this);
  //   hauler_odom_sub_ = nh_.subscribe(CAPRICORN_TOPIC + HAULER + CHEAT_ODOM_TOPIC, 1000, &Scheduler::updateHaulerPose, this);

  scout_odom_sub_ = nh_.subscribe(SCOUT + RTAB_ODOM_TOPIC, 1000, &Scheduler::updateScoutPose, this);
  excavator_odom_sub_ = nh_.subscribe(EXCAVATOR + RTAB_ODOM_TOPIC, 1000, &Scheduler::updateExcavatorPose, this);
  hauler_odom_sub_ = nh_.subscribe(HAULER + RTAB_ODOM_TOPIC, 1000, &Scheduler::updateHaulerPose, this);

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
  // ensure that hauler pose is set before doing anything else
  startHauler();

  while (ros::ok() && start_scheduler_)
  {
    updateRobotStatus();

    updateHauler();
    updateExcavator();
    updateScout();

    sendScoutGoal(scout_desired_task);
    sendExcavatorGoal(excavator_desired_task);
    sendHaulerGoal(hauler_desired_task);

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

void Scheduler::updateRobotStatus()
{
  // THIS IS BAD, MUST BE HANDLED //
  // If the task fails, then that logic should be taken care of as well
  scout_task_completed_ = scout_client_->getState().isDone();         // == actionlib::SimpleClientGoalState::SUCCEEDED;
  excavator_task_completed_ = excavator_client_->getState().isDone(); // == actionlib::SimpleClientGoalState::SUCCEEDED;
  hauler_task_completed_ = hauler_client_->getState().isDone();       // == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void Scheduler::startExcavator()
{
  sendExcavatorGoal(EXCAVATOR_GOTO_DEFAULT_ARM_POSE);
}

void Scheduler::startScout()
{
  sendScoutGoal(SCOUT_SEARCH_VOLATILE);
  // sendScoutGoal(SCOUT_SYNC_ODOM);
}

void Scheduler::startHauler()
{
  sendHaulerGoal(HAULER_GO_TO_PROC_PLANT);
  hauler_goal_.task = HAULER_GO_TO_PROC_PLANT;
  while (!hauler_task_completed_)
  {
    ROS_WARN("HAULER GOING TO PROC PLANT");
    updateRobotStatus();
  }
  hauler_task_completed_ = false;
  sendHaulerGoal(HAULER_PARK_AT_HOPPER);
  hauler_goal_.task = HAULER_PARK_AT_HOPPER;
  while (!hauler_task_completed_)
  {
    ROS_WARN("HAULER PARKING AT HOPPER");
    updateRobotStatus();
  }
  hauler_task_completed_ = false;
  sendHaulerGoal(HAULER_RESET_ODOM);
  hauler_goal_.task = HAULER_RESET_ODOM;
  while (!hauler_task_completed_)
  {
    ROS_WARN("HAULER_RESETTING ODOM");
    updateRobotStatus();
  }

  ROS_ERROR("Hauler has been reset!"); //Once hauler has reached the hopper, it uses its reset odom using ground truth. Make sure, inirtialize rtabmap is called with use_gt=false
}

void Scheduler::updateScout()
{
  if (excavator_goal_.task == EXCAVATOR_GO_TO_SCOUT && excavator_task_completed_)
    scout_desired_task = (SCOUT_UNDOCK);
  if (scout_goal_.task == SCOUT_UNDOCK && scout_task_completed_)
    scout_desired_task = (SCOUT_SEARCH_VOLATILE);
}

void Scheduler::updateExcavator()
{

  static bool first_task = true;

  if (scout_goal_.task == SCOUT_SEARCH_VOLATILE && scout_task_completed_)
  {
    if ((excavator_goal_.task == EXCAVATOR_DIG_AND_DUMP_VOLATILE && excavator_task_completed_) || first_task)
    {
      excavator_desired_task = (EXCAVATOR_GO_TO_LOC);
      first_task = false;
    }
  }
  if (excavator_goal_.task == EXCAVATOR_GO_TO_LOC && excavator_task_completed_)
  {
    excavator_desired_task = (EXCAVATOR_GO_TO_SCOUT);
  }
  if (excavator_goal_.task == EXCAVATOR_GO_TO_SCOUT && excavator_task_completed_)
    excavator_desired_task = (EXCAVATOR_PARK_AND_PUB);
  if (hauler_goal_.task == HAULER_PARK_AT_EXCAVATOR && hauler_task_completed_)
  {
    excavator_desired_task = (EXCAVATOR_DIG_AND_DUMP_VOLATILE);
    hauler_got_stuff_ = true;
  }
}

void Scheduler::updateHauler()
{
  ROS_INFO_STREAM("Scout Task:" << (scout_goal_.task) << " task completed:" << scout_task_completed_);
  ROS_INFO_STREAM("Excav Task:" << (excavator_goal_.task) << " task completed:" << excavator_task_completed_);
  ROS_INFO_STREAM("Hauler Task:" << (hauler_goal_.task) << " task completed:" << hauler_task_completed_);

  bool dumping_done = hauler_goal_.task == HAULER_DUMP_VOLATILE_TO_PROC_PLANT && hauler_task_completed_;
  bool not_dumping = hauler_goal_.task != HAULER_DUMP_VOLATILE_TO_PROC_PLANT;

  if ((dumping_done || not_dumping) && hauler_goal_.task != HAULER_PARK_AT_EXCAVATOR)
  {
    bool excavator_going = excavator_goal_.task == EXCAVATOR_GO_TO_SCOUT || excavator_goal_.task == EXCAVATOR_GO_TO_LOC;
    bool excavator_waiting = (excavator_goal_.task == EXCAVATOR_PARK_AND_PUB);

    if (excavator_going || excavator_waiting)
      hauler_desired_task = (HAULER_GO_TO_LOC);
    else
    {
      // reset the hauler's odom at the hopper if the hauler is not moving towards the excavator
      // -> does not matter if runs multiple times while waiting for the excavator, as hauler won't be moving anyways
      hauler_desired_task = (HAULER_RESET_ODOM);
    }
  }
  if (excavator_goal_.task == EXCAVATOR_PARK_AND_PUB && excavator_task_completed_)
  {
    if (hauler_goal_.task == HAULER_GO_TO_LOC && hauler_task_completed_)
      hauler_desired_task = (HAULER_PARK_AT_EXCAVATOR);
  }
  if (excavator_goal_.task == EXCAVATOR_DIG_AND_DUMP_VOLATILE && excavator_task_completed_ && hauler_got_stuff_)
  {
    // before going back to the processing plant, face the processing plant to reset odom with excavator
    //if (hauler_goal_.task = HAULER_FACE_PROCESSING_PLANT)
    hauler_desired_task = (HAULER_DUMP_VOLATILE_TO_PROC_PLANT);
    hauler_got_stuff_ = false;
  }
}

void Scheduler::sendScoutGoal(const STATE_MACHINE_TASK task)
{
  // Scout will reset its odom w.r.t. the excavator before undocking
  // thus check w.r.t. excavator EXCAVATOR_GO_TO_SCOUT task completion
  if (task == STATE_MACHINE_TASK::SCOUT_SYNC_ODOM) //Add AND case for the hauler to be parked near the scout to be completed for this task to be run.
  {
    sendRobotGoal(SCOUT, scout_client_, scout_goal_, task, excavator_pose_);
  }
  else
  {
    sendRobotGoal(SCOUT, scout_client_, scout_goal_, task);
  }
}

void Scheduler::sendExcavatorGoal(const STATE_MACHINE_TASK task)
{
  if (task == EXCAVATOR_GO_TO_LOC)
  {
    std::lock_guard<std::mutex> lock(scout_pose_mutex);
    geometry_msgs::PoseStamped excavator_goal_pose;
    excavator_goal_pose.header.frame_id = MAP;
    excavator_goal_pose.pose = NavigationAlgo::getPointCloserToOrigin(scout_pose_.pose, excavator_pose_.pose, 5.0);

    sendRobotGoal(EXCAVATOR, excavator_client_, excavator_goal_, task, excavator_goal_pose);
  }
  else if (task == STATE_MACHINE_TASK::EXCAVATOR_SYNC_ODOM)
  {
    // check based on hauler states if completed: HAULER_GO_BACK_TO_EXCAVATOR, HAULER_PARK_AT_EXCAVATOR
    bool hauler_done_parking = (hauler_goal_.task == STATE_MACHINE_TASK::HAULER_GO_BACK_TO_EXCAVATOR || hauler_goal_.task == STATE_MACHINE_TASK::HAULER_PARK_AT_EXCAVATOR) && hauler_task_completed_;
    if (hauler_done_parking)
    {
      sendRobotGoal(EXCAVATOR, excavator_client_, excavator_goal_, task, hauler_pose_);
    }
    else
    {
      ROS_WARN("awaiting hauler parking completion"); //Crashes when the hauler odometry is null.
    }
  }
  else
    sendRobotGoal(EXCAVATOR, excavator_client_, excavator_goal_, task);
}

void Scheduler::sendHaulerGoal(const STATE_MACHINE_TASK task)
{
  if (task == HAULER_GO_TO_LOC)
  {
    std::lock_guard<std::mutex> lock(excavator_pose_mutex);

    bool excavator_waiting = (excavator_goal_.task == EXCAVATOR_PARK_AND_PUB);

    geometry_msgs::PoseStamped hauler_goal_pose;
    geometry_msgs::PoseStamped ref_pose = excavator_waiting ? excavator_pose_ : scout_pose_;
    hauler_goal_pose.header.frame_id = MAP;
    hauler_goal_pose.pose = NavigationAlgo::getPointCloserToOrigin(ref_pose.pose, hauler_pose_.pose, -5.0);

    sendRobotGoal(HAULER, hauler_client_, hauler_goal_, task, hauler_goal_pose);
  }
  else
  {
    sendRobotGoal(HAULER, hauler_client_, hauler_goal_, task);
    // ROS_INFO("Hauler pose has been reset to:");
    // ROS_INFO_STREAM(hauler_pose_);
  }
}

void Scheduler::sendRobotGoal(std::string robot_name, RobotClient *robot_client, state_machines::RobotStateMachineTaskGoal &robot_goal, const STATE_MACHINE_TASK task)
{
  if (robot_goal.task != task)
  {
    ROS_WARN_STREAM("SCHEDULER : Setting " << robot_name << " Task: " << task);

    // This is bad, should be removed //
    if (task == EXCAVATOR_PARK_AND_PUB)
      ros::Duration(20.0f).sleep();

    robot_goal.task = task;
    robot_client->sendGoal(robot_goal);
  }
}

void Scheduler::sendRobotGoal(std::string robot_name, RobotClient *robot_client, state_machines::RobotStateMachineTaskGoal &robot_goal, const STATE_MACHINE_TASK task, const geometry_msgs::PoseStamped &goal_loc)
{
  if (robot_goal.task != task)
  {
    ROS_WARN_STREAM("SCHEDULER : Setting " << robot_name << " Task: " << task);
    robot_goal.task = task;
    robot_goal.goal_loc = goal_loc;
    ROS_WARN("GOAL_LOC = ");
    ROS_WARN_STREAM(goal_loc);
    robot_client->sendGoal(robot_goal);
  }
}

/**
 * @brief Callback to the scout pose topic
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
 * @brief Callback to the Excavator pose topic
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
 * @brief Callback to the Hauler pose topic
 * 
 * @param msg 
 */
void Scheduler::updateHaulerPose(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(hauler_pose_mutex);
  hauler_pose_.header = msg->header;
  hauler_pose_.pose = msg->pose.pose;
}

/*
// Scheduler reset odom states in order of completion

// hauler reset odom
// first reset
beginning of simulation->
hauler rtabmap initialized with get_true_pose = false
startHauler() set hauler desired task to goToProcPlant()
update hauler reset_odom with use_gt = true to obtain true pose of hauler at hopper and store it in memory if the goaltask of goToProcPlant() is completed
hauler exits reset odom steps (finished -> hauler spin)
// subsequent resets
hauler arrives at hopper
update hauler sets desired task to reset_odom with use_gt = true upon detecting hauler dump volatile to processing plant completion (dumping_done boolean)
hauler exits reset odom steps (finished -> hauler spin)
//////////////////

// excavator reset odom
// first reset
excavator rtabmap initialized with get_true_pose = true
// subsequent resets
excavator finishes collecting volatiles
update hauler sets desired task to face processing plant upon detecting digging completion
update excavator sets desired task to face processing plant upon detecting digging completion
update excavator sets desired task to reset odom w.r.t. hauler odom upon detecting hauler face processing plant completion
excavator exits reset odom steps (finished -> excavator -> spin)
/////////////////

// scout reset odom
// first reset
scout rtabmap initialized with get_true_pose = true (CANNOT CALL RESET ODOM WITH use_gt = true if doing this step)
// subsequent resets
excavator finishes go to scout state
update excavator sets desired task to face processing plant upon detecting excavator go to scout completion
update scout sets desired task to face processing plant upon detecting excavator go to scout completion
update scout sets desired task to reset odom w.r.t. excavator upon detecting excavator face processing plant completion
update excavator sets desired task to turn to face scout again upon detecting scout reset odom completion
scout exits reset odom steps (finished -> update scout sets desired task to undock upon detecting excavator face scout completion)
////////////////

/////////////////////////
*/