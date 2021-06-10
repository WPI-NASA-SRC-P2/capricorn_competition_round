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

  //startScout();
  startExcavator();
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
}

void Scheduler::updateRobotStatus()
{
  // THIS IS BAD, MUST BE HANDLED //
  // If the task fails, then that logic should be taken care of as well
  scout_task_completed_ = scout_client_->getState().isDone();         //== actionlib::SimpleClientGoalState::SUCCEEDED;
  excavator_task_completed_ = excavator_client_->getState().isDone(); //== actionlib::SimpleClientGoalState::SUCCEEDED;
  hauler_task_completed_ = hauler_client_->getState().isDone();       //== actionlib::SimpleClientGoalState::SUCCEEDED;
}

void Scheduler::startHauler()
{
  hauler_desired_task = HAULER_RESET_ODOM_AT_HOPPER;
}

void Scheduler::startExcavator()
{
  excavator_desired_task = EXCAVATOR_GOTO_DEFAULT_ARM_POSE;
}

void Scheduler::startScout()
{
  scout_desired_task = SCOUT_SEARCH_VOLATILE;
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

  //This boolean checks IF the hauler has COMPLETED dumping volatile to processing plant.
  bool dumping_done = hauler_goal_.task == HAULER_DUMP_VOLATILE_TO_PROC_PLANT && hauler_task_completed_;
  //This boolean checks if hauler has COMPLETED resetting its odometry at the HOPPER.
  bool odom_reset_done = hauler_goal_.task == HAULER_RESET_ODOM_AT_HOPPER && hauler_task_completed_;
  //This boolean checks if the hauler is NOT dumping volatile, basically if its GOING TO THE LOC, PARKING AT THE EXCAVATOR or is not doing anything.
  bool not_dumping = hauler_goal_.task != HAULER_DUMP_VOLATILE_TO_PROC_PLANT;

  //Conditions of excavator that should be met for the HAULER TO GO TO VOLATILE LOCATION.
  if ((dumping_done || not_dumping || odom_reset_done) && hauler_goal_.task != HAULER_PARK_AT_EXCAVATOR)
  {
    bool excavator_going = excavator_goal_.task == EXCAVATOR_GO_TO_SCOUT || excavator_goal_.task == EXCAVATOR_GO_TO_LOC;
    bool excavator_waiting = (excavator_goal_.task == EXCAVATOR_PARK_AND_PUB);
    if (excavator_going || excavator_waiting)
      hauler_desired_task = (HAULER_FOLLOW_EXCAVATOR);
  }
  //Conditions of excavator that should be met for the HAULER_PARK_AT_EXCAVATOR.
  if (excavator_goal_.task == EXCAVATOR_PARK_AND_PUB && excavator_task_completed_)
  {
    if (hauler_goal_.task == HAULER_FOLLOW_EXCAVATOR && hauler_task_completed_)
      hauler_desired_task = (HAULER_PARK_AT_EXCAVATOR);
  }
  //Conditions of excavator that should be met for the HAULER_DUMP_VOLATILE_TO_PROC_PLANT.
  if (excavator_goal_.task == EXCAVATOR_DIG_AND_DUMP_VOLATILE && excavator_task_completed_ && hauler_got_stuff_)
  {
    hauler_desired_task = (HAULER_DUMP_VOLATILE_TO_PROC_PLANT);
    hauler_got_stuff_ = false;
  }
}

void Scheduler::sendScoutGoal(const STATE_MACHINE_TASK task)
{
  // if (task == SCOUT_UNDOCK)
  // {
  //   // reset the scout's odometry when it is facing the excavator
  //   geometry_msgs::PoseStamped scout_goal_pose;
  //   scout_goal_pose = rotatePose(excavator_pose_, M_PI);
  //   sendRobotGoal(SCOUT, scout_client_, scout_goal_, task, scout_goal_pose);
  // }
  // else
  // {
    sendRobotGoal(SCOUT, scout_client_, scout_goal_, task);
  // }
}

void Scheduler::sendExcavatorGoal(const STATE_MACHINE_TASK task)
{
  if (task == EXCAVATOR_GO_TO_LOC || task == EXCAVATOR_GO_TO_SCOUT)
  {
    std::lock_guard<std::mutex> lock(scout_pose_mutex);
    geometry_msgs::PoseStamped excavator_goal_pose;
    excavator_goal_pose.header.frame_id = MAP;
    sendRobotGoal(EXCAVATOR, excavator_client_, excavator_goal_, task, scout_pose_);
  }
  // if (task == EXCAVATOR_GO_TO_SCOUT)
  // {
  //   sendRobotGoal(EXCAVATOR, excavator_client_, excavator_goal_, task, scout_pose_); 
  // }
  // if (task == EXCAVATOR_DIG_AND_DUMP_VOLATILE)
  // {
  //   // reset odometry during dig and dump volatile state!
  //   geometry_msgs::PoseStamped excavator_goal_pose;
  //   excavator_goal_pose = rotatePose(hauler_pose_, M_PI);
  //   sendRobotGoal(EXCAVATOR, excavator_client_, excavator_goal_, task, excavator_goal_pose);
  // }
  //Adding what to do when excavator reset odometry is being called

  else
    sendRobotGoal(EXCAVATOR, excavator_client_, excavator_goal_, task);
}

void Scheduler::sendHaulerGoal(const STATE_MACHINE_TASK task)
{
  // if (task == HAULER_GO_TO_LOC)
  // {
  //   std::lock_guard<std::mutex> lock(excavator_pose_mutex);

  //   bool excavator_waiting = (excavator_goal_.task == EXCAVATOR_PARK_AND_PUB);

  //   geometry_msgs::PoseStamped hauler_goal_pose;
  //   geometry_msgs::PoseStamped ref_pose = excavator_waiting ? excavator_pose_ : scout_pose_;
  //   hauler_goal_pose.header.frame_id = MAP;
  //   hauler_goal_pose.pose = NavigationAlgo::getPointCloserToOrigin(ref_pose.pose, hauler_pose_.pose, -5.0);

  //   sendRobotGoal(HAULER, hauler_client_, hauler_goal_, task, hauler_goal_pose);
  // }
  // if (task == HAULER_GO_BACK_TO_EXCAVATOR)
  // {
  //   sendRobotGoal(HAULER, hauler_client_, hauler_goal_, task, excavator_pose_);
  // }
  // else
    sendRobotGoal(HAULER, hauler_client_, hauler_goal_, task);
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

/**
 * @brief Reset odometry pose conversion logic. Used to rotate the pose of a robot by 180 degrees for resetOdom
 * 
 * @param msg, @param theta (radians)
 */
geometry_msgs::PoseStamped Scheduler::rotatePose(const geometry_msgs::PoseStamped &msg, double theta)
{
  // convert the input message's orientation quaternion into rpy for easy rotation
  tf2::Quaternion q(msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);

  // rotate the yaw by pi to have 180 degree rotation of the goal pose
  q.setRPY(r, p, y + theta);

  geometry_msgs::PoseStamped new_pose;
  new_pose.header = msg.header;
  //new_pose.pose.position.x = (msg.pose.position.x) + (3*(std::cos(y))); // potential offset, might need to look at which one needs cosine or sine based on wgat is observed
  //new_pose.pose.position.y = (msg.pose.position.y) + (3*(std::sin(y))); 
  //new_pose.pose.position.z = (msg.pose.position.z);  

  new_pose.pose.position = msg.pose.position;
  //new_pose.pose.orientation.z = (msg->pose.pose.orientation.z) + 1; // potential offset?
  new_pose.pose.orientation = tf2::toMsg(q);

  return new_pose;

  /**
  * @brief RESET LOGIC reference
  * geometry_msgs::PoseStamped new_excav_pose;
  * new_excav_pose.header.frame_id = ref_robot_name + BASE_FOOTPRINT;
  * new_excav_pose.pose.position.x = 3;
  * new_excav_pose.pose.orientation.z = 1;
  * NavigationAlgo::transformPose(new_excav_pose, MAP, )

  */
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