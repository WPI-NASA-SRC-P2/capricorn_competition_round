#include <team_level/robot_pose_register.h>

RobotPoseRegister::RobotPoseRegister(ros::NodeHandle nh)
{
    scout_1_odom_sub = nh.subscribe("/" + SCOUT_1_NAME + RTAB_ODOM_TOPIC, 1000, &RobotPoseRegister::odomCBScout1, this);
    scout_2_odom_sub = nh.subscribe("/" + SCOUT_2_NAME + RTAB_ODOM_TOPIC, 1000, &RobotPoseRegister::odomCBScout2, this);
    scout_3_odom_sub = nh.subscribe("/" + SCOUT_3_NAME + RTAB_ODOM_TOPIC, 1000, &RobotPoseRegister::odomCBScout3, this);
    excavator_1_odom_sub = nh.subscribe("/" + EXCAVATOR_1_NAME + RTAB_ODOM_TOPIC, 1000, &RobotPoseRegister::odomCBExcavator1, this);
    excavator_2_odom_sub = nh.subscribe("/" + EXCAVATOR_2_NAME + RTAB_ODOM_TOPIC, 1000, &RobotPoseRegister::odomCBExcavator2, this);
    excavator_3_odom_sub = nh.subscribe("/" + EXCAVATOR_3_NAME + RTAB_ODOM_TOPIC, 1000, &RobotPoseRegister::odomCBExcavator3, this);
    hauler_1_odom_sub = nh.subscribe("/" + HAULER_1_NAME + RTAB_ODOM_TOPIC, 1000, &RobotPoseRegister::odomCBHauler1, this);
    hauler_2_odom_sub = nh.subscribe("/" + HAULER_2_NAME + RTAB_ODOM_TOPIC, 1000, &RobotPoseRegister::odomCBHauler2, this);
    hauler_3_odom_sub = nh.subscribe("/" + HAULER_3_NAME + RTAB_ODOM_TOPIC, 1000, &RobotPoseRegister::odomCBHauler3, this);
}

geometry_msgs::PoseStamped RobotPoseRegister::getRobotPose(ROBOTS_ENUM robot)
{
    geometry_msgs::PoseStamped robot_pose;
    if(ROBOT_ENUM_POSE_MAP.find(robot) != ROBOT_ENUM_POSE_MAP.end()) {
        robot_pose = ROBOT_ENUM_POSE_MAP[robot];
    }
    else {
        ROS_INFO_STREAM("Haven't received any message for the robot enum"<< robot <<" yet");
    }
    return robot_pose;
}

void RobotPoseRegister::odomCBScout1(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header = msg->header;
    robot_pose.pose = msg->pose.pose;
    ROBOT_ENUM_POSE_MAP[SCOUT_1] = robot_pose;
}

void RobotPoseRegister::odomCBScout2(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header = msg->header;
    robot_pose.pose = msg->pose.pose;
    ROBOT_ENUM_POSE_MAP[SCOUT_2] = robot_pose;
}

void RobotPoseRegister::odomCBScout3(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header = msg->header;
    robot_pose.pose = msg->pose.pose;
    ROBOT_ENUM_POSE_MAP[SCOUT_3] = robot_pose;
}

void RobotPoseRegister::odomCBExcavator1(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header = msg->header;
    robot_pose.pose = msg->pose.pose;
    ROBOT_ENUM_POSE_MAP[EXCAVATOR_1] = robot_pose;
}

void RobotPoseRegister::odomCBExcavator2(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header = msg->header;
    robot_pose.pose = msg->pose.pose;
    ROBOT_ENUM_POSE_MAP[EXCAVATOR_2] = robot_pose;
}

void RobotPoseRegister::odomCBExcavator3(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header = msg->header;
    robot_pose.pose = msg->pose.pose;
    ROBOT_ENUM_POSE_MAP[EXCAVATOR_3] = robot_pose;
}

void RobotPoseRegister::odomCBHauler1(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header = msg->header;
    robot_pose.pose = msg->pose.pose;
    ROBOT_ENUM_POSE_MAP[HAULER_1] = robot_pose;
}

void RobotPoseRegister::odomCBHauler2(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header = msg->header;
    robot_pose.pose = msg->pose.pose;
    ROBOT_ENUM_POSE_MAP[HAULER_2] = robot_pose;
}

void RobotPoseRegister::odomCBHauler3(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header = msg->header;
    robot_pose.pose = msg->pose.pose;
    ROBOT_ENUM_POSE_MAP[HAULER_3] = robot_pose;
}
