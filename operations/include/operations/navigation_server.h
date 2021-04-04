#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <operations/TrajectoryWithVelocities.h>
#include <nav_msgs/Odometry.h>
#include <srcp2_msgs/BrakeRoverSrv.h>

#include <operations/navigation_algorithm.h>
#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

#include <math.h>
#include <string>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mutex>

// Create a type called `Server` that is a SimpleActionServer that uses the NavigationAction type
typedef actionlib::SimpleActionServer<operations::NavigationAction> Server;

// Allows us to omit COMMON_NAMES from anything in that namespace
using namespace COMMON_NAMES;

class NavigationServer
{
public:
    NavigationServer(ros::NodeHandle& nh, std::string robot_name);
    ~NavigationServer();

private:
    const float BASE_SPEED = 0.6;
    const float DIST_EPSILON = 0.05;
    const float ANGLE_EPSILON = 0.01;

    std::string robot_name_;

    // Publishers for each wheel velocity and steering controller
    ros::Publisher front_left_vel_pub_, front_right_vel_pub_, back_left_vel_pub_, back_right_vel_pub_;
    ros::Publisher front_left_steer_pub_, front_right_steer_pub_, back_left_steer_pub_, back_right_steer_pub_;

    // Used to get the current robot pose
    ros::Subscriber update_current_robot_pose_;

    ros::ServiceClient brake_client_;

    // Declare robot pose to be used globally
    geometry_msgs::PoseStamped robot_pose_;
    std::mutex pose_mutex_;

    // Used to perform transforms between the robot and map reference frames
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener* listener_;

    // Whether we are currently manually driving, or automatically following a trajectory
    bool manual_driving_ = false;

    /**
     * @brief 
     * 
     */
    void initVelocityPublisher(ros::NodeHandle &nh, const std::string &robot_name);

    /**
    * @brief Initialise the publishers for wheel steering angles
    */
    void initSteerPublisher(ros::NodeHandle &nh, const std::string &robot_name);

    /**
    * @brief Initialise publihers used to debug
    */
    void initDebugPublisher(ros::NodeHandle &nh, const std::string &robot_name);

    /**
    * @brief Call Publisher initializers
    * 
    */
    void initPublishers(ros::NodeHandle &nh, const std::string &robot_name);

    /**
    * @brief Subscribes to an odometry topic, and updates the global robot_pose
    * 
    * @param msg The odometry message to process
    */
    void updateRobotPose(const nav_msgs::Odometry::ConstPtr &msg);

    geometry_msgs::PoseStamped* getRobotPose();

    /**
    * @brief Initialize the subscriber for robot position
    * 
    */
    void initSubscribers(ros::NodeHandle &nh, std::string &robot_name);

    /**
    * @brief Publish the message over rostopic
    * 
    * @param publisher   publisher over which message will be published
    * @param data        data that will be published over the publisher
    */
    void publishMessage(ros::Publisher &publisher, float data);

    /**
    * @brief Steers the robot wheels for the angles
    * 
    * @param angles Steering angles
    *                  The vector will be in order:
    *                  Clockwise from top, starting with FRONT_LEFT
    * 
    *          element 0: Front Left Wheel
    *          element 1: Front Right Wheel
    *          element 2: Back Right Wheel
    *          element 3: Back Left Wheel
    */
    void steerRobot(const std::vector<double> &angles);

    /**
    * @brief Steers the robot wheels for the angles
    * 
    * @param angle Angles at which the robot wheels will be steered
    */
    void steerRobot(const double angle);

    /**
    * @brief Move robot wheels with the given velocities
    * 
    * @param velocity Wheel Velocities
        *                  The vector will be in order:
        *                  Clockwise from top, starting with FRONT_LEFT
        * 
        *          element 0: Front Left Wheel
        *          element 1: Front Right Wheel
        *          element 2: Back Right Wheel
        *          element 3: Back Left Wheel
    */
    void moveRobotWheels(const std::vector<double> velocity);

    /**
    * @brief Move robot wheels with the given velocity 
    * 
    * @param velocity velocity for the wheels
    */
    void moveRobotWheels(const double velocity);

    /**
     * @brief Sends a goal received from the Robot SM to the planner. Receives and returns the trajectory
     * 
     * @param goal The end goal for the robot to go to
     * @return operations::TrajectoryWithVelocities* 
     */
    operations::TrajectoryWithVelocities* sendGoalToPlanner(const operations::NavigationGoalConstPtr &goal);

    /**
     * @brief 
     * 
     * @param brake 
     */
    void brakeRobot(bool brake);

    bool rotateRobot(const geometry_msgs::PoseStamped& target_robot_pose);

    bool driveDistance(double delta_distance);

    void automaticDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    void linearDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    void angularDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    void spiralDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    void followDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    void execute(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    void cancelGoal();
    
};