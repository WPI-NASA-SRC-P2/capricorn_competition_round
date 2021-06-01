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
    NavigationServer(ros::NodeHandle &nh, std::string robot_name);
    ~NavigationServer();

private:
    // Default speeds for straight lines and turn in place (linear wheel velocity in m/s)
    const float BASE_DRIVE_SPEED = 0.6; //Need to experiment, defaults are 0.6 and 0.3
    const float BASE_SPIN_SPEED = 0.3;

    // Tolerances for linear and angular moves
    const float DIST_EPSILON = 1;    // default = 0.05
    const float ANGLE_EPSILON = 0.2; // default = 0.01
    const float SPIRAL_SPEED = 0.1;  // default = 0.5

    // How far the robot should travel before it asks for a new trajectory, in meters. Used in automaticDriving.
    const double TRAJECTORY_RESET_DIST = 5;

    std::string robot_name_;

    // The actionlib server
    Server *server_;

    // Rate limiter
    ros::Rate *update_rate_;

    // Publishers for each wheel velocity and steering controller
    ros::Publisher front_left_vel_pub_, front_right_vel_pub_, back_left_vel_pub_, back_right_vel_pub_;
    ros::Publisher front_left_steer_pub_, front_right_steer_pub_, back_left_steer_pub_, back_right_steer_pub_;

    // Debug publisher. Can be used to publish any PoseStamped. Used to visualize in RViz
    ros::Publisher waypoint_pub_;

    // Used to get the current robot pose
    ros::Subscriber update_current_robot_pose_;

    ros::ServiceClient brake_client_;

    // If true, use crab drive. If false, use point-and-go drive. Set in the constructor from a parameter
    bool CRAB_DRIVE_;

    // Declare robot pose to be used globally
    geometry_msgs::PoseStamped robot_pose_;
    std::mutex pose_mutex_;

    // Used to perform transforms between the robot and map reference frames
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener *listener_;

    // Whether we are currently manually driving, or automatically following a trajectory
    bool manual_driving_ = false;

    // Continue tracing spiral until this variable is true
    bool spiral_motion_continue_ = true;

    // How much distance the robot has traveled since the last planner call. Compared against TRAJECTORY_RESET_DIST.
    double total_distance_traveled_ = 0;

    // Whether we should get a new trajectory from the planner. Set by driveDistance in automaticDriving.
    bool get_new_trajectory_ = false;

    /**
     * @brief Initialize the publishers for wheel speeds
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
    void initDebugPublishers(ros::NodeHandle &nh, const std::string &robot_name);

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

    geometry_msgs::PoseStamped *getRobotPose();

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
    operations::TrajectoryWithVelocities sendGoalToPlanner(const geometry_msgs::PoseStamped &goal);

    /**
     * @brief Used to transform each pose in the trajectory into the map frame.
     * 
     * @param traj The trajectory to transform the poses of
     * @return operations::TrajectoryWithVelocities The resulting trajectory
     */
    operations::TrajectoryWithVelocities getTrajInMapFrame(const operations::TrajectoryWithVelocities &traj);

    /**
     * @brief Set the manual brake on the robot.
     * 
     * @param brake True if the brake should be set, false, if it should be released.
     */
    void brakeRobot(bool brake);

    /**
     * @brief Rotates the wheels to point at a target pose.
     * 
     * @param target_robot_pose The pose to point the wheels towards. Can be in any frame of reference.
     * @return true Succeeded in turning the wheels to that angle
     * @return false Failed to turn the wheels to that angle
     */
    bool rotateWheels(const geometry_msgs::PoseStamped &target_robot_pose);

    /**
     * @brief Rotates the robot in place to face a target pose.
     * 
     * @param target_robot_pose The pose to face, in any frame of reference.
     * @return true Succeeded in turning the robot in place.
     * @return false Failed to complete the turn.
     */
    bool rotateRobot(const geometry_msgs::PoseStamped &target_robot_pose);

    /**
     * @brief Drives the robot a specific distance.
     * 
     * @param delta_distance The distance between the robot and the goal position.
     * @return true Suceeded in driving the robot to the specified distance.
     * @return false Failed in driving the robot to the specified distance.
     */
    bool driveDistance(double delta_distance);

    /**
     * @brief Drives to a goal, based on waypoints generated by the local planner. NAV_TYPE::GOAL
     * 
     * @param goal The goal of the action. Uses the PoseStamped member to pass to the planner.
     *             HOTFIX: THIS MUST NOT BE A REF, AS ITS NOT CONSTANT
     * @param action_server The action server that this function is operating on.
     */
    void automaticDriving(geometry_msgs::PoseStamped goal_pose, Server *action_server);

    /**
     * @brief THIS IS A HOTFIX FOR THE NAVIGATION THINKING IT HAS REACHED A GOAL. MUST BE FIXED ASAP
     * 
     * @param goal The goal of the action. Uses the PoseStamped member to pass to the planner.
     * @param action_server The action server that this function is operating on.
     */
    void automaticDrivingCrosscheck(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    /**
     * @brief Manually drive forwards or backwards. NAV_TYPE::MANUAL
     * 
     * @param goal The goal of the action. Uses the forward_velocity member to choose a velocity
     * @param action_server The action server that this function is operating on.
     */
    void linearDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    /**
     * @brief Manually spins in place. NAV_TYPE::MANUAL
     * 
     * @param goal The goal of the action. Uses the angular_velocity member to choose a velocity
     * @param action_server The action server that this function is operating on.
     */
    void angularDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    /**
     * @brief Revolve the robot around a geometry_msgs::Point. NAV_TYPE::REVOLVE
     * 
     * @param goal The goal of the action. Uses the angular_velocity member to choose a velocity
     * @param action_server 
     */
    void revolveDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    /**
     * @brief Revolve the robot around a geometry_msgs::Point.
     *        Actual rotation logic
     * 
     * @param revolve_about      Point of rotation of the robot
     * @param forward_velocity   Velocity with which the center of the robot should travel
     */
    void revolveRobot(geometry_msgs::PointStamped &revolve_about, double forward_velocity);

    /**
     * @brief Spirals the robot. Used to locate volatiles. NAV_TYPE::SPIRAL. NOT YET IMPLEMENTED
     * 
     * @param goal The goal of the action.
     * @param action_server The action server that this function is operating on.
     */
    void spiralDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    /**
     * @brief Spirals the robot. Used to locate volatiles. NAV_TYPE::FOLLOW. NOT YET IMPLEMENTED
     * 
     * @param goal The goal of the action.
     * @param action_server The action server that this function is operating on.
     */
    void followDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server);

    /**
     * @brief Perform navigation operations.
     *  
     * @param goal The goal of the action. See operations/action/Navigation.action for full definition
     */
    void execute(const operations::NavigationGoalConstPtr &goal);

    /**
     * @brief Removes the goal position and cancels the navigation process.
     * 
     */
    void cancelGoal();

    /**
    * @brief Get the Travel Theta object
    * 	 			 The spiral motion generator requires the 'theta' covered by
    * 	 			 the robot in terms of radian. 
    * 	 			 This theta is incremental, [0, inf]
    * 	 			 If the robot traverses one complete rotation, output will
    * 	 			 be 2PI, and on completition of two rotations, the output
    * 	 			 should be 4PI. This function returns this traveling theta
    * 	 			 Please suggest if you can find a more intuitive name
    *				
    * 
    * @param yaw 	 Current Yaw of the robot with respect to the initial position.
    * @param rotation_counter 		Number of rotations already completed prior to 
    *								this on-going rotation
    * @return double 				Theta travelled
    */
    double getCumulativeTheta(double yaw, int &rotation_counter);
};