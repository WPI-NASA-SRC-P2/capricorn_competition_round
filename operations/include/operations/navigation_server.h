#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <planning/TrajectoryWithVelocities.h>
#include <planning/trajectory.h>
#include <nav_msgs/Odometry.h>
#include <srcp2_msgs/BrakeRoverSrv.h>

#include <operations/navigation_algorithm.h>
#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

// Used to visualize poses in an array
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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
    bool initial_turn_completed = false;

    // Tolerances for linear and angular moves
    const float DIST_EPSILON = 0.5;
    const float ANGLE_EPSILON = 0.1;
    float c_dist_epsilon_ = DIST_EPSILON;

    // Delta heading limits for smooth drive
    const float HALF_VIEWING = M_PI/6;
    const float MAX_TURNING_RAD = HALF_VIEWING - ANGLE_EPSILON;
    const float MIN_TURNING_RAD = -HALF_VIEWING + ANGLE_EPSILON;

    // Default speeds for straight lines and turn in place (linear wheel velocity in m/s)
    const float BASE_DRIVE_SPEED = 0.6;
    const float BASE_SPIN_SPEED = 0.3;

    // How far the robot should travel before it asks for a new trajectory, in meters. Used in automaticDriving.
    const double LARGE_TRAJECTORY_REST_DIST = 5; 
    const double SMALL_TRAJECTORY_REST_DIST = 3; 
    double trajectory_reset_dist = SMALL_TRAJECTORY_REST_DIST;

    std::string robot_name_;

    // The actionlib server
    Server *server_;

    // Rate limiter
    ros::Rate *update_rate_;

    // Publishers for each wheel velocity and steering controller
    ros::Publisher front_left_vel_pub_, front_right_vel_pub_, back_left_vel_pub_, back_right_vel_pub_;
    ros::Publisher front_left_steer_pub_, front_right_steer_pub_, back_left_steer_pub_, back_right_steer_pub_;

    // Debug publisher. Used to visualize poses in gazebo with utils/render_poses.py
    ros::Publisher waypoint_pub_;

    // Used to get the current robot pose
    ros::Subscriber update_current_robot_pose_;

    // Triggered when a plan reset is requested
    ros::Subscriber replan_sub_;

    ros::ServiceClient brake_client_;

    ros::ServiceClient trajectory_client_;
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

    // How much distance the robot has traveled since the last planner call. Compared against TRAJECTORY_RESET_DIST.
    double total_distance_traveled_ = 0;

    // Whether we should get a new trajectory from the planner. Set by driveDistance in automaticDriving.
    bool get_new_trajectory_ = false;

    /**
     * @brief Uses waypoint_pub_ to publish poses to be visualized in Gazebo.
     * 
     * @param waypoints 
     */
    void publishWaypoints(std::vector<geometry_msgs::PoseStamped> waypoints);

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

    geometry_msgs::PoseStamped getRobotPose();

    /**
     * @brief Callback from the planer to replan.
     * 
     * @param msg Empty message. Whenever its received, we reset the trajectory
     */
    void replanCB(const std_msgs::Bool::ConstPtr &msg);

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
    * @brief Steers the robot wheels for the angles.
    * 
    * @param angle Angle at which all robot wheels will be steered.
    */
    void steerRobot(const double angle);

    /**
    * @brief Move robot wheels with the given velocities.
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
    * @brief Move robot wheels with the given velocity .
    * 
    * @param velocity velocity for the wheels.
    */
    void moveRobotWheels(const double velocity);

    /**
     * @brief Sends a goal received from the Robot SM to the planner. Receives and returns the trajectory
     * 
     * @param goal The end goal for the robot to go to
     * @return planning::TrajectoryWithVelocities* 
     */
    planning::TrajectoryWithVelocities sendGoalToPlanner(const geometry_msgs::PoseStamped& goal);

    /**
     * @brief Used to transform each pose in the trajectory into the map frame.
     * 
     * @param traj The trajectory to transform the poses of
     * @return planning::TrajectoryWithVelocities The resulting trajectory
     */
    planning::TrajectoryWithVelocities getTrajInMapFrame(const planning::TrajectoryWithVelocities& traj);

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
     * @brief Used to request a new trajectory after the robot has performed the inital turn upon receiving new goal (Helps with obstacle detection) OR to handle replan requests from planner.
     * @param replan_request Used to indicate whether the new trajectory request was called for an initial turn double traj check or for replan request for dynamic planning.
     */
    void requestNewTrajectory(bool replan_request = false);

    /**
     * @brief Execute a smooth drive to a waypoint. WARNING: Will keep moving after termination.
     * 
     * @param waypoint The waypoint to drive the robot to.
     * @param future_waypoint The next waypoint to drive the robot to.
     * @return true The robot successfully made it to the waypoint.
     * @return false The robot did not, either because it couldn't or because of an interrupt.
     */
    bool smoothDriving(const geometry_msgs::PoseStamped waypoint, const geometry_msgs::PoseStamped future_waypoint);

    /**
     * @brief Drives to a goal, based on waypoints generated by the local planner. NAV_TYPE::GOAL
     * 
     * @param goal The goal of the action. Uses the PoseStamped member to pass to the planner.
     * @param action_server The action server that this function is operating on.
     * @param smooth Whether to follow the trajectory smoothly or do turn in places.
     */
    void automaticDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server, bool smooth);

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
};