/*
Author(s): Jibran, Aditya, Albert
Email: mmuqeetjibran@wpi.edu
TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

NOTE: For control flow of program, refer to joint_state_callback. 
      I would have done the processing and publishing somewhere else, but wanted to minimize lag as much as possible. 
*/


#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <utils/common_names.h>
#include <math.h>
#include <Eigen/Dense>

#define GAZEBO_WHEEL_IMPERFECTION 0                // The wheel might not be perfectly the radius that has been given, so might need to TUNE this, 0.031 is a good value for linear velocity, but not angular
                                                   //  Plus its the method used a lot to compensate for the effects not taken care of by the simple assumptions of pure rolling. 
#define UPDATE_HZ 20
#define ROBOT_WIDTH 0.5
#define ROBOT_LENGTH 0.532
#define WHEEL_RAD   0.17

class EncoderOdom
{
    public: 
        /**
         * @brief Constructor for EncoderOdom objects
         * 
         * @param robot_name define robot name for publishers and subscribers
         * @param update_rate define the update rate of ros spin
         * 
         * @todo: Get rid of gazebo stuff before submission. 
         * 
         */

        
        EncoderOdom(std::string &robot_name, float update_rate) : robot_name_(robot_name), update_rate_(update_rate)
        {
            imu_sub = nh_.subscribe("/" + robot_name_ + "/imu", 1000, &EncoderOdom::robotIMUCallback, this); 
            joint_sub_ = nh_.subscribe("/" + robot_name_ + "/joint_states", 1000, &EncoderOdom::joint_state_callback, this);
            gazebo_sub = nh_.subscribe("/gazebo/model_states", 1000, &EncoderOdom::gazebo_callback, this);
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>(robot_name_ + "/encoder_odom", 1000, true);
            ground_truth_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(robot_name_ + "/ground_truth_velocities", 1000, true);
            
        }

        /**
         * @brief rounding off numbers to 2 decimal places
         * Unrounded values, especially for calculating the rover velocity, result in unstable values while solving Ax = b
         * @param none
         */
        float round(float var)
        {
            float value = (int)(var * 100 + .5);
            return (float)value / 100;
        }

        void robotIMUCallback(const sensor_msgs::Imu &imu_msg)
        {
            long int start = std::clock();

            // Obtain Linear Acceleration Data (All data obtained for IMU corresponds to world -> IMU link frame)
            float lin_acc_x = imu_msg.linear_acceleration.x;
            float lin_acc_y = imu_msg.linear_acceleration.y;
            float lin_acc_z = imu_msg.linear_acceleration.z;
            float angular_z_imu_ = imu_msg.angular_velocity.z;

            // Obtain Orientation in terms of Quaternion
            orient_x_ = imu_msg.orientation.x;
            orient_y_ = imu_msg.orientation.y;
            orient_z_ = imu_msg.orientation.z;
            orient_w_ = imu_msg.orientation.w;

            //Clear the linear velocities before updating
            linear_x_imu_ = 0;
            linear_y_imu_ = 0;
            linear_z_imu_ = 0;

            // Integrate to obtain Linear Velocity Data
            double dt = ( std::clock() - start)/ (double)CLOCKS_PER_SEC * 100;

            // ---------- Declare lin_vel_x & clock-----
            linear_x_imu_ += lin_acc_x * dt;
            linear_y_imu_ += lin_acc_y * dt;
            linear_z_imu_ += lin_acc_z * dt;
        }

        /**
         * @brief Callback to the gazebo/model_states topic (published by gazebo)
         */
        void gazebo_callback(const gazebo_msgs::ModelStates &msg)
        {
            // Finding the index of scout in the model_states.
            std::string str = "small_scout_1";
            for(int i = 0;i<msg.name.size(); i++) {
                if(msg.name.at(i) == str) {
                    scout_index = i;
                }
            }

            // THe ground truth velocities in the map frame. 
            x_dot_map = msg.twist[scout_index].linear.x;                
            y_dot_map = msg.twist[scout_index].linear.y;
            theta_dot = msg.twist[scout_index].angular.z;

            //The orientation of scout wrt the map frame. This will be used to convert the scout velocities from map -> robot frame.
            tf::Quaternion q(
            msg.pose[scout_index].orientation.x,
            msg.pose[scout_index].orientation.y,
            msg.pose[scout_index].orientation.z,
            msg.pose[scout_index].orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            rover_yaw = -yaw;
            
            // x_dot = std::cos(pitch)*std::cos(yaw)*(x_dot_map) - std::cos(pitch)*std::sin(yaw)*(y_dot_map) + std::sin(pitch)*(theta_dot);
            // y_dot = (std::cos(roll)*std::sin(yaw) + std::sin(roll)*std::sin(pitch)*sin(yaw))*(x_dot_map)   + (std::cos(roll)*std::cos(yaw) - std::sin(roll)*std::sin(pitch)*std::sin(yaw))*(y_dot_map) -(std::sin(roll)*std::cos(pitch)*(theta_dot));

            // Given velocities in map frame, convert to robot frame - accounts for the 2D tilt.
            //   | x_dot |  __ | cos(t) -sin(t) | | x_dot_map |  
            //   | y_dot |  -- | sin(t)  cos(t) | | y_dot_map |

            x_dot = (std::cos(rover_yaw) * x_dot_map) - (std::sin(rover_yaw) * y_dot_map); 
            y_dot = (std::sin(rover_yaw) * x_dot_map) + (std::cos(rover_yaw) * y_dot_map);

            ground_truth_velocities_.linear.x = x_dot;
            ground_truth_velocities_.linear.y = y_dot;
            ground_truth_velocities_.angular.z = theta_dot;

            ground_truth_vel_pub_.publish(ground_truth_velocities_);


        }

        /**
         * @brief Function that CONVERTS the wheel velocities and steering angles into robot velocity. Uses the rolling constraints as defined in mobile robot kinematics for steered standard wheels.
         *  
         * 
         * @param GAZEBO_WHEEL_IMPERFECTION
         */

        void robotVelocity() {
            float alpha_fl, alpha_fr, alpha_bl, alpha_br, alpha_abs, beta_bl, beta_br, beta_fl, beta_fr; 
            float radius = 0.17 - GAZEBO_WHEEL_IMPERFECTION;
            float l = std::hypot(ROBOT_WIDTH, ROBOT_LENGTH);                       
            alpha_fl = std::atan2(ROBOT_WIDTH, ROBOT_LENGTH);                                              //Will always remain constant
            alpha_fr = std::atan2(-ROBOT_WIDTH, ROBOT_LENGTH);
            alpha_br = std::atan2(-ROBOT_WIDTH, -ROBOT_LENGTH);
            alpha_bl = std::atan2(ROBOT_WIDTH, -ROBOT_LENGTH);
            alpha_abs = std::abs(alpha_fl);
            float steering_angles[] {fl_steering_joint_pos_, fr_steering_joint_pos_, br_steering_joint_pos_, bl_steering_joint_pos_};  //Steering angles: In clockwise order starting from front-left-wheel, keep all zero for linear velocity case.                                                                                                                              
            beta_fl =  M_PI_2 - alpha_abs + steering_angles[0];                            // beta is the angle between wheel axis and the line connecting steering axis to base_footprint. Be careful about the direction of wheel axis as the signs(+ or -) of the wheel velocities directly affect it. 
            beta_fr =  M_PI_2 + alpha_abs + steering_angles[1];                            // steering_angles is what the callback from sensor_msgs/JointStates is received. It is the angle between the steering frame and base_footprint. 
            beta_br = -M_PI_2 - alpha_abs + steering_angles[2];
            beta_bl = -M_PI_2 + alpha_abs + steering_angles[3];
            //===================== CALCULATION ============================//             ​
            Eigen::MatrixXf A(4, 3);                                                           // Rolling Constraints matrix. The problem is in the form Ax = b, where:
            Eigen::VectorXf b(4);                                                              // A has the rolling constraints, x is the odometry and b is the individual wheel velocities                                                                                    //The solution :  x = [[A]^-1] * [b] 
            A(0, 0) = std::sin(alpha_fl + beta_fl);                                      
            A(0, 1) = -std::cos(alpha_fl + beta_fl);
            A(0, 2) = -l * std::cos(beta_fl);
            A(1, 0) = std::sin(alpha_fr + beta_fr);
            A(1, 1) = -std::cos(alpha_fr + beta_fr);
            A(1, 2) = -l * std::cos(beta_fr);
            A(2, 0) = std::sin(alpha_br + beta_br);
            A(2, 1) = -std::cos(alpha_br + beta_br);
            A(2, 2) = -l * std::cos(beta_br);
            A(3, 0) = std::sin(alpha_bl + beta_bl);
            A(3, 1) = -std::cos(alpha_bl + beta_bl);
            A(3, 2) = -l * std::cos(beta_bl);
            b << radius*fl_wheel_joint_vel_, radius*fr_wheel_joint_vel_, radius*br_wheel_joint_vel_, radius*bl_wheel_joint_vel_; 
            // std::cout << "Here is the matrix A:\n" << A << std::endl;
            // std::cout << "Here is th0 vector b:\n" << b << std::endl;
            Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);                                // Matrix of velocities [ x_dot; y_dot; theta_dot ]

            // std::cout << "The solution is:\n" << x << std::endl;           // Refer to https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html for more solver methods

            // Instantaneous Velocities of robot in robot frame. Using the IMU-roll and pitch, the horizontal components of these velocities are taken as the robot is assumed to be in a 2D world.          
            linear_x_  = x(0);
            linear_y_  = x(1);
            angular_z_ = x(2);
        }


        /**
         * @brief Function that receives the wheel velocities and steering angles. Uses the rolling constarints as defined in mobile robot kinematics for steered standard wheels to calculate robot velocity. 
         * Refer to: http://www.cs.cmu.edu/~rasc/Download/AMRobots3.pdf 
         * @param GAZEBO_WHEEL_IMPERFECTION, 
         */
        void robotVelocity2D() {

            // Convert IMU orientation from Quaternion to Rotation matrix
            tf::Quaternion q(orient_x_,orient_y_,orient_z_,orient_w_);
            tf::Matrix3x3 m(q);

            //Get the roll-pitch-yaw of above quaternion. 
            double roll_imu, pitch_imu, yaw_imu;
            m.getRPY(roll_imu, pitch_imu, yaw_imu);

            //Set the yaw to zero and pass the roll-pitch to construct the rotation matrix. This is done to account for the roll-pitch changes encountered in craters. 
            // If yaw is taken into account, the velocity ends up being projected in world frame. We want it in the robot frame. 
            tf::Quaternion q_imu;
            q_imu.setRPY( roll_imu, pitch_imu, 0.0 ); 
            // ROS_INFO_STREAM("IMU Quaternion : " << q_imu[0] << " " << q_imu[1] << " " << q_imu[2] << " " << q_imu[3]); 
            // ROS_INFO_STREAM("ROLL: " << roll_imu << " PITCH: " << pitch_imu);
            
            tf::Matrix3x3 m2;
            m2.setRotation(q_imu);
            // tf::Matrix3x3 m2;
            // m2 = m3.inverse();

            // -------------- Rotation Matrix to convert from frame i(robot) to i+1(flat frame) FOR REFERENCE ------------------
            //     R(α, β, γ) = 	
                
            // |cos(β)cos(γ)	                        −cos(β)sin(γ)	                           sin(β) | |x_dot_robot|
            // |    cos(α)sin(γ) + sin(α)sin(β)cos(γ)	cos(α)cos(γ) − sin(α)sin(β)sin(γ)	−sin(α)cos(β) | |y_dot_robot|
            // |    sin(α)sin(γ) − cos(α)sin(β)cos(γ)	sin(α)cos(γ) + cos(α)sin(β)sin(γ)	cos(α)cos(β)  | |theta_dot_robot|
            
            //The rows of the rotation matrix.            
            tf::Vector3 first_row = m2.getRow(0);
            tf::Vector3 second_row = m2.getRow(1);
            tf::Vector3 third_row = m2.getRow(2);

            //The rotation matrix being multiplied with the linear velocities of rover to get the X-Y velocities in the horizontal frame (z-axis perpendicular to ground)
            
            linear_x_flat_ = (first_row[0] * linear_x_) + (first_row[1] * linear_y_) + (first_row[2] * linear_z_imu_);
            linear_y_flat_ = (second_row[0] * linear_x_ ) + (second_row[1] * linear_y_ ) + (second_row[2] * linear_z_imu_);
            angular_z_flat_ = angular_z_;    //Imagination tells that they should both be the same. Only the circle traced by the vector differs in both planes. 

            // DEBUG STATEMENTS
            // ROS_INFO("The rotation matrix is : ");
            // ROS_INFO_STREAM(*first_row << " " << *(first_row + 1) << " " << *(first_row + 2));
            // ROS_INFO_STREAM(*second_row << " " << *(second_row + 1) << " " << *(second_row + 2) );
            // ROS_INFO_STREAM(*third_row << " " << *(third_row + 1) << " " << *(third_row + 2) );

        }

        /**
         * @brief   Publishes odometry after all velocity calculations are done. 
         * 
         * @param:  round/no round on velocities
         */
        void robotVelocityPublisher() {

            // DEBUG STATEMENTS
            // std::cout << "The linear velocity of IMU in z direction is: " << linear_z_imu_ << std::endl;
            // std::cout << "The ground truth (in robot frame) is:\n "<< x_dot << " " << y_dot << " " << theta_dot << std::endl;
            // std::cout << "The robot encoder velocity before imu transformation are:\n "<< linear_x_<< " " << linear_y_ << " " << angular_z_ << std::endl;
            // std::cout << "The robot encoder velocity is:\n "<< linear_x_flat_ << " " << linear_y_flat_ << " " << angular_z_ << std::endl;

            output_odom_.twist.twist.linear.x = round(linear_x_flat_);
            output_odom_.twist.twist.linear.y = round(linear_y_flat_);
            output_odom_.twist.twist.linear.z = 0.0;
            output_odom_.twist.twist.angular.x = 0.0;
            output_odom_.twist.twist.angular.y = 0.0;
            output_odom_.twist.twist.angular.z = round(angular_z_flat_);
            output_odom_.header = header_;
            output_odom_.child_frame_id = header_.frame_id; 
            output_odom_.header.frame_id = "odom";
            odom_pub_.publish(output_odom_);
        }

        /**
         * @brief Store the joint data, since there's so many joints and the names are huge, a seperate function looks cleaner and makes the flow more readable. 
         */
        
        void storeJointsData(const sensor_msgs::JointState::ConstPtr &joint_data) {
            // Header (used in republishing)
            header_ = joint_data->header;

            // positions
            bl_steering_joint_pos_ = round(joint_data->position[4]);
            br_steering_joint_pos_ = round(joint_data->position[5]);
            fl_steering_joint_pos_ = round(joint_data->position[6]);
            fr_steering_joint_pos_ = round(joint_data->position[7]);

            // velocities
            bl_wheel_joint_vel_ = round(joint_data->velocity[0]);
            br_wheel_joint_vel_ = round(joint_data->velocity[1]);
            fl_wheel_joint_vel_ = round(joint_data->velocity[2]);
            fr_wheel_joint_vel_ = round(joint_data->velocity[3]);
        }

        /**
         * @brief Callback to the robot joint states, and has the complete flow and final publishing of odometry. 
         * 
         * @param joints JointStates message indicating the encoder values of the rover
         */

        void joint_state_callback(const sensor_msgs::JointState::ConstPtr &joints)
        {
            /** TODO: Make sure mutexes are used wherever necessary */
            // const std::lock_guard<std::mutex> lock(encoder_odom_mutex_);

            //Copying the shared pointed of joints for transferring to storeJointsData function. Not necessary, but results in clean code.
            sensor_msgs::JointState::ConstPtr joint_data{joints};
            storeJointsData(joint_data);

            //Calculate the instantaneous robot velocity in its frame
            robotVelocity();

            //Convert the velocites into the flat 2D world using IMU orientation, ensure that IMU data is latest and without significant lag. 
            robotVelocity2D(); 
            
            //The final publishing of odometry. 
            robotVelocityPublisher();
        }

        std::vector<double> debug_x_;



    protected:
        ros::NodeHandle nh_;
        std::string robot_name_;
        /** @todo: update_rate should be defined in construct */
        ros::Rate update_rate_;
        // subscribe to joint states to obtain rover encoder data
        ros::Subscriber joint_sub_; 
        // subscribe to the navigation type being used by nav server in order to correctly calculate velocities
        ros::Subscriber nav_type_sub_, gazebo_sub; 
        // subscribe to the velocity from imu
        ros::Subscriber imu_sub;

        // output odometry message
        nav_msgs::Odometry output_odom_;
        // output odometry publisher
        ros::Publisher odom_pub_, ground_truth_vel_pub_;  
        std::mutex encoder_odom_mutex_;\
        geometry_msgs::Twist ground_truth_velocities_;

        // Header
        std_msgs::Header header_;
        // positions (instantiated here to not clog up constructor)
        float bl_steering_joint_pos_{0.0};
        float br_steering_joint_pos_{0.0};
        float fl_steering_joint_pos_{0.0};
        float fr_steering_joint_pos_{0.0};
        // velocities (instantiated here to not clog up constructor)
        float bl_wheel_joint_vel_{0.0};
        float br_wheel_joint_vel_{0.0};
        float fl_wheel_joint_vel_{0.0};
        float fr_wheel_joint_vel_{0.0};

        float linear_x_, linear_y_, angular_z_;
        float linear_x_imu_, linear_y_imu_, linear_z_imu_, angular_z_imu_;
        float linear_x_flat_, linear_y_flat_, angular_z_flat_;
        float orient_x_, orient_y_, orient_z_, orient_w_;

        /** gazebo attributes TODO: Get rid of these before submission, only used for testing */ 
        int scout_index;
        float x_dot, y_dot, theta_dot;
        float rover_yaw, x_dot_map, y_dot_map;    

};

int main(int argc, char *argv[])
{
    std::string name(argv[1]);
    ros::init(argc, argv, name + "_encoders");
    EncoderOdom encoder_odom(name, 10.0);
    while(ros::ok())
    {
        ros::spinOnce();
    }
}
