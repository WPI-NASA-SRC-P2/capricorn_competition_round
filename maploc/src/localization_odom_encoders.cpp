#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// #include <capricorn_common/robot_state.h>

// #include <capricorn_common/robot_description.h>

#include <utils/common_names.h>

#include <math.h>

#define UPDATE_HZ 20
#define ROBOT_WIDTH RobotDescription::WHEEL_SEPERATION_WIDTH
#define WHEEL_RAD   RobotDescription::WHEEL_RADIUS

class EncoderOdom
{
    public: 
        /**
         * @brief Constructor for EncoderOdom objects
         * 
         * @param robot_name define robot name for publishers and subscribers
         * @param update_rate define the update rate of ros spin
         * 
         * @todo: ENSURE THIS CONSTRUCTOR ALIGNS WITH THE PROTECTED FIELDS ETC
         * 
         */
        EncoderOdom(std::string &robot_name, float update_rate) : robot_name_(robot_name), update_rate_(update_rate)
        {
            joint_sub_ = nh_.subscribe("/" + robot_name_ + "/joint_states", 1000, &EncoderOdom::joint_state_callback, this);
            // the nav_type should be published on /capricorn/rover_name/nav_type_topic in nav server
            nav_type_sub_ = nh_.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + COMMON_NAMES::NAV_TYPE_TOPIC, 1, &EncoderOdom::nav_type_callback, this);
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>(COMMON_NAMES::CAPRICORN_TOPIC + robot_name_ + "/odom/encoder_odom", 1, true);
        }
    
        /**
         * @brief Callback to the nav_type topic (published by navigation stack)
         * 
         * @param received_nav_type integer representing the navigation type
         */
        void nav_type_callback(const std_msgs::Int8::ConstPtr &received_nav_type)
        {
            nav_type_ = received_nav_type->data;
        }

        /**
         * @brief Callback to the robot joint states topic (published by NASA), store them as class variables
         * 
         * @param joints JointStates message indicating the encoder values of the rover
         */
        void joint_state_callback(const sensor_msgs::JointState::ConstPtr &joints)
        {
            // Header (used in republishing)
            header_ = joints->header;
            // positions
            bl_wheel_joint_pos_ = joints->position[0];
            br_wheel_joint_pos_ = joints->position[1];
            fl_wheel_joint_pos_ = joints->position[2];
            fr_wheel_joint_pos_ = joints->position[3];
            bl_steering_joint_pos_ = joints->position[4];
            br_steering_joint_pos_ = joints->position[5];
            fl_steering_joint_pos_ = joints->position[6];
            fr_steering_joint_pos_ = joints->position[7];
            // velocities
            bl_wheel_joint_vel_ = joints->velocity[0];
            br_wheel_joint_vel_ = joints->velocity[1];
            fl_wheel_joint_vel_ = joints->velocity[2];
            fr_wheel_joint_vel_ = joints->velocity[3];
            bl_steering_joint_vel_ = joints->velocity[4];
            br_steering_joint_vel_ = joints->velocity[5];
            fl_steering_joint_vel_ = joints->velocity[6];
            fr_steering_joint_vel_ = joints->velocity[7];

            // unsure where best location to put this switch statement is
            switch(nav_type_)
            {
                case COMMON_NAMES::NAV_TYPE::MANUAL:
                    // Manual driving
                    manual_odom();
                    break;
                case COMMON_NAMES::NAV_TYPE::GOAL:
                    // Trajectory generation with the planner from a goal
                    goal_odom();
                    break;
                case COMMON_NAMES::NAV_TYPE::REVOLVE:
                    // Revolve the robot around a fixed point
                    revolve_odom();
                    break;
                case COMMON_NAMES::NAV_TYPE::SPIRAL:
                    // Archimedean spiral (scout finding volatiles)
                    spiral_odom();
                    break;
                case COMMON_NAMES::NAV_TYPE::FOLLOW:
                    // Follow an object in frame
                    follow_odom();
                    break;
            }
            // unsure what these do/are for
            // last_odom.header = joints->header;
            // last_odom.header.seq = seq;
            // seq++;

            // last_odom.child_frame_id = (robot_name + "_tf/base_footprint").c_str();
            // last_odom.header.frame_id = (robot_name + "_tf/base_footprint").c_str();
            
        }
        /**
         * @brief calculate the encoder odometry based on the manual navigation type
         * 
         */
        void manual_odom()
        {
            // output_odom_ = 
        }

        /**
         * @brief calculate the encoder odometry based on the goal navigation type
         * 
         */
        void goal_odom()
        {
            // output_odom_ = 
        }

        /**
         * @brief calculate the encoder odometry based on the revolve navigation type
         * 
         */
        void revolve_odom()
        {
            // output_odom_ = 
        }

        /**
         * @brief calculate the encoder odometry based on the spiral navigation type
         * 
         */
        void spiral_odom()
        {
            // output_odom_ = 
        }

        /**
         * @brief calculate the encoder odometry based on the follow navigation type
         * 
         */
        void follow_odom()
        {
            // output_odom_ = 
        }

    protected:
        ros::NodeHandle nh_;
        std::string robot_name_;
        /** @todo: update_rate should be defined in construct */
        ros::Rate update_rate_;
        // subscribe to joint states to obtain rover encoder data
        ros::Subscriber joint_sub_; 
        // subscribe to the navigation type being used by nav server in order to correctly calculate velocities
        ros::Subscriber nav_type_sub_; 
        // Nav type (defined in common_names under the enum NAV_TYPE::NAVNAME)
        int nav_type_; 
        // Header
        std_msgs::Header header_;
        // positions (instantiated here to not clog up constructor)
        float bl_wheel_joint_pos_{0.0};
        float br_wheel_joint_pos_{0.0};
        float fl_wheel_joint_pos_{0.0};
        float fr_wheel_joint_pos_{0.0};
        float bl_steering_joint_pos_{0.0};
        float br_steering_joint_pos_{0.0};
        float fl_steering_joint_pos_{0.0};
        float fr_steering_joint_pos_{0.0};
        // velocities (instantiated here to not clog up constructor)
        float bl_wheel_joint_vel_{0.0};
        float br_wheel_joint_vel_{0.0};
        float fl_wheel_joint_vel_{0.0};
        float fr_wheel_joint_vel_{0.0};
        float bl_steering_joint_vel_{0.0};
        float br_steering_joint_vel_{0.0};
        float fl_steering_joint_vel_{0.0};
        float fr_steering_joint_vel_{0.0};
        // output odometry message
        nav_msgs::Odometry output_odom_;
        // output odometry publisher
        ros::Publisher odom_pub_;   
};

int main(int argc, char *argv[])
{
    std::string name(argv[1]);
    EncoderOdom encoder_odom(name, 10.0);

    // //While this node is running, publish any new encoder odometry we've calculated
    while(ros::ok())
    {
        // odom_pub.publish(last_odom);

        ros::spinOnce();
        /** @todo: FIX UPDATE RATE STUFF */
        // update_rate.sleep();
    }
}
