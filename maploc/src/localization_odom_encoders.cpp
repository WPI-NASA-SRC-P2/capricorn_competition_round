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

// nav_msgs::Odometry last_odom;
// int seq = 0;

// RobotStateInformer::RobotDriveMode mode = RobotStateInformer::RobotDriveMode::SKID;

// RobotStateInformer *state_informer;

// std::string robot_name;

// void drive_mode_callback(const std_msgs::Int8::ConstPtr& new_mode)
// {
//     std::cout << "Mode: " << *new_mode << std::endl;

//     mode = (RobotStateInformer::RobotDriveMode) new_mode->data;
// }

// nav_msgs::Odometry skid_odom(const sensor_msgs::JointState::ConstPtr &joints)
// {
//     float vl, vr;

//     nav_msgs::Odometry odom;

//     if(robot_name == CAPRICORN_COMMON_NAMES::HAULER + "1")
//     {
//         vl = joints->velocity[9] * WHEEL_RAD; //Front left velocity
//         vr = joints->velocity[12] * WHEEL_RAD; //Front right velocity
//     }
//     else if(robot_name == CAPRICORN_COMMON_NAMES::EXCAVATOR + "1")
//     {
//         vl = joints->velocity[11] * WHEEL_RAD;
//         vr = joints->velocity[14] * WHEEL_RAD;
//     }
//     else if(robot_name == CAPRICORN_COMMON_NAMES::SCOUT + "1")
//     {
//         vl = joints->velocity[8] * WHEEL_RAD;
//         vr = joints->velocity[11] * WHEEL_RAD;
//     }

//     float omega = (vr - vl)/(2*ROBOT_WIDTH);

//     float x_vel = (vr + vl) / 2;

//     odom.twist.twist.linear.x = x_vel;
//     odom.twist.twist.linear.y = 0;
//     odom.twist.twist.angular.z = omega;

//     return odom;
// }

// nav_msgs::Odometry crab_odom(const sensor_msgs::JointState::ConstPtr &joints)
// {
//     std::cout << "crab" << std::endl;
//     float vl, vr, angleLeft, angleRight;

//     nav_msgs::Odometry odom;


//     if(robot_name == CAPRICORN_COMMON_NAMES::HAULER + "1")
//     {
//         vl = joints->velocity[9] * WHEEL_RAD; //Front left velocity
//         vr = joints->velocity[12] * WHEEL_RAD; //Front right velocity
//         angleLeft = joints->position[8];
//         angleRight = joints->position[11];
//     }
//     else if(robot_name == CAPRICORN_COMMON_NAMES::EXCAVATOR + "1")
//     {
//         vl = joints->velocity[11] * WHEEL_RAD;
//         vr = joints->velocity[14] * WHEEL_RAD;
//         angleLeft = joints->position[10];
//         angleRight = joints->position[13];
//     }
//     else if(robot_name == CAPRICORN_COMMON_NAMES::SCOUT + "1")
//     {
//         vl = joints->velocity[8] * WHEEL_RAD;
//         vr = joints->velocity[11] * WHEEL_RAD;
//         angleLeft = joints->position[7];
//         angleRight = joints->position[10];
//     }

//     float velocity = (vr + vl) / 2;
//     float angle = (angleLeft + angleRight) / 2;

//     odom.twist.twist.linear.x = cos(angle) * velocity;
//     odom.twist.twist.linear.y = sin(angle) * velocity;

//     odom.twist.twist.angular.z = 0;

//     return odom;
// }
// nav_msgs::Odometry radial_odom(const sensor_msgs::JointState::ConstPtr &joints)
// {
//     float vl, vr, angleLeft, angleRight;

//     nav_msgs::Odometry odom;

//     //This comment block is the closest to correct. Maybe come back to this at some point
//     /*vl = state_informer->getJointVelocity(CAPRICORN_COMMON_NAMES::FRONT_LEFT_PREFIX + CAPRICORN_COMMON_NAMES::WHEEL_JOINT);
//     vr = state_informer->getJointVelocity(CAPRICORN_COMMON_NAMES::FRONT_RIGHT_PREFIX + CAPRICORN_COMMON_NAMES::WHEEL_JOINT);

//     angleLeft = state_informer->getJointPosition(CAPRICORN_COMMON_NAMES::FRONT_LEFT_PREFIX + CAPRICORN_COMMON_NAMES::STEERING_ARM_JOINT);
//     angleRight = state_informer->getJointPosition(CAPRICORN_COMMON_NAMES::FRONT_RIGHT_PREFIX + CAPRICORN_COMMON_NAMES::STEERING_ARM_JOINT);

//     std::cout << vl << "\t\t" << vr << std::endl;*/


//     if(robot_name == CAPRICORN_COMMON_NAMES::HAULER + "1")
//     {
//         vl = joints->velocity[9] * WHEEL_RAD; //Front left velocity
//         vr = joints->velocity[12] * WHEEL_RAD; //Front right velocity
//         angleLeft = joints->position[8];
//         angleRight = joints->position[11];
//     }
//     else if(robot_name == CAPRICORN_COMMON_NAMES::EXCAVATOR + "1")
//     {
//         vl = joints->velocity[11] * WHEEL_RAD;
//         vr = joints->velocity[14] * WHEEL_RAD;
//         angleLeft = joints->position[10];
//         angleRight = joints->position[13];
//     }
//     else if(robot_name == CAPRICORN_COMMON_NAMES::SCOUT + "1")
//     {
//         vl = joints->velocity[8] * WHEEL_RAD;
//         vr = joints->velocity[11] * WHEEL_RAD;
//         angleLeft = joints->position[7];
//         angleRight = joints->position[10];
//     }

//     float robot_radius = ((1 + tan(angleLeft)) * (ROBOT_WIDTH/2)) / tan(angleLeft);

//     float outer_radius = robot_radius + (ROBOT_WIDTH/2);
//     float inner_radius = robot_radius - (ROBOT_WIDTH/2);

//     float omega_outer = vr / outer_radius;
//     float omega_inner = vl / inner_radius;

//     float omega = (omega_outer + omega_inner) / 2;
//     float velocity = omega * robot_radius;

//     odom.twist.twist.linear.x = velocity;
//     odom.twist.twist.linear.y = 0;
//     odom.twist.twist.angular.z = omega;

//     return odom;
// }

/*
 * Calculated the angular velocity that a wheel generates.
 * @param wheel_vel wheel velocity
 * @param wheel_angle the angle of the wheel from vertical, clockwise +
 * @param robot_radius distance from center of chassis to wheel
 * @param chassis_angle which internal chassis angle to consider to calculate cross product
 */
double calc_angular_velocity(double wheel_vel, double wheel_angle, double robot_radius, double chassis_angle)
{
    //We want the angle of the wheel from horizontal, plus the providec chassis angle
    double theta = (90 - wheel_angle) + chassis_angle;

    //Calculate and return the magnitude of the cross product
    return wheel_vel * robot_radius * sin(theta);
}

// nav_msgs::Odometry swerve_odom(const sensor_msgs::JointState::ConstPtr &joints)
// {
//     //Order of wheels is arbitrary, as we are doing a simple average.
//     //What does matter is that the order is the same across both arrays
//     double velocities[4];
//     double angles[4];

//     nav_msgs::Odometry odom;

//     if(robot_name == CAPRICORN_COMMON_NAMES::HAULER + "1")
//     {
//         velocities[0] = joints->velocity[9] * WHEEL_RAD; //Front left velocity
//         velocities[1] = joints->velocity[12] * WHEEL_RAD; //Front right velocity
//         velocities[2] = joints->velocity[6] * WHEEL_RAD; //Back right
//         velocities[3] = joints->velocity[3] * WHEEL_RAD;
//         angles[0] = joints->position[8];
//         angles[1] = joints->position[11];
//         angles[2] = joints->position[5];:_M_construct null not valid
// [8] * WHEEL_RAD; //Back right
//         velocities[3] = joints->velocity[5] * WHEEL_RAD;
//         angles[0] = joints->position[10];
//         angles[1] = joints->position[13];
//         angles[2] = joints->position[7];
//         angles[3] = joints->position[4];
//     }
//     else if(robot_name == CAPRICORN_COMMON_NAMES::SCOUT + "1")
//     {
//         velocities[0] = joints->velocity[8] * WHEEL_RAD;
//         velocities[1] = joints->velocity[11] * WHEEL_RAD;
//         velocities[2] = joints->velocity[5] * WHEEL_RAD; //Back right
//         velocities[3] = joints->velocity[2] * WHEEL_RAD;
//         angles[0] = joints->position[7];
//         angles[1] = joints->position[10];
//         angles[2] = joints->position[4];
//         angles[3] = joints->position[1];
//     }

//     double total_x = 0;
//     double total_y = 0;
//     double total_angular = 0;

//     //We perform a simple average of each wheel's x and y component to determine the overall xy linear velocity.
//     total_x += cos(angles[0]) * velocities[0];
//     total_x += cos(angles[1]) * velocities[1];
//     total_x += cos(angles[2]) * velocities[2];
//     total_x += cos(angles[3]) * velocities[3];

//     //Do a simple average
//     double x_vel = total_x / 4.0;

//     total_y += sin(angles[0]) * velocities[0];
//     total_y += sin(angles[1]) * velocities[1];
//     total_y += sin(angles[2]) * velocities[2];
//     total_y += sin(angles[3]) * velocities[3];

//     double y_vel = total_y / 4.0;

//     double L = RobotDescription::WHEEL_SEPERATION_LENGTH;
//     double W = RobotDescription::WHEEL_SEPERATION_WIDTH;

//     double radius = sqrt(pow(L, 2) + pow(W, 2));

//     double internal_angle_1 = atan2(L, W);
//     double internal_angle_2 = 90 - internal_angle_1;

//     //We do the same for angular velocities, but this math is a little more complicated. In essence, we perform
//     //a cross product between the wheel force vector and the position vector from the robot center to the wheel.
//     //This will give us a torque about the center of the robot.
//     total_angular += calc_angular_velocity(velocities[0], angles[0], radius, internal_angle_1);
//     total_angular += calc_angular_velocity(velocities[1], angles[1], radius, internal_angle_2);
//     total_angular -= calc_angular_velocity(velocities[2], angles[2], radius, internal_angle_1);
//     total_angular -= calc_angular_velocity(velocities[3], angles[3], radius, internal_angle_2);

//     double omega = total_angular / 4.0;

//     odom.twist.twist.linear.x = x_vel;
//     odom.twist.twist.linear.y = y_vel;
//     odom.twist.twist.angular.z = omega;

//     return odom;
// }

class EncoderOdom
{
    public: 
      EncoderOdom::EncoderOdom(const std::string &robot_name, float update_rate) : robot_name_(robot_name), update_rate_(float update_rate) {}
    
        EncoderOdom::joint_state_callback(const sensor_msgs::JointState::ConstPtr &joints)
        {
            /*
            joints->name[] (string)
            joints->position[]
            joints->velocity[]
            joints->effort[]
            */
            // float bl_wheel_joint = joints->
            ROS_INFO_STREAM(*(joints));
            /*
            switch(mode)
            {
                case RobotStateInformer::RobotDriveMode::STRAIGHT:
                case RobotStateInformer::RobotDriveMode::CRAB_DRIVE:
                    last_odom = crab_odom(joints);
                    break;
                case RobotStateInformer::RobotDriveMode::SKID:
                    last_odom = skid_odom(joints);
                    break;
                case RobotStateInformer::RobotDriveMode::ROTATE_ROBOT:
                case RobotStateInformer::RobotDriveMode::RADIAL:
                    last_odom = radial_odom(joints);
                    break;
                case RobotStateInformer::RobotDriveMode::SWERVE:
                    last_odom = swerve_odom(joints);
            }

            last_odom.header = joints->header;
            last_odom.header.seq = seq;
            seq++;

            last_odom.child_frame_id = (robot_name + "_tf/base_footprint").c_str();
            last_odom.header.frame_id = (robot_name + "_tf/base_footprint").c_str();
            */
        }

    protected:
        ros::NodeHandle nh_;
        std::string robot_name_;
        ros::Rate update_rate(UPDATE_HZ);
        ros::Subscriber joint_sub_ = nh.subscribe("/" + robot_name_ + "/joint_states", 1000, joint_state_callback);
        ros::Subscriber nav_type_sub_ = nh.subscribe("/" + robot_name_ + COMMON_NAMES::NAV_TYPE_TOPIC)
        // Header
        std_msgs::Header header;
        // Nav type (defined in common_names under the enum NAV_TYPE::NAVNAME)
        int nav_type; 
        // positions
        float bl_wheel_joint_pos{0.0};
        float br_wheel_joint_pos{0.0};
        float fl_wheel_joint_pos{0.0};
        float fr_wheel_joint_pos{0.0};
        float bl_steering_joint_pos{0.0};
        float br_steering_joint_pos{0.0};
        float fl_steering_joint_pos{0.0};
        float fr_steering_joint_pos{0.0};
        // velocities
        float bl_wheel_joint_vel{0.0};
        float br_wheel_joint_vel{0.0};
        float fl_wheel_joint_vel{0.0};
        float fr_wheel_joint_vel{0.0};
        float bl_steering_joint_vel{0.0};
        float br_steering_joint_vel{0.0};
        float fl_steering_joint_vel{0.0};
        float fr_steering_joint_vel{0.0};

        
        
        
};

/*
 * Given a joints pointer from ROS (sensor:msgs::JointState), calculates encoder odometry in the chassis frame.
 * Assumes skid steering. Calculates yaw velocity and x velocity.
*/

int main(int argc, char *argv[])
{
    EncoderOdom encoder_odom(name, 10.0);

}

    // //Setup robot_name from passed in arguments
    // std::string name(argv[1]);
    // // std::string number(argv[2]);

    // std::string robot_name = name;

    // //Startup ROS
    // ros::init(argc, argv, "capricorn_odom_encoders");

    // ros::NodeHandle nh;
    // ros::Rate update_rate(UPDATE_HZ);

    // //state_informer = new RobotStateInformer(nh, name, 1);

    // //Publish on encoder_odom
    // // ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/capricorn/" + robot_name + "/odom/encoder_odom", 1, true);

    // //Subscribe to our robot's joint_states
    // ros::Subscriber joint_sub = nh.subscribe("/" + robot_name + "/joint_states", 1000, joint_state_callback);
    // std::cout << "Subscribed to joint states" << std::endl;

    // // ros::Subscriber mode_sub = nh.subscribe("/capricorn/" + robot_name + "/nav/drive_mode", 1000, drive_mode_callback);
    // // std::cout << "Subscribed to drive mode" << std::endl;

    // std::cout << "Spinning..." << std::endl;

    // //While this node is running, publish any new encoder odometry we've calculated
    // while(ros::ok())
    // {
    //     // odom_pub.publish(last_odom);

    //     ros::spinOnce();
    //     update_rate.sleep();
    // }

    // std::cout << "Shouldn't reach here" << std::endl;