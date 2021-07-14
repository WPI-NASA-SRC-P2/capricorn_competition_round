/*
Author: Chris DeMaio
Email: cjdemaio@wpi.edu
TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

// Include statements
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <utils/common_names.h>

// Global variable to hold the unfiltered data as it comes from the imu
sensor_msgs::Imu imu_unfiltered;

// Variable to track whether we have received a message
bool imu_message_received = false;

// Keep track of how many measurements we've made so we can keep track of avgs
int reading_count = 0;
int prev_reading_count = 0;

// An adjustable parameter for the filter
// MUST BE BETWEEN 0 AND 1
double filter_factor = 0.2;
const int array_size = 20;

// Variables to hold avgs
double orientation_x_array[array_size];
double orientation_y_array[array_size];
double orientation_z_array[array_size];
double orientation_w_array[array_size];
double angular_velocity_x_array[array_size];
double angular_velocity_y_array[array_size];
double angular_velocity_z_array[array_size];
double linear_acceleration_x_array[array_size];
double linear_acceleration_y_array[array_size];
double linear_acceleration_z_array[array_size];

double orientation_x_avg = 0.0;
double orientation_y_avg = 0.0;
double orientation_z_avg = 0.0;
double orientation_w_avg = 0.0;
double angular_velocity_x_avg = 0.0;
double angular_velocity_y_avg = 0.0;
double angular_velocity_z_avg = 0.0;
double linear_acceleration_x_avg = 0.0;
double linear_acceleration_y_avg = 0.0;
double linear_acceleration_z_avg = 0.0;

// Callback fn to run every time the imu subscriber sees new data
void imu_callback(sensor_msgs::Imu imu_data) 
{   
    // Store the data in imu_unfiltered
    imu_unfiltered = imu_data;

    // We must have received a message to enter this function, so set message_received to true
    imu_message_received = true;

    // The number of readings we have taken is 1 more than the last time we were in this function
    reading_count++;

    
}

// Updates the averages based on the latest imu readings
void updateArrays(int index){

    // Procedure for updating avgs:
    // 1. Multiply the previous avg by the number of previous readings to get to the sum of all previous readings.
    // 2. Add the newest reading to get the sum of all readings, including the latest one.
    // 3. Divide the sum of all readings by the total number of readings to get the average.

    orientation_x_array[index] = imu_unfiltered.orientation.x;
    orientation_y_array[index] = imu_unfiltered.orientation.y;
    orientation_z_array[index] = imu_unfiltered.orientation.z;
    orientation_w_array[index] = imu_unfiltered.orientation.w;

    angular_velocity_x_array[index] = imu_unfiltered.angular_velocity.x;
    angular_velocity_y_array[index] = imu_unfiltered.angular_velocity.y;
    angular_velocity_z_array[index] = imu_unfiltered.angular_velocity.z;

    linear_acceleration_x_array[index] = imu_unfiltered.linear_acceleration.x;
    linear_acceleration_y_array[index] = imu_unfiltered.linear_acceleration.y;
    linear_acceleration_z_array[index] = imu_unfiltered.linear_acceleration.z;

}

double arrayAvg(double array[]){

    double sum = 0.0;
    for (int i = 0; i < array_size; i++){
        sum += array[i];
    }
    return sum / array_size;
}

void recalcAverages(){

    orientation_x_avg = arrayAvg(orientation_x_array);
    orientation_y_avg = arrayAvg(orientation_y_array);
    orientation_z_avg = arrayAvg(orientation_z_array);
    orientation_w_avg = arrayAvg(orientation_w_array);

    angular_velocity_x_avg = arrayAvg(angular_velocity_x_array);
    angular_velocity_y_avg = arrayAvg(angular_velocity_y_array);
    angular_velocity_z_avg = arrayAvg(angular_velocity_z_array);

    linear_acceleration_x_avg = arrayAvg(linear_acceleration_x_array);
    linear_acceleration_y_avg = arrayAvg(linear_acceleration_y_array);
    linear_acceleration_z_avg = arrayAvg(linear_acceleration_z_array);

}

// The piece de resistance
double lowPassExponential(double average, double newVal){
    
    // The filter_factor determines the weight given to previous readings vs new readings.
    if (0.0 < filter_factor && filter_factor <= 1.0)
    {
        return average * (1.0 - filter_factor) + newVal * filter_factor;
    } 
    else
    {
        ROS_ERROR_STREAM("Filter Factor must be between > 0 and <= 1.";);
        return -1.0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_lowpass");
    ros::NodeHandle nh;

    std::string robot_name = argv[1];
    
    // Subscribe to the imu readings
    ros::Subscriber imu_odom_sub = nh.subscribe("/"+ robot_name + COMMON_NAMES::IMU_TOPIC, 223, imu_callback);
    // Publish filtered imu readings
    ros::Publisher imu_odom_pub = nh.advertise<sensor_msgs::Imu>("/" + robot_name + COMMON_NAMES::IMU_FILTERED_TOPIC, 223);

    // Wait here until the imu has given us something
    while(ros::ok() && !imu_message_received)
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    // Inf loop
    while(ros::ok())
    {

        // Create a variable to hold the filtered data
        sensor_msgs::Imu imu_filtered;

        // Only do the stuff if we have received new data since the last time we did the stuff.
        if(reading_count > prev_reading_count)
        {

            // Populate all the bits and pieces of imu_filtered with data from imu_unfiltered, applying the lowpass filter as appropriate.

            imu_filtered.header = imu_unfiltered.header;

            imu_filtered.orientation.x = lowPassExponential(orientation_x_avg, imu_unfiltered.orientation.x);
            imu_filtered.orientation.y = lowPassExponential(orientation_y_avg, imu_unfiltered.orientation.y);
            imu_filtered.orientation.z = lowPassExponential(orientation_z_avg, imu_unfiltered.orientation.z);
            imu_filtered.orientation.w = lowPassExponential(orientation_w_avg, imu_unfiltered.orientation.w);

            imu_filtered.orientation_covariance = imu_unfiltered.orientation_covariance;

            imu_filtered.angular_velocity.x = lowPassExponential(angular_velocity_x_avg, imu_unfiltered.angular_velocity.x);
            imu_filtered.angular_velocity.y = lowPassExponential(angular_velocity_y_avg, imu_unfiltered.angular_velocity.y);
            imu_filtered.angular_velocity.z = lowPassExponential(angular_velocity_z_avg, imu_unfiltered.angular_velocity.z);
            
            imu_filtered.angular_velocity_covariance = imu_unfiltered.angular_velocity_covariance;

            imu_filtered.linear_acceleration.x = lowPassExponential(linear_acceleration_x_avg, imu_unfiltered.linear_acceleration.x);
            imu_filtered.linear_acceleration.y = lowPassExponential(linear_acceleration_y_avg, imu_unfiltered.linear_acceleration.y);
            imu_filtered.linear_acceleration.z = lowPassExponential(linear_acceleration_z_avg, imu_unfiltered.linear_acceleration.z);
            
            imu_filtered.linear_acceleration_covariance = imu_unfiltered.linear_acceleration_covariance;

            // Publish imu_filtered now that we have filled in all the blanks.

            imu_odom_pub.publish(imu_filtered);

            updateArrays(reading_count % 20);
            recalcAverages();


            // Update the number of readings we have seen and handled.

            prev_reading_count = reading_count;

        }

        // Spinny spin
        ros::spinOnce();

    }

    ros::spin();
    return 0;
}