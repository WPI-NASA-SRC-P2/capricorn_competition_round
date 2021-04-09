#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <operations/navigation_algorithm.h>
#include <perception/ObjectArray.h>
//#include <perception/msg/Object.msg>


#define UPDATE_HZ 10

std::string robot_name;
ros::Publisher fl_steer, fr_steer, bl_steer, br_steer, fl_drive, fr_drive, bl_drive, br_drive;
ros::Subscriber image_sub; 
bool hopper_x, parked = false;
bool processingPlant_z;
bool found_orientation = false;

std::vector<double> drive, steer;
float max_diff = -0;

void publishMessage(std::vector<double> velocity, std::vector<double> angle)
{
    std_msgs::Float64 angle_msg, vel_msg;
    angle_msg.data = angle[0];
    fl_steer.publish(angle_msg);
    angle_msg.data = angle[1];
    fr_steer.publish(angle_msg);
    angle_msg.data = angle[2];
    br_steer.publish(angle_msg);
    angle_msg.data = angle[3];
    bl_steer.publish(angle_msg);
    
    vel_msg.data = velocity[0];
    fl_drive.publish(vel_msg);
    vel_msg.data = velocity[1];
    fr_drive.publish(vel_msg);
    vel_msg.data = velocity[2];
    br_drive.publish(vel_msg);
    vel_msg.data = velocity[3];
    bl_drive.publish(vel_msg);    
}

void objects_callback(const perception::ObjectArray& objects) 
{
    ROS_INFO("In callback");
    
    bool found_ea = false, found_ra = false;
    float center_x_ea = INT_MAX, center_x_ra = INT_MAX, z_ra = INT_MAX;
    float size_x_ea = INT_MAX, size_y_ea = INT_MAX, size_x_ra = INT_MAX, size_y_ra= INT_MAX;
    long area_ra = -1, area_ea = -1;
        
    for(int i = 0; i < objects.number_of_objects; i++) 
    {   
        if(objects.obj[i].label == "robotAntenna") 
        {
            if(found_ra)
            {
                return;
            }
            found_ra = true;
            z_ra = objects.obj[i].point.z;
            size_x_ra = objects.obj[i].size_x;
            size_y_ra = objects.obj[i].size_y;
            area_ra = size_x_ra * size_y_ra;
            center_x_ea = objects.obj[i].center.x;
        }
        else if(objects.obj[i].label == "excavatorArm") 
        {
            if(found_ea)
            {
                return;
            }
            found_ea = true;
            size_x_ea = objects.obj[i].size_x;
            size_y_ea = objects.obj[i].size_y;
            area_ea = size_x_ea * size_y_ea;
            center_x_ra = objects.obj[i].center.x;
        }
    }

    if(found_orientation)
    {
        ROS_INFO_STREAM("Height of RA: "<<size_y_ra);
        ROS_INFO_STREAM("Depth of RA: "<<z_ra);
    }

    if(found_ra && z_ra < 3 && found_orientation)
    {
        drive = {0,0,0,0};
        steer = {0,0,0,0};
    }

    if(found_ea && found_ra)
    {
        float ratio = size_x_ea / size_y_ea;
        ROS_INFO_STREAM("Dimensions of RA X: "<<size_x_ra<<", Y: "<<size_y_ra);
        ROS_INFO_STREAM("Dimensions of EA X: "<<size_x_ea<<", Y: "<<size_y_ea);
        ROS_INFO_STREAM("Ratio: "<<size_x_ea / size_y_ea);

        if(center_x_ea < center_x_ra)
        {
            float diff = center_x_ra - center_x_ea;
            max_diff = std::max(max_diff, diff);

            if(diff > 150 && diff < max_diff - 20)
            {
                ROS_INFO("Stop");
                found_orientation = true;
                drive = {3,3,3,3};
                steer = {0,0,0,0};
            }
            ROS_INFO_STREAM("Center RA: "<<center_x_ea<<", Center EA: "<<center_x_ra);
            ROS_INFO_STREAM("Difference centers: "<<diff);
        }
        ROS_INFO_STREAM("");
    }


}

int main(int argc, char *argv[])
{
    std::string name(argv[1]);
    robot_name = name;

    ros::init(argc, argv, "park_hauler");

    ros::NodeHandle nh;

    fl_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/steer/command/position", 10);
    fr_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/steer/command/position", 10);
    bl_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/steer/command/position", 10);
    br_steer = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/steer/command/position", 10);

    fl_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_left_wheel/drive/command/velocity", 10);
    fr_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/front_right_wheel/drive/command/velocity", 10);
    bl_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_left_wheel/drive/command/velocity", 10);
    br_drive = nh.advertise<std_msgs::Float64>("/" + robot_name + "/back_right_wheel/drive/command/velocity", 10);

    ros::Rate update_rate(UPDATE_HZ);
    double radius = std::atof(argv[2]);
    double velocity = std::atof(argv[3]);
    geometry_msgs::Point pt;
    pt.x = radius;
    drive = NavigationAlgo::getDrivingVelocitiesRadialTurn(pt, velocity);
    steer = NavigationAlgo::getSteeringAnglesRadialTurn(pt);

    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + robot_name + COMMON_NAMES::OBJECT_DETECTION_OBJECTS_TOPIC, 1, &objects_callback);

    while (ros::ok())
    {

        // ROS_INFO_STREAM("publishing");
        if(parked)
        {
            ROS_INFO("Parked");
            break;
        }
        publishMessage(drive, steer);
        ros::spinOnce();
        update_rate.sleep();
    }

    ROS_INFO("Exiting");
}