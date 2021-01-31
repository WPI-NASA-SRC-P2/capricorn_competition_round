#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

//Class containing  Action Server methods
class actionToBePerformed
{
    protected:

    //create a nodehandle to create subscribers and publishers
    ros::NodeHandle nh;

    //declare a simple action server type object
    //create a nodehandle before this line otherwise it creates weird errors
    actionlib::SimpleActionServer<template_actionServer::template_actionMsgAction> action_server_;

    //create a variable to hold the name of the action server node
    std::string action_name_; 


    public:

    //constructor with initialization list
    actionToBePerformed(std::string name) :
        action_server_(nh, name, boost::bind(&actionToBePerformed) )
        action_name(name)
    {
        initializeSubsribers();
        initializePublishers();
        acion_server_.start();
    }


    private:

    //Initialize Subscribers here
    void initializeSubscribers(void)
    {
        ROS_INFO("Subscribers Initialized");
    }
 
    //Initialize Publishers here
    void initializePublishers(void)
    {
        ROS_INFO("Publishers Initialized");
    }

    //create callback functions for initialized subsribers
    void actionCallback(const template_actionServer::template_actionServerMsgPointer &goal)
    {
         
         do
         {
             /* 
             this is where all the cool robotics stuff happens
            */
         } while ({/* condition */});
        
}


int main(int argc, char** argv)
{
    //Initializing the Action Server Node
    ros::init(argc, argv, "template_actionServer");

    //Creating an instance of the Action Class
    actionToBePerformed action_1 

    //Setting the rate
    ros::Rate rate(0.5)

    int n = 0;
    while( ros::ok())
    {
        ros::spinOnce();
        n++;
        rate.sleep();
    }


}