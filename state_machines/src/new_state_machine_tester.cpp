#include <ros/ros.h>
#include <state_machines/hauler_state_machine.h>
#include <state_machines/excavator_state_machine.h>
#include <state_machines/scout_state_machine.h>
#include <state_machines/common_robot_state_machine.h>

#include <state_machines/RobotStateMachineTaskAction.h>
#include <actionlib/server/simple_action_server.h>


int main(int argc, char** argv)
{
   ros::init(argc, argv, "state_machine_tester");
   ros::NodeHandle nh;

   try {
      // add all the schedulers
      HaulerScheduler hSchd(700);
      ScoutScheduler sSchd(700);
      ExcavatorScheduler eSchd(700);

      // add all the states for hauler
      hSchd.addState(new GoToProcPlant());
      hSchd.addState(new ParkAtHopper());
      hSchd.addState(new ResetOdom());
      hSchd.addState(new UndockHopper());
      hSchd.addState(new DumpVolatileAtHopper());
      hSchd.addState(new GoToExcavator());
      hSchd.addState(new ParkAtExcavator());
      hSchd.addState(new UndockExcavator());
      hSchd.addState(new DumpVolatile());

      // add all the states for excavator
      eSchd.addState(new GoToDefaultArmPosition());
      eSchd.addState(new GoToScout());
      eSchd.addState(new ParkAndPub());
      eSchd.addState(new DigAndDump());
      
      // add all the states for scout
      sSchd.addState(new Search());
      sSchd.addState(new Undock());
      sSchd.addState(new Locate());
      
      sSchd.setInitialState(SCOUT_SEARCH_VOLATILE);
      hSchd.setInitialState(HAULER_RESET_ODOM_AT_HOPPER);
      eSchd.setInitialState(EXCAVATOR_DIG_AND_DUMP_VOLATILE);  
      
      eSchd.exec();
      hSchd.exec();
      sSchd.exec(); 
      
      return 0;
   }
   catch(StateMachineException& ex) {
      std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
   }
}