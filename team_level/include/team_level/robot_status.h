
class RobotStatus{
   public:
      RobotStatus();
      int addRobot()
      int getId();
      bool hasFailed();
      bool isDone();
   private:
      ros::Subscriber robot_state_subscriber;
}
