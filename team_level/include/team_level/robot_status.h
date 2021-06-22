
class RobotStatus{
   public:
      RobotStatus();
      void addRobot(ROBOTS_ENUM robot)
      bool hasFailed(ROBOTS_ENUM robot);
      bool isDone(ROBOTS_ENUM robot);
   private:
      ros::Subscriber robot_state_subscriber;
}
