
class Team{
   void setScout(RobotStatus& scout);
   void setExcavator(RobotStatus& excavator);
   void setHauler(RobotStatus& hauler);
   RobotStatus getExcavator();
   RobotStatus getHauler();
   RobotStatus getScout();
   bool shouldScoutDisband();
   void disbandScout();
   bool shouldTeamDisband();
   void disbandTeam();
   bool isExcavatorRecruited();
   bool isHaulerRecruited();

   void updateMacroState();
   void setMacroState();
   TeamState getMacroState();
}
