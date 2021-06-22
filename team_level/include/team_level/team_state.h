
// /**
//  * The macrostate is a state machine for team-level coordination.
//  *
//  * A state in the macrostate is a collection of individual robot states.
//  */
// class MacroState {
//   public:
// };

class TeamState {
   
public:

   TeamState(uint32_t un_id,
         const std::string& str_name) :
      m_pcRobotScheduler(nullptr),
      m_unId(un_id),
      m_strName(str_name) {}

   virtual ~TeamState() {}

   uint32_t getId() const { return m_unId; }

   const std::string& getName() const { return m_strName; }
   
   virtual void entryPoint() = 0;
   
   virtual void exitPoint() = 0;
   
   virtual void step() = 0;
   
   virtual State& transition() = 0;

   State& getState(uint32_t un_state);

   void setRobotScheduler(RobotScheduler& c_robot_scheduler);
   
private:

   RobotScheduler* m_pcRobotScheduler;
   uint32_t m_unId;
   std::string m_strName;
};

// All the states as per the diagram
class TeamSearch: public TeamState{

}

class Recruite: public TeamState{

}

class ReachSite: public TeamState{
   
}
