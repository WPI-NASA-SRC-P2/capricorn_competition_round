


class battery_level
{
private:
    
public:
    static constexpr float discharge_rate = 0.01; // -1.0%/10 seconds
    static constexpr float speed = 0.6;
    battery_level(/* args */);
    ~battery_level();

    /**
     * @brief Calculates the 2-D euclidean distance between two points
     * 
     * @param target_x Target Location X coordinate
     * @param target_y Target Location Y coordinate
     * @param current_x  Robot's Current Location X coordinate
     * @param current_y  Robot's Current Location Y coordinate
     * @return float The distance between two points
     */
    static float calc_distance(float target_x, float target_y, float current_x, float current_y);

    /**
     * @brief Calculates the needed minimum battery level for the robot to drive to target location in the simulation
     * 
     * @param discharge_rate The rate at which the battery level decreases
     * @param distance The distance from the robot's current location to the target location in the simulation
     * @param speed The speed that the robot is traveling in the simulation
     * @return float The mimimun battery level that the robot will need to drive to the target location in the simulation
     */
    static float base_battery_level(const float discharge_rate, float distance, const float speed);

    /**
     * @brief Calculate the soft deline for the robot at current location in the simulation
     * 
     * @param percentage_needed Base percentage level required by the robot to reach the target location (this value doesn't consider obstacles and other delays)
     * @return float Soft battery level required  
     */
    static float calc_soft_deadline(float percentage_needed);

    /**
     * @brief Calculates the hard battery level deadline for the robot at current location in the simulation
     * 
     * @param percentage_needed Base percentage level required by the robot to reach the target location (this value doesn't consider obstacles and other delays)
     * @return float Hard battery level deadline
     */
    static float calc_hard_deadline(float percentage_needed);
};

battery_level::battery_level(/* args */)
{
}

battery_level::~battery_level()
{
}
