


class battery_level
{
private:
public:
    static const float discharge_rate = 0.01; // -1.0%/10 seconds
    static const float speed = 0.6;
    battery_level(/* args */);
    ~battery_level();
    static float calc_distance(float target_x, float target_y, float current_x, float current_y);
    static float base_battery_level(const float discharge_rate, float distance, const float speed);
    static float calc_soft_deadline(float percentage_needed);
    static float calc_hard_deadline(float percentage_needed);
};

battery_level::battery_level(/* args */)
{
}

battery_level::~battery_level()
{
}
