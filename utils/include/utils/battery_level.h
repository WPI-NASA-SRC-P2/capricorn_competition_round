class battery_level
{
private:
public:
    float discharge_rate = 0.01; // -1.0%/10 seconds
    float speed = 0.6;
    float distance;
    battery_level(/* args */);
    ~battery_level();
    float base_battery_level(const float discharge_rate, float distance, const float speed);
    float calc_soft_deadline(float percentage_needed);
    float calc_hard_deadline(float percentage_needed);
};

battery_level::battery_level(/* args */)
{
}

battery_level::~battery_level()
{
}
