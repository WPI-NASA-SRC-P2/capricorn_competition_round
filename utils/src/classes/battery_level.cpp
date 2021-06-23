#include <iostream>
#include "utils/battery_level.h"
#include <bits/stdc++.h>

float battery_level::calc_distance(float target_x, float target_y, float current_x, float current_y){
    return sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
}


float battery_level::base_battery_level(const float discharge_rate, float distance, const float speed){
    float percentage_needed = discharge_rate * (distance / speed);
    return percentage_needed;
}


float battery_level::calc_soft_deadline(float percentage_needed){
    float soft_deadline = 0;
    float safety_factor = 1.4;
    soft_deadline = percentage_needed*safety_factor;
    return soft_deadline;
}


float battery_level::calc_hard_deadline(float percentage_needed){
    float hard_deadline = 0;
    float safety_factor = 1.2;
    hard_deadline = percentage_needed*safety_factor;
    return hard_deadline;
}