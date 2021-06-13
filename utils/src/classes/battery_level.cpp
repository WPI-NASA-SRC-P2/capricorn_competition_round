#include <iostream>
#include "utils/battery_level.h"
#include <bits/stdc++.h>
/**
 * @brief 
 * 
 * @param target_x 
 * @param target_y 
 * @param current_x 
 * @param current_y 
 * @return float 
 */
float calc_distance(float target_x, float target_y, float current_x, float current_y){
    return sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
}


/**
 * @brief 
 * 
 * @param discharge_rate The rate at which the battery level is drained
 * @param distance 
 * @param speed 
 * @return float 
 */
float base_battery_level(const float discharge_rate, float distance, const float speed){
    float percentage_needed = discharge_rate * (distance / speed);
}

/**
 * @brief 
 * 
 * @param percentage_needed The percetnage needed to drive to task
 * @return float 
 */
float calc_soft_deadline(float percentage_needed){
    float soft_deadline = 0;
    float safety_factor = 1.4;
    soft_deadline = percentage_needed*safety_factor;
    return soft_deadline;
}

/**
 * @brief 
 * 
 * @param percentage_needed 
 * @return float 
 */
float calc_hard_deadline(float percentage_needed){
    float hard_deadline = 0;
    float safety_factor = 1.2;
    hard_deadline = percentage_needed*safety_factor;
    return hard_deadline;
}