//
// Created by ggg17 on 2023/6/27.
//

#include "fan_pwm.h"

uint32_t fan_duty_calculate(float current_temp, uint32_t current_duty) {
    uint32_t proposed_duty_cycle;

    if (current_temp <= LOW_TEMP) {
        proposed_duty_cycle = DUTY_MIN;
    } else if (current_temp >= HIGH_TEMP) {
        proposed_duty_cycle = DUTY_MAX;
    } else {
        proposed_duty_cycle = DUTY_MIN + (current_temp - LOW_TEMP) * (DUTY_MAX - DUTY_MIN) / (HIGH_TEMP - LOW_TEMP);
    }

    if (proposed_duty_cycle > current_duty + DUTY_CHANGE_LIMIT) {
        proposed_duty_cycle = current_duty + DUTY_CHANGE_LIMIT;
    } else if (proposed_duty_cycle < current_duty - DUTY_CHANGE_LIMIT) {
        proposed_duty_cycle = current_duty - DUTY_CHANGE_LIMIT;
    }

    return (uint32_t) proposed_duty_cycle;
}