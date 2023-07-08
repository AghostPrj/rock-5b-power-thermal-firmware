//
// Created by ggg17 on 2023/6/27.
//

#ifndef ROCK_5B_POWER_THERMAL_CTRL_FAN_PWM_H
#define ROCK_5B_POWER_THERMAL_CTRL_FAN_PWM_H

#include "main.h"

#define PERIOD 720

#define DUTY_MIN 200
#define DUTY_MAX 720

#define DUTY_CHANGE_LIMIT 15

#define LOW_TEMP 35
#define HIGH_TEMP 65


uint32_t fan_duty_calculate(float current_temp, uint32_t current_duty);


#endif //ROCK_5B_POWER_THERMAL_CTRL_FAN_PWM_H
