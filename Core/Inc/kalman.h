//
// Created by ggg17 on 2023/6/26.
//

#ifndef ROCK_5B_POWER_THERMAL_CTRL_KALMAN_H
#define ROCK_5B_POWER_THERMAL_CTRL_KALMAN_H

#include <stdint.h>

typedef struct {
    volatile float x_est_last;
    volatile float P_last;
    volatile float K;
    volatile float P;
    volatile float P_temp;
    volatile float x_temp_est;
    volatile float x_est;
    volatile float Q;
    volatile float R;
    volatile float z_measured;
    volatile float z_real;
    volatile float sum_error;
} kalman_state;

void kalmanInit(kalman_state *s, float Q, float R);

void kalmanInitWithDefaultValue(kalman_state *s, float Q, float R, float init_value);

void kalmanUpdateUint32(kalman_state *state, uint32_t measurement);

void kalmanUpdateInt32(kalman_state *state, int32_t measurement);

void kalmanUpdateFloat(kalman_state *state, float measurement);

float kalmanGetValue(kalman_state *state);

#endif //ROCK_5B_POWER_THERMAL_CTRL_KALMAN_H
