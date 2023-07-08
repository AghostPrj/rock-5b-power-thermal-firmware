//
// Created by ggg17 on 2023/6/26.
//

#include "kalman.h"

void kalmanInit(kalman_state *s, float Q, float R) {
    s->Q = Q;
    s->R = R;
    s->P = 1.0f;
    s->x_est_last = 0.0f;
}

void kalmanInitWithDefaultValue(kalman_state *s, float Q, float R, float init_value) {
    kalmanInit(s, Q, R);
    s->x_est_last = init_value;
}

void kalmanUpdateUint32(kalman_state *state, uint32_t measurement) {
    state->x_temp_est = state->x_est_last;
    state->P_temp = state->P_last + state->Q;
    state->K = state->P_temp * (1.0f / (state->P_temp + state->R));
    state->x_est = state->x_temp_est + state->K * ((float) measurement - state->x_temp_est);
    state->P = (1 - state->K) * state->P_temp;
    state->P_last = state->P;
    state->x_est_last = state->x_est;
}

void kalmanUpdateInt32(kalman_state *state, int32_t measurement) {
    state->x_temp_est = state->x_est_last;
    state->P_temp = state->P_last + state->Q;
    state->K = state->P_temp * (1.0f / (state->P_temp + state->R));
    state->x_est = state->x_temp_est + state->K * ((float) measurement - state->x_temp_est);
    state->P = (1 - state->K) * state->P_temp;
    state->P_last = state->P;
    state->x_est_last = state->x_est;
}

void kalmanUpdateFloat(kalman_state *state, float measurement) {
    state->x_temp_est = state->x_est_last;
    state->P_temp = state->P_last + state->Q;
    state->K = state->P_temp * (1.0f / (state->P_temp + state->R));
    state->x_est = state->x_temp_est + state->K * (measurement - state->x_temp_est);
    state->P = (1 - state->K) * state->P_temp;
    state->P_last = state->P;
    state->x_est_last = state->x_est;
}

float kalmanGetValue(kalman_state *state) {
    return state->x_est;
}

