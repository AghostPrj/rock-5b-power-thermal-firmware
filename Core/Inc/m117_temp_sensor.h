//
// Created by ggg17 on 2023/1/15.
//

#ifndef TEST_401CCU6_M117_TEMP_SENSOR_H
#define TEST_401CCU6_M117_TEMP_SENSOR_H


#include "main.h"

#define M117_DEVICE_ADDR 0x44
#define M117_DEVICE_WRITE_ADDR (M117_DEVICE_ADDR << 1)| 0x1
#define M117_DEVICE_READ_ADDR (M117_DEVICE_ADDR << 1)

#define M117_EXEC_ERROR 0xFFFF


uint8_t crc8_m117(uint8_t *data, uint16_t length);

uint16_t set_m117_config(I2C_HandleTypeDef *hi2c, uint8_t conf);

uint16_t get_m117_config_and_status(I2C_HandleTypeDef *hi2c);

uint16_t clear_m117_status(I2C_HandleTypeDef *hi2c);

uint16_t reset_m117(I2C_HandleTypeDef *hi2c);

float read_m117_temperature(I2C_HandleTypeDef *hi2c);

#endif //TEST_401CCU6_M117_TEMP_SENSOR_H
