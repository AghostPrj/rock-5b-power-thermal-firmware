//
// Created by ggg17 on 2023/1/15.
//
#include "m117_temp_sensor.h"
#include "cmsis_os2.h"


extern UART_HandleTypeDef huart1;

const uint16_t M117_CONVERT_TEMPERATURE_CMD = 0x44CC;
const uint16_t M117_WRITE_CONFIG_CMD = 0x0652;
const uint16_t M117_READ_CONFIG_AND_STATUS_CMD = 0x2DF3;
const uint16_t M117_CLEAR_STATUS_CMD = 0x4130;
const uint16_t M117_BREAK_REPEATED_CONVERT_TEMPERATURE_CMD = 0x9330;
const uint16_t M117_SOFT_RESET_CMD = 0xA230;

uint8_t crc8_m117(uint8_t *data, uint16_t length) {
    uint8_t i;
    uint8_t crc = 0xff;        // Initial value
    while (length--) {
        crc ^= *data++;        // crc ^= *data; data++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

uint16_t set_m117_config(I2C_HandleTypeDef *hi2c, uint8_t conf) {
    HAL_StatusTypeDef i2cCmdResult;
    i2cCmdResult = HAL_I2C_IsDeviceReady(hi2c, 0x44 << 1, 3, 0xf);
    if (i2cCmdResult == HAL_OK) {
        osDelay(5);

        uint8_t data[5] = {0x06, 0x52, conf, 0xff, 0x0};

        data[4] = crc8_m117((uint8_t *) &data, 4);

        i2cCmdResult = HAL_I2C_Master_Transmit(hi2c, M117_DEVICE_WRITE_ADDR, (uint8_t *) &data,
                                               5, 0xf);

        if (i2cCmdResult == HAL_OK) {
            return 0x0;
        } else {
            return M117_EXEC_ERROR;
        }


    } else {
        return M117_EXEC_ERROR;
    }

}

uint16_t reset_m117(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef i2cCmdResult;
    i2cCmdResult = HAL_I2C_IsDeviceReady(hi2c, 0x44 << 1, 3, 0xf);


    if (i2cCmdResult == HAL_OK) {
        osDelay(5);
        i2cCmdResult = HAL_I2C_Master_Transmit(hi2c, M117_DEVICE_WRITE_ADDR, (uint8_t *) &M117_SOFT_RESET_CMD,
                                               2, 0xf);
        if (i2cCmdResult == HAL_OK) {
            return 0x0;
        } else {
            return M117_EXEC_ERROR;
        }
    } else {
        return M117_EXEC_ERROR;
    }
}

uint16_t clear_m117_status(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef i2cCmdResult;
    i2cCmdResult = HAL_I2C_IsDeviceReady(hi2c, 0x44 << 1, 3, 0xf);


    if (i2cCmdResult == HAL_OK) {
        osDelay(5);

        i2cCmdResult = HAL_I2C_Master_Transmit(hi2c, M117_DEVICE_WRITE_ADDR, (uint8_t *) &M117_CLEAR_STATUS_CMD,
                                               2, 0xf);
        if (i2cCmdResult == HAL_OK) {
            return 0x0;
        } else {
            return M117_EXEC_ERROR;
        }

    } else {
        return M117_EXEC_ERROR;
    }
}

uint16_t get_m117_config_and_status(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef i2cCmdResult;
    uint8_t i2cResult[3] = {0x0, 0x0, 0x0};
    i2cCmdResult = HAL_I2C_Master_Transmit(hi2c, M117_DEVICE_WRITE_ADDR, (uint8_t *) &M117_READ_CONFIG_AND_STATUS_CMD,
                                           2, 0xf);
    if (i2cCmdResult == HAL_OK) {
        osDelay(10);

        i2cCmdResult = HAL_I2C_Master_Receive(hi2c, M117_DEVICE_READ_ADDR, (uint8_t *) &i2cResult, 3, 0xf);
        if (i2cCmdResult == HAL_OK) {
            uint8_t cmdResultCrc = i2cResult[2];
            uint8_t calcCrc = crc8_m117((uint8_t *) &i2cResult, 2);

            if (cmdResultCrc != calcCrc) {
                return M117_EXEC_ERROR;
            }

            uint16_t result = (i2cResult[1] << 8) | i2cResult[0];

            return result;

        } else {
            return M117_EXEC_ERROR;
        }


    } else {
        return M117_EXEC_ERROR;
    }
}

float read_m117_temperature(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef i2cCmdResult;


    uint8_t i2cResult[3] = {0x0, 0x0, 0x0};
    i2cCmdResult = HAL_I2C_IsDeviceReady(hi2c, 0x44 << 1, 3, 0xf);
    if (i2cCmdResult == HAL_OK) {
        osDelay(1);
        i2cCmdResult = HAL_I2C_Master_Transmit(hi2c, M117_DEVICE_WRITE_ADDR, (uint8_t *) &M117_CONVERT_TEMPERATURE_CMD,
                                               2, 0xf);
        if (i2cCmdResult == HAL_OK) {
            osDelay(1);
            i2cCmdResult = HAL_I2C_Master_Receive(hi2c, M117_DEVICE_READ_ADDR, (uint8_t *) &i2cResult, 3, 0xf);
            if (i2cCmdResult == HAL_OK) {
                uint8_t cmdResultCrc = i2cResult[2];
                uint8_t calcCrc = crc8_m117((uint8_t *) &i2cResult, 2);

                if (cmdResultCrc != calcCrc) {
                    return M117_EXEC_ERROR;
                }
                int16_t tmp = (int16_t) (i2cResult[0] << 8 | i2cResult[1]);

                return (float) (tmp / 256.0 + 40);

            } else {
                return M117_EXEC_ERROR;
            }
        } else {
            return M117_EXEC_ERROR;
        }


    } else {
        return M117_EXEC_ERROR;
    }
}