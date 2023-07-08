/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

#include "i2c.h"
#include "usbd_cdc_if.h"
#include "semphr.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */





extern IWDG_HandleTypeDef hiwdg;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim10;

extern uint8_t *outputBuffer;
extern uint32_t fanItCount;

extern uint16_t *adcValueBuffer1;

extern kalman_state *kalmanStateVdda;
extern kalman_state *kalmanStateMcuTemperature;
extern kalman_state *kalmanStateAdcVal0;
extern kalman_state *kalmanStateAdcVal1;
extern kalman_state *kalmanStateAdcVal2;
extern kalman_state *kalmanStateAdcVal3;

extern kalman_state *kalmanStateSensorTemperatureSensor;

extern kalman_state *kalmanStateFanSpeedSensor;

volatile uint32_t actualVdda = 0;
volatile int16_t mcuTemperature = 0;

volatile float sensorTemperature = 0;
volatile uint32_t currentDuty = DUTY_MAX;

volatile int32_t sensorReadErrorCount = 0;

volatile uint32_t adcVoltage[4] = {0};

volatile uint32_t fanSpeed = 0;


osThreadId_t updateFanSpeedValueTaskHandle;
const osThreadAttr_t updateFanSpeedValueTaskAttributes = {
        .name = "updateFanSpeedValueTaskHandle",
        .stack_size = 512 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t updateAdcValueTaskHandle;
const osThreadAttr_t updateAdcValueTaskAttributes = {
        .name = "updateAdcValueTaskHandle",
        .stack_size = 512 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t updateSensorTemperatureAndPWMValueTaskHandle;
const osThreadAttr_t updateSensorTemperatureAndPWMValueTaskAttributes = {
        .name = "updateSensorTemperatureAndPWMValueTaskHandle",
        .stack_size = 512 * 8,
        .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
        .name = "defaultTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void startUpdateSensorTemperatureAndPWMValueTask(void *args);

void startUpdateAdcValueTask(void *args);

_Noreturn void startUpdateFanSpeedValueTask(void *args);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */



    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    updateAdcValueTaskHandle = osThreadNew(startUpdateAdcValueTask, NULL, &updateAdcValueTaskAttributes);
    updateSensorTemperatureAndPWMValueTaskHandle = osThreadNew(startUpdateSensorTemperatureAndPWMValueTask, NULL,
                                                               &updateSensorTemperatureAndPWMValueTaskAttributes);
    updateFanSpeedValueTaskHandle = osThreadNew(startUpdateFanSpeedValueTask, NULL, &updateFanSpeedValueTaskAttributes);
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN StartDefaultTask */




    uint32_t devId0 = HAL_GetUIDw0();
    uint32_t devId1 = HAL_GetUIDw1();
    uint32_t devId2 = HAL_GetUIDw2();


    uint8_t *deviceId = malloc(32);
    if (deviceId == NULL) {
        Error_Handler();
    }


    sprintf(deviceId, "%08lX%08lX%08lX", devId2, devId1, devId0);
    HAL_IWDG_Refresh(&hiwdg);


    uint8_t *outputFormatStr = malloc(256);
    if (deviceId == NULL) {
        Error_Handler();
    }

    for (int i = 0; i < 256; i++) {
        *(outputFormatStr + i) = 0;
    }

    char *prefix = "{\"op\":\"data_transfer\",\"ver\":\"v1.0\",\"data\":{\"dev_id\":\"";
    memcpy(outputFormatStr, prefix, strlen(prefix));


    strcat(outputFormatStr, deviceId);
    strcat(outputFormatStr, "\",\"sys_tick\":%lu,"
                            "\"vdda\":%lu,\"mcu_temp\":%hd,"
                            "\"adc_ch_1\":%lu,\"adc_ch_2\":%lu,"
                            "\"adc_ch_3\":%lu,\"adc_ch_4\":%lu,"
                            "\"sensor_temp\":%ld.%ld,\"duty_pwm\":%lu.%lu,"
                            "\"fan_speed\":%lu}}\n");

    int32_t sensorTemperatureIntPart = 0, sensorTemperatureMantissa = 0;

    uint32_t pwmDutyIntPart = 0, pwmDutyMantissa = 0;

    float duty = 0;

    free(outputFormatStr);

    uint32_t sendLen = 0;

    for (int i = 0; i < 50; i++) {
        osDelay(20);

        HAL_IWDG_Refresh(&hiwdg);
    }


    /* Infinite loop */
    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);


        if (sensorTemperature < -274) {
            sensorTemperatureIntPart = -999;
            sensorTemperatureMantissa = 0;
        } else {
            sensorTemperatureIntPart = (int32_t) sensorTemperature;

            if (sensorTemperature < 0) {
                sensorTemperatureMantissa = (int32_t) (
                        ((sensorTemperature * -1) - ((float) sensorTemperatureIntPart * -1)) * 1000);
            } else {
                sensorTemperatureMantissa = (int32_t) (((sensorTemperature) - ((float) sensorTemperatureIntPart)) *
                                                       1000);
            }

        }


        duty = (float) currentDuty * 100 / PERIOD;

        pwmDutyIntPart = (uint32_t) duty;
        pwmDutyMantissa = (uint32_t) ((duty - (float) pwmDutyIntPart) * 1000);


        sprintf(outputBuffer, outputFormatStr,
                HAL_GetTick(),
                actualVdda,
                mcuTemperature,
                adcVoltage[0],
                adcVoltage[1],
                adcVoltage[2],
                adcVoltage[3],
                sensorTemperatureIntPart,
                sensorTemperatureMantissa,
                pwmDutyIntPart,
                pwmDutyMantissa,
                fanSpeed);
        sendLen = strlen(outputBuffer);
        HAL_UART_Transmit(&huart1, outputBuffer, sendLen, 0xff);
        CDC_Transmit_FS(outputBuffer, sendLen);

        for (int i = 0; i < 5; ++i) {
            HAL_IWDG_Refresh(&hiwdg);
            osDelay(100);
        }


    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
_Noreturn void startUpdateAdcValueTask(void *args) {

    while (1) {

        kalmanUpdateUint32(kalmanStateVdda,
                           __LL_ADC_CALC_VREFANALOG_VOLTAGE((*(adcValueBuffer1 + 5)), LL_ADC_RESOLUTION_12B));
        actualVdda = (uint32_t) kalmanGetValue(kalmanStateVdda);

        kalmanUpdateInt32(kalmanStateMcuTemperature,
                          __LL_ADC_CALC_TEMPERATURE(actualVdda, *(adcValueBuffer1 + 4), LL_ADC_RESOLUTION_12B));
        mcuTemperature = (int16_t) kalmanGetValue(kalmanStateMcuTemperature);

        kalmanUpdateUint32(kalmanStateAdcVal0,
                           __LL_ADC_CALC_DATA_TO_VOLTAGE(actualVdda, *(adcValueBuffer1 + 0), LL_ADC_RESOLUTION_12B));
        adcVoltage[0] = (uint32_t) kalmanGetValue(kalmanStateAdcVal0);

        kalmanUpdateUint32(kalmanStateAdcVal1,
                           __LL_ADC_CALC_DATA_TO_VOLTAGE(actualVdda, *(adcValueBuffer1 + 1), LL_ADC_RESOLUTION_12B));
        adcVoltage[1] = (uint32_t) kalmanGetValue(kalmanStateAdcVal1);

        kalmanUpdateUint32(kalmanStateAdcVal2,
                           __LL_ADC_CALC_DATA_TO_VOLTAGE(actualVdda, *(adcValueBuffer1 + 2), LL_ADC_RESOLUTION_12B));
        adcVoltage[2] = (uint32_t) kalmanGetValue(kalmanStateAdcVal2);

        kalmanUpdateUint32(kalmanStateAdcVal3,
                           __LL_ADC_CALC_DATA_TO_VOLTAGE(actualVdda, *(adcValueBuffer1 + 3), LL_ADC_RESOLUTION_12B));
        adcVoltage[3] = (uint32_t) kalmanGetValue(kalmanStateAdcVal3);

        osDelay(5);
    }
}

_Noreturn void startUpdateSensorTemperatureAndPWMValueTask(void *args) {

    reset_m117(&hi2c1);
    clear_m117_status(&hi2c1);

    osDelay(10);

    for (uint16_t i = 0; i < 16; i++) {
        osDelay(10);
        read_m117_temperature(&hi2c1);
    }

    float init_temperature = read_m117_temperature(&hi2c1);
    if (init_temperature >= 0xffff) {
        init_temperature = 25;
    }
    kalmanInitWithDefaultValue(kalmanStateSensorTemperatureSensor, 0.01f, 0.01f, init_temperature);

    float tmpSensorTemperature = 0;


    while (1) {
        if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) {
            tmpSensorTemperature = read_m117_temperature(&hi2c1);
        } else {
            tmpSensorTemperature = 0xffff;
        }


        if (tmpSensorTemperature >= 0xffff) {
            if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) {
                HAL_I2C_DeInit(&hi2c1);
            }


            osDelay(50);


            if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET) {
                HAL_StatusTypeDef initState = HAL_I2C_Init(&hi2c1);
                if (initState == HAL_OK) {
                    osDelay(5);
                    reset_m117(&hi2c1);
                    clear_m117_status(&hi2c1);
                }
            }

            if (sensorReadErrorCount <= 10) {
                sensorReadErrorCount++;
            } else {
                currentDuty = DUTY_MAX;
                sensorTemperature = -999;
            }


        } else {
            kalmanUpdateFloat(kalmanStateSensorTemperatureSensor, tmpSensorTemperature);
            sensorTemperature = kalmanGetValue(kalmanStateSensorTemperatureSensor);
            currentDuty = fan_duty_calculate(sensorTemperature,
                                             __HAL_TIM_GET_COMPARE(&htim11, TIM_CHANNEL_1));
            sensorReadErrorCount = 0;
        }


        if (__HAL_TIM_GET_COMPARE(&htim11, TIM_CHANNEL_1) != currentDuty) {
            __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, currentDuty);
        }

        if (__HAL_TIM_GET_COMPARE(&htim10, TIM_CHANNEL_1) != currentDuty) {
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, currentDuty);
        }

        osDelay(5);

    }
}

_Noreturn void startUpdateFanSpeedValueTask(void *args) {
    uint32_t tmpFanItNum = 0;
    uint32_t lastUpdate = 0;
    uint32_t nowTick = 0;

    float tmpFanSpeed = 0;
    while (1) {


        for (int i = 0; i < 5; ++i) {
            HAL_IWDG_Refresh(&hiwdg);
            osDelay(100);
        }


        tmpFanItNum = fanItCount;
        fanItCount = 0;

        nowTick = HAL_GetTick();


        if (nowTick <= lastUpdate) {
            lastUpdate = HAL_GetTick();
            continue;
        } else {

            if (tmpFanItNum < 1) {
                fanSpeed = 0;
            } else {

                tmpFanSpeed = ((float) (tmpFanItNum * 60 * 500) / (float) (nowTick - lastUpdate));
                kalmanUpdateFloat(kalmanStateFanSpeedSensor, tmpFanSpeed);
                fanSpeed = (uint32_t) kalmanGetValue(kalmanStateFanSpeedSensor);
            }
            lastUpdate = nowTick;
        }

    }
}

/* USER CODE END Application */

