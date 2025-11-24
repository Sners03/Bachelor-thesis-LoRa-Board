/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "utilities_def.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "main.h"
#include "stdio.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
/**
 * BUFFER:
 * 7 Bytes Header
 * 	4 Bytes Dev Addr
 * 	1 Byte FCtl
 * 	2 Bytes FCount
 * 1 Byte Port
 * 12 Bytes Payload
 * 	4 Bytes ax
 * 	4 Bytes ay
 * 	4 Bytes az
 */
#define BUFFER_SIZE 	19
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */
static UTIL_TIMER_Object_t timerTransmit;   //[JT]
static UTIL_TIMER_Object_t timerReceive;    //[JT]
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRANSMIT_PERIOD_MS 5000  /* set Tx period of timer to 2 seconds */     //[JT]
#define MPU_ADDR         0x68 << 1  // shifted for HAL (8-bit)
#define WHO_AM_I_REG         0x75
#define PWR_MGMT_1_REG       0x6B
#define ACCEL_XOUT_H         0x3B


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;
/* USER CODE BEGIN PV */
uint16_t BufferSize = BUFFER_SIZE;   //[JT]
uint8_t Buffer[BUFFER_SIZE];         //[JT]

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
static void TransmitPacket(void *context);    //[JT]
void MPU_Init(void);
void MPU_ReadAccel_f(float* ax, float* ay, float* az);
/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */
	MPU_Init();

  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */
  APP_LOG(TS_ON, VLEVEL_L, "******TRANSMITTER******\n\r");
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                        LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                        true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

  Radio.SetMaxPayloadLength(MODEM_LORA, BUFFER_SIZE);
  Radio.SetChannel(RF_FREQUENCY);

  	Buffer[0] = 90;
    Buffer[1] = 90;
    Buffer[2] = 90;
    Buffer[3] = 90;
    //Buffer[4] = '2';
    // Frame Counter
    Buffer[5] = 0;
    Buffer[6] = 1;
    //Buffer[7] = '_';
    //Buffer[8] = 'T';
    //Buffer[9] = 'X';

    /* Add delay between TX and RX =
    time Busy_signal is ON in RX opening window */
    HAL_Delay(Radio.GetWakeupTime() + 50);

    UTIL_TIMER_Create(&timerTransmit, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, TransmitPacket, NULL);
    UTIL_TIMER_SetPeriod(&timerTransmit, TRANSMIT_PERIOD_MS);
    UTIL_TIMER_Start(&timerTransmit);  // start transmitting packets every 2s

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */
static void TransmitPacket(void *context)
{
  float ax = 111111111111111.11111111111111;
  float ay = 2.0;
  float az = 3.0;
  MPU_ReadAccel_f(&ax, &ay, &az);
  //APP_LOG(TS_ON, VLEVEL_L, "x:%.2f\n", ax);
  memcpy(Buffer+9, &ax, 4);
  memcpy(Buffer+13, &ay, 4);
  memcpy(Buffer+17, &az, 4);
  APP_LOG(TS_ON, VLEVEL_L, "%s\n\r", Buffer);
  Radio.Send(Buffer, BufferSize);
}

// ---- Simple helper functions ----
uint8_t MPU_ReadReg(uint8_t reg) {
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, reg, 1, &data, 1, HAL_MAX_DELAY);
    return data;
}

void MPU_WriteReg(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, reg, 1, &value, 1, HAL_MAX_DELAY);
}

void MPU_Init(void) {
    // Wake up the MPU
    MPU_WriteReg(PWR_MGMT_1_REG, 0x00);
    HAL_Delay(100);

    // Verify communication
    uint8_t who_am_i = MPU_ReadReg(WHO_AM_I_REG);
    if (who_am_i == 0x71 || who_am_i == 0x73) {
    	APP_LOG(TS_ON, VLEVEL_L,"MPU9250 detected! WHO_AM_I = 0x%X\r\n", who_am_i);
    } else {
    	APP_LOG(TS_ON, VLEVEL_L,"MPU9250 not found! Read: 0x%X\r\n", who_am_i);
    }
}


void MPU_ReadAccel_f(float* ax, float* ay, float* az) {
	/**
	 * Reads the Data like in the normal version, but instantly convert it to a human understandable float
	 *
	 * sensitivity values:
	 * +-2g => 16384
	 * +-4g => 8192
	 * +-8g => 4096
	 * +-16g => 2048
	 *
	 * can be configured using Register (0x1C), but for our use-case (sending LoRa data) the default +-2g is fine
	 * config = MPU_WriteReg(0x1C, 0x00); // 0x00=+-2g, 0x08=+-4g, 0x10=+-8g, 0x18=+-16g
	 *
	 */
    int16_t rawX, rawY, rawZ;
    uint8_t rawData[6];

    HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, ACCEL_XOUT_H, 1, rawData, 6, HAL_MAX_DELAY);
    rawX = ((int16_t)rawData[0] << 8) | rawData[1];
    rawY = ((int16_t)rawData[2] << 8) | rawData[3];
    rawZ = ((int16_t)rawData[4] << 8) | rawData[5];

    float sensitivity = 16384.0f;  // for +-2g
    float g = 9.81f;
    *ax = (float)rawX / sensitivity * g;
    *ay = (float)rawY / sensitivity * g;
    *az = (float)rawZ / sensitivity * g;
    return;
}

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  //[JT]
  APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");
  UTIL_TIMER_Start(&timerTransmit);  //Transmit packet in 2s
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */
/* USER CODE END PrFD */
