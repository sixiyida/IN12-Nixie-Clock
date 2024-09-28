/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <time.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//时间储存结构�??
typedef struct DateTImeStruct{
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t dayofmonth;
	uint8_t month;
	uint16_t year;
	uint8_t dayOfWeek;  /*Su=0 Mo=1 Tu=3 We=4 Th=5 Fr=6 Sa=7 */
}DateTime;

void Write_74hc595_Test(uint32_t TxData1,uint32_t TxData2);
void Write_Num_UP();
void Write_Num_Time();
uint16_t Random_Num();
uint32_t Bytes_Config(uint32_t n1,uint32_t n2);
uint32_t Num_Select_L(uint16_t num);
uint32_t Num_Select_R(uint16_t num);
uint32_t Time_Check();


//-------------------------------------//
//通用函数原型
//-------------------------------------//
void delay_us(int16_t nus);

//-------------------------------------//
//DS3231相关函数原型
//-------------------------------------//
void DS3231_Init(void);
uint8_t DS3231_setDate(uint8_t year,uint8_t mon,uint8_t day);
uint8_t DS3231_setTime(uint8_t hour , uint8_t min , uint8_t sec);
uint8_t DS3231_getDate(DateTime* ans);
uint8_t DS3231_getTime(DateTime* ans);

//-------------------------------------//
//模拟I2C函数原型
//-------------------------------------//
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_Wait_Ask(void);
void I2C_Ack(void);
void I2C_NAck(void);
void I2C_WriteByte(uint8_t data);
uint8_t I2C_Read_Byte(uint8_t ack);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SER2_Pin GPIO_PIN_3
#define SER2_GPIO_Port GPIOA
#define STCP_Pin GPIO_PIN_4
#define STCP_GPIO_Port GPIOA
#define SHCP_Pin GPIO_PIN_6
#define SHCP_GPIO_Port GPIOA
#define SER1_Pin GPIO_PIN_7
#define SER1_GPIO_Port GPIOA
#define I2C_SCL_Pin GPIO_PIN_11
#define I2C_SCL_GPIO_Port GPIOA
#define I2C_SDA_Pin GPIO_PIN_12
#define I2C_SDA_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LED_ON HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_OFF HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define LED_TURN HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

#define SER1_L HAL_GPIO_WritePin(SER1_GPIO_Port, SER1_Pin, GPIO_PIN_RESET)
#define SER1_H HAL_GPIO_WritePin(SER1_GPIO_Port, SER1_Pin, GPIO_PIN_SET)

#define SER2_L HAL_GPIO_WritePin(SER2_GPIO_Port, SER2_Pin, GPIO_PIN_RESET)
#define SER2_H HAL_GPIO_WritePin(SER2_GPIO_Port, SER2_Pin, GPIO_PIN_SET)

#define STCP_L HAL_GPIO_WritePin(STCP_GPIO_Port, STCP_Pin, GPIO_PIN_RESET)
#define STCP_H HAL_GPIO_WritePin(STCP_GPIO_Port, STCP_Pin, GPIO_PIN_SET)

#define SHCP_L HAL_GPIO_WritePin(SHCP_GPIO_Port, SHCP_Pin, GPIO_PIN_RESET)
#define SHCP_H HAL_GPIO_WritePin(SHCP_GPIO_Port, SHCP_Pin, GPIO_PIN_SET)

//-------------------------------------//
//模拟I2C引脚及操作相关宏定义
//-------------------------------------//

#define I2C_SCL_Pin GPIO_PIN_11
#define I2C_SCL_GPIOx GPIOA
#define I2C_SDA_Pin GPIO_PIN_12
#define I2C_SDA_GPIOx GPIOA

#define I2C_SCL_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define I2C_SCL_H HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
#define I2C_SDA_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)
#define I2C_SDA_H HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET)

//-------------------------------------//
//DS3231相关宏定�?????
//-------------------------------------//
#define DS3231_ADDRESS	      0x68      //I2C 从机地址
#define	DS3231_ADDRESS_Write	0xD0
#define	DS3231_ADDRESS_Read		0xD1

/* DS3231 Registers. Refer Sec 8.2 of application manual */
#define DS3231_SEC_REG        0x00    // �?????
#define DS3231_MIN_REG        0x01    //
#define DS3231_HOUR_REG       0x02
#define DS3231_WDAY_REG       0x03
#define DS3231_MDAY_REG       0x04
#define DS3231_MONTH_REG      0x05
#define DS3231_YEAR_REG       0x06

#define DS3231_AL1SEC_REG     0x07
#define DS3231_AL1MIN_REG     0x08
#define DS3231_AL1HOUR_REG    0x09
#define DS3231_AL1WDAY_REG    0x0A

#define DS3231_AL2MIN_REG     0x0B
#define DS3231_AL2HOUR_REG    0x0C
#define DS3231_AL2WDAY_REG    0x0D

#define DS3231_CONTROL_REG          0x0E
#define DS3231_STATUS_REG           0x0F
#define DS3231_AGING_OFFSET_REG     0x0F
#define DS3231_TMP_UP_REG           0x11
#define DS3231_TMP_LOW_REG          0x12

#define EverySecond     0x01
#define EveryMinute     0x02
#define EveryHour       0x03




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
