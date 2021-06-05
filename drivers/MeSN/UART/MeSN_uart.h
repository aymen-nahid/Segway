/**
  ******************************************************************************
  * @file    u_uart.c
  * @author  BDufay
  * @version 
  * @date    2016-01
  * @brief   Uart lib for STM32Lxx.  
  *          This file provides firmware functions to manage the UART
  *          It is designed to handle both USART1 or USART3 periph on STM32L0 
  *          and STM32L1 MCUs respectively. Both are used for LicPro praticals
  *          and MeSN2A practicals. Selecting the appropriate MCU have to be
  *          done in u_uart.h header file.
  ******************************************************************************  
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_UART_H
#define __USER_UART_H

// Choose between USART1 (STM32L0 for LicPro practicals) or USART3 (STM32L1 for MeSN practicals)
//#define		USE_UART1
#define		USE_UART3

/* Includes ------------------------------------------------------------------*/
#include "errorStatus.h"
#if defined(USE_UART1)
  #include "stm32l0xx_hal.h"
#elif defined(USE_UART3)
  #include "stm32l1xx_hal.h"
#else
  #error "Please select USART instance"
#endif

/* Exported constants --------------------------------------------------------*/
#define UART_BUFFERSIZE 	256

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported globals var ------------------------------------------------------*/
extern UART_HandleTypeDef handle_uart;
/* Exported functions --------------------------------------------------------*/
void MeSN_UART_Init();
void MeSN_Uart_PutString_Poll(uint8_t *stringToSend);
MeSN_StatusTypedef MeSN_Uart_GetString(uint8_t *rxString, uint32_t timeOut);

#endif /* __USER_UART_H */
