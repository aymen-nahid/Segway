/**
  ******************************************************************************
  * @file    MotorDriver_port.h
  * @author  
  * @version V1.0.0
  * @date    
  * @brief   This file contains function prototypes and pinout assignation of
	*					 portable layer to manage low level operations of the motor bridge.
  ******************************************************************************
  */
	
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MotorDriver_Port_H
#define __MotorDriver_Port_H

/* Includes ------------------------------------------------------------------*/
//A completer
#include "stm32l1xx_hal.h"		//Needed to access HAL API.

/* Exported Typedef-----------------------------------------------------------*/
typedef enum
{
  GPIO_DIRPIN_LOW = 0,
  GPIO_DIRPIN_HIGH
}GPIO_DirPinState;

/* Exported Constants---------------------------------------------------------*/
/*	Motor Pinout:
		Motor direction IN1 --> PB14 : output PP
		Motor direction IN2 --> PB13 : output PP
		MotorD PWM out --> PB12 	(Timer 10 channel 1)
*/
#define MOTOR_DIR_PORT				GPIOB
#define MOTOR_PWM_PORT				GPIOB

#define MOTOR_PWM_PIN				GPIO_PIN_12
#define MOTOR_IN1_PIN				GPIO_PIN_14
#define MOTOR_IN2_PIN				GPIO_PIN_13

#define MOTOR_PWM_TIM_INSTANCE		TIM10
#define MOTOR_PWM_TIM_CHANNEL		TIM_CHANNEL_1

#define MOTOR_DIR_PORT_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define MOTOR_PWM_PORT_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define MOTOR_PWM_TIM_CLK_ENABLE()	__HAL_RCC_TIM10_CLK_ENABLE()

#define PWM_FREQ					20000				//Desired PWM frequency
#define PWM_SRC_FREQ				32000000			//Timer counting frequency
#define TIMER_MAX_VAL				0xFFFF

#define TIMER_RELOAD_VAL			(uint16_t) ( PWM_SRC_FREQ / PWM_FREQ )	//Number of count before overflow !! Must be <= TIMER_MAX_VAL
#define PWM_DUTYCYCLE_FULL_SCALE		(uint16_t) ( TIMER_RELOAD_VAL )		//100% duty cycle value

/* Exported Fonctions---------------------------------------------------------*/

/**
	* @brief  Configures GPIO pins dedicated to motor direction:
	*         2 pins are required, in pushpull output mode.
  * @param  None
  * @retval None
  */
void MotorDriver_Port_GPIO_Init(void);

/**
	* @brief  Configures Timer dedicated to PWM generation:
	*					1 PWM output pin are required,
	*					maximum PWM frequency is 100kHz.
  * @param  None
  * @retval None
  */
void MotorDriver_Port_PWM_Init(void);


/**
	* @brief  Set direction pin value, IN1,
	*         according to parameter value
	* @param  pinState : GPIO_DIRPIN_LOW or GPIO_DIRPIN_HIGH
  * @retval None
  */
void MotorDriver_Port_SetPin_IN1(GPIO_DirPinState pinState);

/**
	* @brief  Set direction pin value, IN2,
	*         according to parameter value
	* @param  pinState : GPIO_DIRPIN_LOW or GPIO_DIRPIN_HIGH
  * @retval None
  */
void MotorDriver_Port_SetPin_IN2(GPIO_DirPinState pinState);


/**
	* @brief  Set PWM duty cycle on dedicated motor pin
	* @param  dutyCycle : desired duty cycle.
	*         Must be a number between 0 and PWM_DUTYCYCLE_FULL_SCALE
  * @retval None
  */
void MotorDriver_Port_SetPWM(uint32_t dutyCycle);

#endif /*__MotorDriver_Port_H */

/**********END OF FILE****/
