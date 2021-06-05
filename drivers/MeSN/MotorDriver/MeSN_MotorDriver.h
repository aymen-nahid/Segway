/**
  ******************************************************************************
  * @file    MotorDriver.h
  * @author  Basile Dufay
  * @version V1.0.0
  * @date    18-Sept-2017
  * @brief   This file contains all the functions prototypes for driver layer.
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MotorDriver_H
#define __MotorDriver_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"		/* Portable definition of C integer type */

/* Exported TypeDefs -------------------------------------------------------- */

/* Exported Constants---------------------------------------------------------*/
#define MOTOR_MAX_SPEED 				1000

/* Exported public Fonctions--------------------------------------------------*/

/**
  * @brief  Configures Low level hardware for driving MD03A chip.
  * @param  None
  * @retval None
  */
void MeSN_MotorDriver_Init(void);

/**
  * @brief  Set motor speed
  * @param  speed: Specifies the motor speed to apply
	*					This parameter must be a value between -MAX_SPEED and +MAX_SPEED
	*					where the sign represents the rotation direction. 
  * @retval None
  */
void MeSN_MotorDriver_Move(int32_t speed);

/**
  * @brief  Stop motor
  * @param  None
  * @retval None
  */
void MeSN_MotorDriver_Stop(void);

#endif /*__MotorDriver_H */

/**********END OF FILE****/
