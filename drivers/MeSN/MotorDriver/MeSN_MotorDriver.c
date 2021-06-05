/**
  ******************************************************************************
  * @file    MotorDriver.c
  * @author  Basile Dufay
  * @version V1.0.0
  * @date    18-Sept-2017
  * @brief   This file provides a set of firmware functions to manage:
  *          + Sparkfun ROB-09457 dual Motor Bridge based on TB6612FNG 
	*					 + These functions required 2 GPIO access and 1 PWM generation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <MeSN_MotorDriver_port.h>			//access to portable layer API
#include <MeSN_MotorDriver.h>					//check matching between prototype and implementation

/* Private Typedefs ----------------------------------------------------------*/
typedef enum 
{  
  STOP = 0,		//Stop rotation
  CW = 1,			//Clock wise rotation
  CCW = 2,		//CounterClock wise rotation
}RotDir_TypeDef;

/* Private prototypes --------------------------------------------------------*/
static inline uint32_t 				MeSN_MotorDriverCore_Absolute(int32_t SignedNum);
static inline RotDir_TypeDef 	MeSN_MotorDriverCore_RotDir(int32_t SignedNum);
static void 									MeSN_MotorDriverCore_SetDir(RotDir_TypeDef rotDir);
static void										MeSN_MotorDriverCore_SetSpeed(uint32_t rotSpeed);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Configures Low level hardware for driving TB6612FNG chip.
  * @param  None
  * @retval None
  */
void MeSN_MotorDriver_Init()
{
	/* Init 2 GPIO pins to manage motor direction */
	MotorDriver_Port_GPIO_Init();
	/* Init 1 PWM generation to manage motor speed */
	MotorDriver_Port_PWM_Init();
}


/**
  * @brief  Set motor speed
  * @param  speed: Specifies the motor speed to apply
	*					This parameter must be a value between -MAX_SPEED and +MAX_SPEED
	*					where the sign represents the rotation direction. 
  * @retval None
  */
void MeSN_MotorDriver_Move(int32_t speed)
{
	RotDir_TypeDef RotDir;
	uint32_t RotSpeed;
	
	/* Extract rotation direction and speed*/
	RotSpeed = MeSN_MotorDriverCore_Absolute(speed);
	RotDir = MeSN_MotorDriverCore_RotDir(speed);
	
	/* Check parameters */
	if (RotSpeed > MOTOR_MAX_SPEED){	//Speed value exceeds limits
		RotSpeed = MOTOR_MAX_SPEED;
	}
	
	/* Apply motor action */
	MeSN_MotorDriverCore_SetDir(RotDir);
	MeSN_MotorDriverCore_SetSpeed(RotSpeed);
	
}

/**
  * @brief  Stop motor
  * @param  None
  * @retval None
  */
void MeSN_MotorDriver_Stop(void)
{
	/* Apply motor action */
	MeSN_MotorDriverCore_SetDir(STOP);
	MeSN_MotorDriverCore_SetSpeed(0);
}


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Return absolute value of a signed number.
  * @param  SignedNum: signed number from which to extract absolute value
  * @retval Unsigned value which is the absolute value of the entrant parameter
  */
static inline uint32_t MeSN_MotorDriverCore_Absolute(int32_t SignedNum)
{
/* compute absolute value of int argument */
return (unsigned) (SignedNum < 0 ? -SignedNum : SignedNum);
}

/**
  * @brief  Return Rotation direction according to the speed sign.
  * @param  SignedNum: signed number from which to extract absolute value
  * @retval RotDir_TypeDef value which is the sign of the entrant parameter
  */
static RotDir_TypeDef MeSN_MotorDriverCore_RotDir(int32_t SignedNum)
{
	RotDir_TypeDef dir = STOP;
	
	if (SignedNum == 0){
		dir = STOP;
	}
	else if (SignedNum > 0){
		dir = CW;
	}
	else if (SignedNum < 0) {
		dir = CCW;
	}
	
	return dir;
}

/**
  * @brief  Set motor control pin according to rotation direction.
  * @param  rotDir: rotation direction
	*   This parameter can be one of following parameters:
	*     @arg CW	:	clock wise
	*			@arg CCW : counterColck wise
  * @retval none
  */
static void MeSN_MotorDriverCore_SetDir(RotDir_TypeDef rotDir){
	/* Always set pin low first before inverting direction to ensure
	   a "brake to ground" state during transition */
	if (rotDir == CW){
		MotorDriver_Port_SetPin_IN2(GPIO_DIRPIN_LOW);
		MotorDriver_Port_SetPin_IN1(GPIO_DIRPIN_HIGH);
	}
	else if (rotDir == CCW){
		MotorDriver_Port_SetPin_IN1(GPIO_DIRPIN_LOW);
		MotorDriver_Port_SetPin_IN2(GPIO_DIRPIN_HIGH);
	}
	else if (rotDir == STOP){
		MotorDriver_Port_SetPin_IN2(GPIO_DIRPIN_LOW);
		MotorDriver_Port_SetPin_IN1(GPIO_DIRPIN_LOW);
	}
}

/**
  * @brief  Set motor PWM duty cycle according to speed parameter.
  * @param  rotSpeed: desired rotation speed
	*   This parameter must be a positive value between 0 and MAX_SPEED
  * @retval none
  */
static void MeSN_MotorDriverCore_SetSpeed(uint32_t rotSpeed){
	MotorDriver_Port_SetPWM( (uint32_t) (rotSpeed * PWM_DUTYCYCLE_FULL_SCALE / MOTOR_MAX_SPEED) );
}

/**********END OF FILE****/
