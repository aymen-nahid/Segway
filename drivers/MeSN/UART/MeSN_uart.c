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

/* Includes ------------------------------------------------------------------*/
#include "u_uart.h"

/* Public variables */
UART_HandleTypeDef handle_uart;

/* Private define ------------------------------------------------------------*/
#if defined(USE_UART1)
  #define UART_USER     				USART1
	#define UART_TX_PIN						GPIO_PIN_9
	#define UART_RX_PIN						GPIO_PIN_10
	#define GPIO_AF_UART					GPIO_AF4_USART1
	#define UART_PORT							GPIOA
	#define UART_GPIO_SPEED				GPIO_SPEED_LOW;
	#define UART_IRQn							USART1_IRQn
	#define __UART_CLK_ENABLE()		__USART1_CLK_ENABLE()
	#define __UART_CLK_DISABLE()	__USART1_CLK_DISABLE()
	#define TX_DATA_REG						TDR
	#define	STATUS_REG						ISR
	#define	RX_DATA_REG						RDR
	#define	TX_EMPTY_BIT					USART_ISR_TXE
	#define RX_NEMTPY_BIT					USART_ISR_RXNE
#elif defined(USE_UART3)
  #define UART_USER     				USART3
	#define UART_TX_PIN						GPIO_PIN_10
	#define UART_RX_PIN						GPIO_PIN_11
	#define GPIO_AF_UART					GPIO_AF7_USART3
	#define UART_PORT							GPIOC
	#define UART_GPIO_SPEED				GPIO_SPEED_VERY_LOW;
	#define UART_IRQn							USART3_IRQn
	#define __UART_CLK_ENABLE()		__USART3_CLK_ENABLE()
	#define __UART_CLK_DISABLE()	__USART3_CLK_DISABLE()
	#define TX_DATA_REG						DR
	#define	STATUS_REG						SR
	#define	RX_DATA_REG						DR
	#define	TX_EMPTY_BIT					USART_SR_TXE
	#define RX_NEMTPY_BIT					USART_SR_RXNE
#else
  #error "Please select USART instance"
#endif
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
//Define a type dedicated to circular buffer management 
typedef struct {
	volatile uint8_t buffer[UART_BUFFERSIZE];
	volatile uint32_t indexW;
	volatile uint32_t indexR;
	volatile uint32_t eltNb;
} USER_UART_circularBufferTypeDef;

/* Private variables ---------------------------------------------------------*/
// Create a circular buffer dedicated to UART RX management
static USER_UART_circularBufferTypeDef uartRxCircBuff;

/* Private function prototypes -----------------------------------------------*/
static void USER_UART_IRQHandler(void);
static uint8_t USER_Uart_GetChar(void);
static void USER_Uart_PutChar_Poll(uint8_t dataToSend);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief USART init function
  * @param none
  * @retval None
  */
void USER_UART_Init()
{
  /* Configure peripheral */
	handle_uart.Instance = UART_USER;
  handle_uart.Init.BaudRate = 9600;
  handle_uart.Init.WordLength = UART_WORDLENGTH_8B;
  handle_uart.Init.StopBits = UART_STOPBITS_1;
  handle_uart.Init.Parity = UART_PARITY_NONE;
  handle_uart.Init.Mode = UART_MODE_TX_RX;
  handle_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  handle_uart.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&handle_uart);
  
  /* Initialize RX circular buffer and flag */
	uartRxCircBuff.eltNb = 0;
	uartRxCircBuff.indexR = 0;
	uartRxCircBuff.indexW = 0;
		
	/* Enable the UART Data Register not empty Interrupt */
  handle_uart.Instance->CR1 |= USART_CR1_RXNEIE;
}

/**
  * @brief uart sending string by polling
  * @param *stringToSend: pointer to the string to be send
  * @retval none
  */
void USER_Uart_PutString_Poll(uint8_t *stringToSend)
{
	int32_t i = 0;

  //Send data
  for (i=0; stringToSend[i] != '\0'; i++){
		USER_Uart_PutChar_Poll(stringToSend[i] );
	}
}

/**
  * @brief uart receiving string by interruption and circular buffer
  * @param *rxstring : pointer to a buffer where the received data have to be saved 
  */
void USER_Uart_GetString(uint8_t *rxString){
	uint8_t 	tmp = 0;
	uint32_t 	i = 0;
	
	while ( (tmp = USER_Uart_GetChar() ) != '\r'){
		rxString[i++] = tmp;
	}
	
	// replace ENTER key ascii code '\r' by '\0'
  rxString[i] = '\0';
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief ISR dedicated to manage RX IRQ
  * @param none
  */
static void USER_UART_IRQHandler(void)
{
	uint8_t tmp;

	/* check the source of IRQ */
	// Data received IRQ
	if((handle_uart.Instance->STATUS_REG & RX_NEMTPY_BIT) != 0)
	{ 
		//Retrieve received data
		tmp = (uint8_t) handle_uart.Instance->RX_DATA_REG;

		// save data in circular buffer if is not full
		if (uartRxCircBuff.eltNb < UART_BUFFERSIZE) {
			// store data
			uartRxCircBuff.buffer[uartRxCircBuff.indexW] = tmp;

			// circular buffer processing
			uartRxCircBuff.eltNb++;
			uartRxCircBuff.indexW++;
			if (uartRxCircBuff.indexW >= UART_BUFFERSIZE)
				uartRxCircBuff.indexW = 0;
		}
		else {
			// no flow control (hard/soft)
		}
	}
}

/**
	* @brief ISR function according to prototype defined in stm32lxx_startup.s .
	*/
#if defined(USE_UART3)
  void USART3_IRQHandler(void)
  {
    USER_UART_IRQHandler();
  }
#elif defined(USE_UART1)
  void USART1_IRQHandler(void)
  {
		USER_UART_IRQHandler();
  }
#endif

/**
  * @brief sends 8bits payload through UART by polling
  * @param dataTosend: byte to be send
  * @retval none
  */
static void USER_Uart_PutChar_Poll(uint8_t dataToSend)
{
	// check if transmitter is ready to send
	while((handle_uart.Instance->STATUS_REG & TX_EMPTY_BIT) == 0);
	handle_uart.Instance->TX_DATA_REG = (uint8_t)(dataToSend & 0xFF);
}
	
/**
  * @brief check if any data has been received in the circular buffer.
            Circular buffer is feeded by UART RX ISR.
  * @param none
  * @retval received data byte
  */
static uint8_t USER_Uart_GetChar(void) {
	char tmp = 0;

	// wait until there is avalaible data
	while( uartRxCircBuff.eltNb == 0 );
	
	tmp = uartRxCircBuff.buffer[uartRxCircBuff.indexR];

	// circular buffer processing
	uartRxCircBuff.eltNb--;
	uartRxCircBuff.indexR++;
	if ( uartRxCircBuff.indexR >= UART_BUFFERSIZE)
		uartRxCircBuff.indexR = 0;
	return tmp;
}

/**
  * @brief MCU Specific Package (low level) initialisation function.
            Called by HAL library when executing HAL_UART_Init()
  * @param *huart : pointer to uart handle
  * @retval none
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	if(huart->Instance == UART_USER)
	{
		/* Peripheral clock enable */
		__UART_CLK_ENABLE();
	
		/* UART GPIO Configuration */    
		GPIO_InitStruct.Pin = UART_TX_PIN | UART_RX_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = UART_GPIO_SPEED;
		GPIO_InitStruct.Alternate = GPIO_AF_UART;
		HAL_GPIO_Init(UART_PORT, &GPIO_InitStruct);

		/* System interrupt init*/
		HAL_NVIC_SetPriority(UART_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(UART_IRQn);
	}
}

/**
  * @brief MCU Specific Package (low level) de-initialisation function.
            Called by HAL library when executing HAL_UART_DeInit()
  * @param *huart : pointer to uart handle
  * @retval none
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
	if(huart->Instance == UART_USER)
	{
		/* Peripheral clock disable */
		__UART_CLK_DISABLE();
	
		HAL_GPIO_DeInit(GPIOC, UART_TX_PIN | UART_RX_PIN);

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(UART_IRQn);
	}
}

/****END OF FILE****/
