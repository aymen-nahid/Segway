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
#include "MeSN_uart.h"
#include "cmsis_os.h"

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
	#define UART_PORT						GPIOC
	#define UART_GPIO_SPEED					GPIO_SPEED_VERY_LOW;
	#define UART_IRQn						USART3_IRQn
	#define __UART_CLK_ENABLE()				__USART3_CLK_ENABLE()
	#define __UART_CLK_DISABLE()			__USART3_CLK_DISABLE()
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
/* Private variables ---------------------------------------------------------*/
//static USER_UART_circularBufferTypeDef uart1RxCircBuff;	// Create a circular buffer dedicated to UART1 RX management
static osMutexId uartTxSem; 	// Mutex ID
static osMutexDef(uartTxSem); 	// Mutex definition
static osMessageQDef(uartRxQ, UART_BUFFERSIZE, char); // Define message queue
static osMessageQId uartRxQ_ID;

/* Private function prototypes -----------------------------------------------*/
static void MeSN_UART_IRQHandler(void);
static MeSN_StatusTypedef MeSN_Uart_GetChar(uint8_t* rxData, uint32_t timeOut);
static void MeSN_Uart_PutChar_Poll(uint8_t dataToSend);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief USART init function
  * @param none
  * @retval None
  */
void MeSN_UART_Init()
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
  
	/* Create mutex for protecting concurrency on uart Tx */
	uartTxSem = osMutexCreate(osMutex(uartTxSem));

	/* Create RX queue between RX ISR and task */
	uartRxQ_ID = osMessageCreate(osMessageQ(uartRxQ), NULL);
		
	/* Enable the UART Data Register not empty Interrupt */
	handle_uart.Instance->CR1 |= USART_CR1_RXNEIE;
}

/**
  * @brief uart sending string by polling
  * @param *stringToSend: pointer to the string to be send
  * @retval none
  */
void MeSN_Uart_PutString_Poll(uint8_t *stringToSend)
{
	/* wait for uart tx to be free */
	osMutexWait(uartTxSem, osWaitForever);

	/* send string */
	while(*stringToSend != '\0')
	{
		MeSN_Uart_PutChar_Poll(*stringToSend);
		stringToSend++;
	}

	/* free uart tx */
	osMutexRelease(uartTxSem);
}

/**
 * @brief uart receiving string by interruption and circular buffer
 * @param *rxstring : pointer to a buffer where the received data have to be saved
 * @param timeout : amount of time for which the function should wait a data
 */
MeSN_StatusTypedef MeSN_Uart_GetString(uint8_t *rxString, uint32_t timeOut){

	MeSN_StatusTypedef retVal = USER_OK;

	retVal = MeSN_Uart_GetChar(rxString, timeOut);

	while ( (*rxString  != '\r') && (retVal == USER_OK) ){
		rxString++;
		retVal = MeSN_Uart_GetChar(rxString, timeOut);
	}
	
	/* replace ENTER key ascii code '\r' by '\0' */
	if (*rxString  == '\r'){
		*rxString = '\0';
	}
	
	return retVal;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief ISR dedicated to manage RX IRQ
  * @param none
  */
static void MeSN_UART_IRQHandler(void)
{
	uint8_t tmp;

	/* check the source of IRQ */
	// Data received IRQ
	if((handle_uart.Instance->STATUS_REG & RX_NEMTPY_BIT) != 0)
	{ 
		//Retrieve received data
		tmp = (uint8_t) handle_uart.Instance->RX_DATA_REG;

		// save data in dedicated message queue
		if(osMessagePut(uartRxQ_ID, tmp, 0) != osOK ){
			/* no flow control (hard/soft) */
		}
	}
}

/**
	* @brief ISR function according to prototype defined in stm32lxx_startup.s .
	*/
#if defined(USE_UART3)
  void USART3_IRQHandler(void)
  {
    MeSN_UART_IRQHandler();
  }
#elif defined(USE_UART1)
  void USART1_IRQHandler(void)
  {
		MeSN_UART_IRQHandler();
  }
#endif

/**
  * @brief sends 8bits payload through UART by polling
  * @param dataTosend: byte to be send
  * @retval none
  */
static void MeSN_Uart_PutChar_Poll(uint8_t dataToSend)
{
	// check if transmitter is ready to send
	while((handle_uart.Instance->STATUS_REG & TX_EMPTY_BIT) == 0);
	handle_uart.Instance->TX_DATA_REG = (uint8_t)(dataToSend & 0xFF);
}
	
/**
  * @brief check if any data has been received in the queue.
            Queue is feeded by UART RX ISR.
  * @param *rxData : pointer to a buffer which will store the received data
  * @param timeout : amount of time that the function should wait a data before returning
  * @retval received data byte
  */
static MeSN_StatusTypedef MeSN_Uart_GetChar(uint8_t* rxData, uint32_t timeOut) {
	osEvent		event;
	MeSN_StatusTypedef retVal = USER_OK;

	/* wait for avalaible data */
	event = osMessageGet(uartRxQ_ID, timeOut);
	if (event.status == osEventMessage){
		*rxData = event.value.v;
		retVal = USER_OK;
	}
	else {
		*rxData = 0;
		retVal = USER_TIMEOUT;
	}
	
	return retVal;
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
		HAL_NVIC_SetPriority(UART_IRQn, 10, 0);
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
