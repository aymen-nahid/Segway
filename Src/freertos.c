/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "MeSN_uart.h"
#include "MeSN_i2c.h"
#include "lsm6ds3.h"
#include "MeSN_MotorDriver.h"
#include "regulation.h"
#include "Stdio.h"
#include "String.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/


/* USER CODE BEGIN Variables */
// Creation d'un tableau circulaire
#define BUFFERSIZE 100
typedef struct {
	volatile uint8_t buffer[BUFFERSIZE];
	volatile uint32_t indexW;
	volatile uint32_t indexR;
	volatile uint32_t eltNb;
} tab_circularBufferTypeDef;
tab_circularBufferTypeDef tab_angle;
uint8_t Accel_gyro;
#define TIMEOUT 200 					
#define TIMEOUT_CAPTEUR 5 
	

// définition des queues de messages :
	osMessageQId(angle_Q_ID);     // Queue entre Taskregulation et Taskenregistrement
	osMessageQDef(angle_Q,50,uint32_t);
	osMessageQId(Queue_courante);
	osMessageQDef(Queue_courante,50,uint32_t);
// définition des Mutex

	osMutexId(Mutex_GPIOB);
	osMutexId(Mutex_tableau);
	

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

void interfaceAffichage(void);
void help();
void Task_regu(void const * pvParameters);
void Task_enreg(void const * pvParameters);
void Task_uart(void const * pvParameters);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	while(1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	while(1);
}
/* USER CODE END 5 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
/* init circularBuffer */
        tab_angle.eltNb = BUFFERSIZE;
        tab_angle.indexR = 0;
        tab_angle.indexW = 0;
MeSN_UART_Init();
MeSN_MotorDriver_Init();


//MeSN_Uart_PutString_Poll((uint8_t*)"PROJET ROBOT EQULIBRISTE\n");
//MeSN_Uart_PutString_Poll((uint8_t*)"Projet realisé par :- EL BOUMTIRI SAAD\n");
//MeSN_Uart_PutString_Poll((uint8_t*)"	            - EL HASSANI Salim\n");
//MeSN_Uart_PutString_Poll((uint8_t*)"	     	    - EL KHADRI Safouane\n");
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */

	  osMutexDef(Mutex_GPIOB);
		Mutex_GPIOB = osMutexCreate(osMutex(Mutex_GPIOB));
	  osMutexDef(Mutex_tableau);
		Mutex_tableau= osMutexCreate(Mutex_tableau);
  /* USER CODE END RTOS_MUTEX */

  /* Create the thread(s) */
	osThreadDef(TASKREGU,Task_regu, osPriorityHigh, 0, 128);
	osThreadCreate (osThread(TASKREGU), NULL);
	
	osThreadDef(TASKENREG,Task_enreg, osPriorityNormal, 0, 128);
	osThreadCreate (osThread(TASKENREG), NULL);
	
	osThreadDef(TASKUART,Task_uart, osPriorityLow, 0, 128);
	osThreadCreate (osThread(TASKUART), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  angle_Q_ID = osMessageCreate(osMessageQ(angle_Q),NULL);
	Queue_courante = osMessageCreate(osMessageQ(Queue_courante),NULL);
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Application */
void Task_regu(void const * pvParameters){
	TickType_t tick = osKernelSysTick();
	uint32_t accX,rotAngY;
	
	LSM6DS3_begin(&Accel_gyro);
	while(1){

		// Récupération de la valeur d'accéléromètre 
   LSM6DS3_readMgAccelX(&accX);	// accélération mesurée sur l'ax X en milli-g	
// Récupération de la valeur de gyroscope
 LSM6DS3_readMdpsGyroY(&rotAngY ); // vitesse angulaire mesurée sur l'axe Y en milli-deg/s				
// Interroger les capteurs : appeler la fonction de calcul d'angle
 uint32_t Angle_mDeg;
 Angle_mDeg = autoAlgo_angleObs(accX,rotAngY);
// Envoie de la valeur de l'angle	
osMessagePut(angle_Q_ID, Angle_mDeg,0);			
// appeler la fonction regulation
uint32_t vitesse_angulaire_Y = autoAlgo_commandLaw(Angle_mDeg,rotAngY);		
		osMutexWait(Mutex_GPIOB,osWaitForever);
// application de la commande 
MeSN_MotorDriver_Move(vitesse_angulaire_Y);
		osMutexRelease(Mutex_GPIOB);
// périodicité de 10ms
		osDelayUntil(&tick,10);}
} 
void Task_enreg(void const * pvParameters){
	osEvent msg_angle;

	float32_t yangle = 0;
	TickType_t Tiktik = osKernelSysTick();
	while(1){
		/* Attente Queue Message */
		msg_angle =osMessageGet(angle_Q_ID,osWaitForever);
		if(msg_angle .status == osEventMessage){
			osMutexWait(Mutex_tableau,osWaitForever);
			tab_angle.buffer[tab_angle.indexR] =msg_angle.value.v;
      tab_angle.eltNb --;
      tab_angle.indexR++;
			if (tab_angle.indexR == BUFFERSIZE)
			{    tab_angle.indexR = 0;}
		// enregistreent de la valeur d'angle 
		// prendre le mutex_tableau pour accéder au tableau
		osMutexRelease(Mutex_tableau);
		osMessagePut(Queue_courante,msg_angle.value.v,0);
			yangle=msg_angle.value.v;
		}
		if((int32_t)(yangle/1000) <= 25||(int32_t)(yangle/1000) >= -25){
			int k=0;
			while(k<5){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
				osDelay(100);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				osDelay(100);
			k++;
				}
		 
		}
	else{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
				
		}
	osDelayUntil(&Tiktik,1050);
	}
	
}

void Task_uart(void const * pvParameters){

	char angle[10], tmp[10];
	MeSN_StatusTypedef msgEvent;
	osEvent msgStream;
	uint8_t	str[100];
	
	int etat ;
	
	//Affichage du menu de l'application
	interfaceAffichage();

	while(1)
	{
		/* Enregistrement data */
		msgEvent = MeSN_Uart_GetString((uint8_t*)tmp,osWaitForever);
		
		if(strncmp(tmp,"read",4)==0){
			etat =0;
		}
		else if(strncmp(tmp,"dump",4)==0){
		  etat=1;
		}
		else if(strncmp(tmp,"stream",6)==0){
			etat=2;
		}
		else if(strncmp(tmp,"help",4)==0){
			etat=3;	
		}
		
		switch(etat)
		{
			case 0 :
				//Affichage de la derniere valeur de l'angle
				//Prise du mutex pour acces aux resource partagé
				osMutexWait(Mutex_tableau, osWaitForever);
				// charger l'angle en millidegré dans la caractére tmpStr
				sprintf(angle,"%d ",(uint32_t)tab_angle.buffer[tab_angle.indexR]/1000);
			//Liberation du Mutex fin de la section critique
				osMutexRelease(Mutex_tableau);
				// transmettre le caratére vers l'afficheur
		  	MeSN_Uart_PutString_Poll((uint8_t*)"\n\r");
			  MeSN_Uart_PutString_Poll((uint8_t*)angle);
				MeSN_Uart_PutString_Poll((uint8_t*)"\n\r");
			break;
			case 1 :
				//Affichage des 100 dernieres valeurs d'angle
				for(int i=0;i<BUFFERSIZE;i++)
				{//Prise du mutex pour acces aux resource partagé
					osMutexWait(Mutex_tableau, osWaitForever);
					sprintf((char*)str,"%d   \r",(uint32_t)tab_angle.buffer[i]/1000);
					//Liberation du Mutex fin de la section critique
					osMutexRelease(Mutex_tableau);
					MeSN_Uart_PutString_Poll(str);
					MeSN_Uart_PutString_Poll((uint8_t*)"\n\r");
				}
				break;
			case 2: // commande stream
				//Affichage l'angle courante
				{
					do
					{
						msgStream = osMessageGet(Queue_courante,osWaitForever);
						sprintf((char*)str,"%d  ",(uint32_t)msgStream.value.v/1000);
						MeSN_Uart_PutString_Poll(str);
					}while(strncmp((char*)tmp,"x",1) != 0);
				}
				break;
			case 3:
				help();
				interfaceAffichage();
				break;
			default :
				MeSN_Uart_PutString_Poll((uint8_t*)"La commande est introuvable");
				break;
		}
	}
}

void help(){
	MeSN_Uart_PutString_Poll((uint8_t*)"-help- : affiche le menu des possibilites \n\r");
	MeSN_Uart_PutString_Poll((uint8_t*)"\n\r");
}
void interfaceAffichage()
{
	MeSN_Uart_PutString_Poll((uint8_t*)"\n\r");
	MeSN_Uart_PutString_Poll((uint8_t*)"-help- : affiche le menu des possibilites \n\r");
	MeSN_Uart_PutString_Poll((uint8_t*)"-read- : la derniere valeur mesuree de l'angle \n\r");
	MeSN_Uart_PutString_Poll((uint8_t*)"-dump- : renvoi les 100 dernieres valeurs mesurees de l'angle \n\r");
	MeSN_Uart_PutString_Poll((uint8_t*)"-stream- : renvoi en continu la derniere valeur mesuree de l'angle \n\r");
	MeSN_Uart_PutString_Poll((uint8_t*)"\n\r");
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
