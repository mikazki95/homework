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
#include <stdlib.h>
//#include "usbd_cdc_if.h"
//#include <arm_math.h>
#include <stdbool.h>
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
extern uint8_t flag_Muestreo;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
int derv_norm_I=0;
int32_t aus_data=0;

#define M_PI 3.14159265358979323846
#define SAMPLE_RATE 441000
float cutoff = 4000.0;
float prevFiltered_II = 0.0;
int32_t filteredData_II=0;
//uint8_t dato_tx[10]={0};

uint8_t msgUSB[16]={0};
uint16_t msgUsb_cnt;


/* USER CODE END Variables */
osThreadId LeerDACHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int32_t DerivadaNormFuncion_I (int derivacion);
/* USER CODE END FunctionPrototypes */

void tarea1(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of LeerDAC */
  osThreadDef(LeerDAC, tarea1, osPriorityNormal, 0, 512);
  LeerDACHandle = osThreadCreate(osThread(LeerDAC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_tarea1 */
/**
  * @brief  Function implementing the LeerDAC thread.
  * @param  argument: Not used
  * @retval None
  */
float filter(float cutofFreq);
/* USER CODE END Header_tarea1 */
void tarea1(void const * argument)
{
  /* USER CODE BEGIN tarea1 */
  /* Infinite loop */
//uint16_t i = 0;

//uint8_t msgUSB[16]={0};
//uint16_t msgUsb_cnt;
/*
uint16_t datoLeido;
uint16_t ads_delta;
uint16_t datoLeido_ant;
*/
int16_t datoLeido;
int16_t ads_delta;
int 	ads_array[100]={0};
int 	index_array=0;
int 	max_i_array =19;
int16_t datoLeido_ant;
//int16_t flagPACE;
//int flag_pacemaker 	= 0;
int cont_delta=0;
uint8_t flag_delta=0;
//Calibración de inicio del ADS7042, mínimo 16 ciclos de reloj, pongo32
HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_RESET);
HAL_SPI_Receive(&hspi1, (uint8_t*)&datoLeido, 1, 1);
HAL_SPI_Receive(&hspi1, (uint8_t*)&datoLeido, 1, 1);
osDelay(1);
HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_SET);

  for(;;)
  {

	  //HAL_GPIO_TogglePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin);
	  //osDelay(500);
	  //Calibración de inicio del ADS7042, mínimo 16 ciclos de reloj, pongo32
//	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_RESET);
//	  HAL_SPI_Receive(&hspi1, (uint8_t*)&datoLeido, 1, 1);
//	  HAL_SPI_Receive(&hspi1, (uint8_t*)&datoLeido, 1, 1);
//	  osDelay(1);
//	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_SET);

	  if(flag_Muestreo)
	  {
		  //Test Bandera cada 5ms
		  //Enviar valor por USB
		  flag_Muestreo = 0; //borrar bandera
		  HAL_GPIO_TogglePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin);

		  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_RESET);
		  HAL_SPI_Receive(&hspi1, (uint8_t*)&datoLeido, 1,0);
		  //HAL_SPI_Receive(&hspi1, (uint8_t*)&datoLeido, 1, 0);
		  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_SET);

		  float filtered_II = prevFiltered_II + (filter(cutoff) * ((float)datoLeido - prevFiltered_II));

		  filteredData_II= (int32_t)filtered_II;
		  prevFiltered_II = filtered_II;
		  //aus_data = filteredData_norm_II;

		  derv_norm_I=abs(DerivadaNormFuncion_I(datoLeido));

		  index_array++;
		  if (index_array>max_i_array)
		  {
			  index_array = 0;
			  //HAL_GPIO_TogglePin(Test_pin0_GPIO_Port, Test_pin0_Pin);
		  }
		  ads_array[index_array]=datoLeido;
		  if(index_array<max_i_array)
		  {
			  ads_delta=abs((ads_array[index_array+1]-datoLeido)/10);
		  }
		  else
		  {
			  ads_delta=abs((ads_array[0]-datoLeido)/10);
		  }
		  //ads_delta=(datoLeido-datoLeido_ant)/10;
		  datoLeido_ant=datoLeido;
		  /*if(ads_delta>250)
		  {
			  HAL_GPIO_WritePin(Test_pin0_GPIO_Port, Test_pin0_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(PULSO_MC_GPIO_Port, PULSO_MC_Pin, GPIO_PIN_SET);
			  msgUsb_cnt = sprintf((char*) msgUSB,"%i,%i,%i,1\r\n",datoLeido, derv_norm_I,ads_delta);
			  //msgUsb_cnt = sprintf((char*) msgUSB,"%i\r",datoLeido);
			  //msgUsb_cnt = sprintf((char*) msgUSB,"%i\r",derv_norm_I);
			  //HAL_UART_Transmit_DMA(&huart2, msgUSB, msgUsb_cnt);
		  }
		  else
		  {
			  HAL_GPIO_WritePin(Test_pin0_GPIO_Port, Test_pin0_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(PULSO_MC_GPIO_Port, PULSO_MC_Pin, GPIO_PIN_RESET);
			  msgUsb_cnt = sprintf((char*) msgUSB,"%i,%i,%i,0\r\n",datoLeido, derv_norm_I,ads_delta);
			  //msgUsb_cnt = sprintf((char*) msgUSB,"%i\r",datoLeido);
			  //msgUsb_cnt = sprintf((char*) msgUSB,"%i\r",derv_norm_I);
			  //HAL_UART_Transmit_DMA(&huart2, msgUSB, msgUsb_cnt);
		  }*/
		  /*
		  msgUsb_cnt = sprintf((char*) msgUSB,"%i,%i,1\r\n",datoLeido,ads_delta);

		  HAL_UART_Transmit_DMA(&huart2, msgUSB, msgUsb_cnt);
*/
		 // HAL_UART_Transmit_DMA(&huart2, dato_tx, 6);

		  //if (ads_delta>50) //se confunde dentro de pulsos
		  //if (datoLeido>4550) // se confunde por amplitud
		  if (ads_delta>250)
		  {
			  //HAL_GPIO_TogglePin(PULSO_MC_GPIO_Port, PULSO_MC_Pin);
			  flag_delta++;
		  }
		  else
		  {
			  flag_delta=0;
		  }

		  if (flag_delta>6)
		  {
			  //flag_delta=1;
			  cont_delta=55;
			  HAL_GPIO_WritePin(Test_pin0_GPIO_Port, Test_pin0_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(PULSO_MC_GPIO_Port, PULSO_MC_Pin, GPIO_PIN_SET);
			  //HAL_GPIO_TogglePin(PULSO_MC_GPIO_Port, PULSO_MC_Pin);
			  //HAL_GPIO_TogglePin(Test_pin0_GPIO_Port, Test_pin0_Pin);
		  }
		  else
		  //else if (cont_delta>32)
		  {
			  cont_delta=0;
			  HAL_GPIO_WritePin(Test_pin0_GPIO_Port, Test_pin0_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(PULSO_MC_GPIO_Port, PULSO_MC_Pin, GPIO_PIN_RESET);
		  }

		  msgUsb_cnt = sprintf((char*) msgUSB,"%i,%i,%i\r\n",datoLeido,flag_delta,cont_delta);

		  HAL_UART_Transmit_DMA(&huart2, msgUSB, msgUsb_cnt);

		  //Comprobar valor umbral y disparar bandera en caso necesario
	  }





    //osDelay(1);
  }
  /* USER CODE END tarea1 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
int32_t DerivadaNormFuncion_I (int derivacion)
{
	/*
    int y, i;
    static int fx_derv[4];
    y = (derivacion << 1) + fx_derv[3] - fx_derv[1] - (fx_derv[0] << 1);
    y >>= 3;
    for (i = 0; i < 3; i++)
        fx_derv[i] = fx_derv[i + 1];
    fx_derv[3] = derivacion;

    return(y);
    */
	int y, i;
	static int fx_derv[4];
	y = (derivacion << 1) + fx_derv[3] - fx_derv[1] - (fx_derv[0] << 1);
	y >>= 3;
	for (i = 0; i < 3; i++)
		fx_derv[i] = fx_derv[i + 1];
	fx_derv[3] = derivacion;

	return(y);

}
float filter(float cutofFreq)
{
    float RC = 1.0 / (cutofFreq * 2 * M_PI);
    float dt = 1.0 / SAMPLE_RATE;
    float alpha = dt / (RC + dt);
    return alpha;
}
/* USER CODE END Application */

