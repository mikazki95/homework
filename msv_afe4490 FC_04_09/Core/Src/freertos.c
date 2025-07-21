/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * Descripción        : Código de FreeRtos para el monitor de signos vitales
 *                      (módulo SMT32f103 de oximetría).
 *
 * Comandos Disponibles:
 *      S: Iniciar.
 *      E: Detener.
 *      I: solicitar información de firmware.
 *
 * Lista de cambios:
 *
 * 1.0.0 - Primer versión estable.
 * 1.0.1 - Primer optimización de código.
 * 1.0.2 - Se agregan definiciones para los comandos.
 * 1.1.0 - Se agrega buffer en la salida de oxigenación para evitar cambios bruscos en el valor calculado.
 * 1.1.1 - Se agrega buffer en la salida de fecuencia cardiaca para evitar cambios bruscos en el valor calculado.
 * 1.1.2 - Ajuste en el manejo de los buffers de oxigeneación y frecuencia cardiaca
 * 			Se envia aviso de detección de sonda: 	O:0 -> no se detecta la sonda
 * 													O:1 -> no se detecta el dedo
 *													O:2 -> estado correcto midiendo
 *
 *
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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    FALSE = 0u, TRUE
} boolean;

typedef enum
{
    CONTROL0 = 0x00,
    LED2STC,
    LED2ENDC,
    LED2LEDSTC,
    LED2LEDENDC,
    ALED2STC,
    ALED2ENDC,
    LED1STC,
    LED1ENDC,
    LED1LEDSTC,
    LED1LEDENDC,
    ALED1STC,
    ALED1ENDC,
    LED2CONVST,
    LED2CONVEND,
    ALED2CONVST,
    ALED2CONVEND,
    LED1CONVST,
    LED1CONVEND,
    ALED1CONVST,
    ALED1CONVEND,
    ADCRSTCNT0,
    ADCRSTENDCT0,
    ADCRSTCNT1,
    ADCRSTENDCT1,
    ADCRSTCNT2,
    ADCRSTENDCT2,
    ADCRSTCNT3,
    ADCRSTENDCT3,
    PRPCOUNT,
    CONTROL1,
    SPARE1,
    TIAGAIN,
    TIA_AMB_GAIN,
    LEDCNTRL,
    CONTROL2,
    SPARE2,
    SPARE3,
    SPARE4,
    RESERVED1,
    RESERVED2,
    ALARM,
    LED2VAL,
    ALED2VAL,
    LED1VAL,
    ALED1VAL,
    LED2ABSVAL,
    LED1ABSVAL,
    DIAG
} afeRegisters;

typedef struct
{
        int32_t heart_rate;
        int32_t spo2;
        int32_t ir;
        int32_t red;
        boolean calculated_value;
} afe44xx_output_values;

typedef union
{
        struct
        {
                uint8_t b;
                uint8_t r;
                uint8_t g;
        } color;
        uint32_t data;
} PixelRGB_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VERSION_FIRMWARE "1.1.2"
#define ADC_VMAX 4095.0         // 2^12-1 Bits
#define ADC_VREF  2.5       //Voltaje de referencia (en volts)
#define RES_PULLUP_TEMPERATURA 10000
#define V_POWER  3.3

#define CMD_INICIAR 'S'
#define CMD_DETENER 'E'
#define CMD_INFO    'I'
#define CMD_PING    'P'

#define FS            25    //sampling frequency
#define BUFFER_SIZE  (FS*4)
#define MA4_SIZE  4 // DONOT CHANGE
#define NUMERO_MUESTRAS_TEMPERATURA 64

#define	TOL_FC			25
#define	LIM_SUP_SONDA	4190000
#define	LIM_INF_SONDA	5000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define min(x,y) ((x) < (y) ? (x) : (y))
//#define millis() (absoluteElapsedTime)
#define millis() HAL_GetTick ()
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/// Dispositivos
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

/// Variables generales
uint8_t versionFirmware[] = VERSION_FIRMWARE;

uint8_t msgBuff[40] =
        { 0 };
uint8_t msgCount;
//uint8_t rxBuff[15] = { 0 };
uint8_t rxByte;
//uint8_t rxCnt = 0;

extern bool flag_4ms;
boolean solicitarInfo = FALSE;
boolean monitoreoActivo = FALSE;
boolean flag_error = FALSE;
boolean	flag_Ping = FALSE;
boolean flag_Start = FALSE ;
uint16_t count_Dato = 0;

boolean	flag_500ms = FALSE;
boolean flag_init_p = FALSE;

uint16_t buffMuestrasTemperatura[NUMERO_MUESTRAS_TEMPERATURA]; //2.844V - 36º
uint16_t buffMuestrasVTotal[NUMERO_MUESTRAS_TEMPERATURA]; //2.844V - 36º

float temperatura;

int32_t val_pleth_Grafica = 0;
int32_t val_pleth = 0;

int afe_oxigenacion = 0;         // Valor leído de spo2 previamente guardado
int afe_frecuenciaCardiaca = 0; // Valor leído de hartrate previamente guardado

/// Variables para FC
int32_t valor_0=0,valor_1=0;
float  	m_min=0, m_act=0;
float	array_frec[10]={0,0,0,0,0,0,0,0,0,0};
float	delta_t=1,frec_bpm=0;
float	prom_frec=0,sum_frec=0;
float	suma_fc=0, prom_fc=0;
float	frec_bpm_0=0,dif_bpm=0;

int		flag_pls=0,flag_edg=0;
int		cont_m=0,cont_clr=0;
int		cont_array=0,lim_array=6;		//  antes lim_array=10
int		delta_m=10;
int		cont_500ms=0;

int8_t	edo_sonda = 0,edo_sonda_ant=0;
unsigned long tiempo_0=0, tiempo_1=0,cont_pulsos=0;
uint32_t	cta_envio = 0;

/// Variables AFE

boolean drdy_trigger = FALSE; // Bandera indicando que se hay listo un dato para leer, se activa por interrupción
uint8_t dec = 0;                                      // Contador para los comandos del AFE

int8_t n_buffer_count;                            //data length
uint16_t aun_ir_buffer[100];                      //infrared LED sensor data
uint16_t aun_red_buffer[100];                     //red LED sensor data
int32_t an_x[BUFFER_SIZE];
int32_t an_y[BUFFER_SIZE];

const uint8_t uch_spo2_table[184] =
        { 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 99, 99, 99,
          99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
          100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 97, 97, 96, 96, 96,
          96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 90, 90, 89, 89, 89, 88, 88, 87,
          87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74,
          73, 72, 72, 71, 70, 69, 69, 68, 67, 66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56,
          55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33,
          31, 30, 29, 28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 3, 2,
          1 };
afe44xx_output_values afe_datos;                // Datos leídos del AFE


/* USER CODE END Variables */
osThreadId principalHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void afe44xxWrite (afeRegisters address, uint32_t data);
uint32_t afe44xxRead (uint8_t address);
void afe44xxInit (void);
void sort_ascend (int32_t *pn_x, int32_t n_size);
void sort_indices_descend (int32_t *pn_x, int32_t *pn_indx, int32_t n_size);
void remove_close_peaks (int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
void find_peak_above (int32_t *pn_locs,
                      int32_t *n_npks,
                      int32_t *pn_x,
                      int32_t n_size,
                      int32_t n_min_height);
void find_peak (int32_t *pn_locs,
                int32_t *n_npks,
                int32_t *pn_x,
                int32_t n_size,
                int32_t n_min_height,
                int32_t n_min_distance,
                int32_t n_max_num);
void estimate_spo2 (uint16_t *pun_ir_buffer,
                    int32_t n_ir_buffer_length,
                    uint16_t *pun_red_buffer,
                    int32_t *pn_spo2,
                    int8_t *pch_spo2_valid,
                    int32_t *pn_heart_rate,
                    int8_t *pch_hr_valid);

boolean getDataIfAvailable (afe44xx_output_values *sensed_values);
void selChADC (ADC_HandleTypeDef *hadc, uint8_t channel);
uint16_t analogRead (ADC_HandleTypeDef *hadc);
void USART_Rev (UART_HandleTypeDef *huart);


/* USER CODE END FunctionPrototypes */

void tarea1(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize)
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
  /* definition and creation of principal */
  osThreadDef(principal, tarea1, osPriorityNormal, 0, 700);
  principalHandle = osThreadCreate(osThread(principal), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_tarea1 */
/**
 * @brief  Function implementing the principal thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_tarea1 */
void tarea1(void const * argument)
{
  /* USER CODE BEGIN tarea1 */
    HAL_UART_Receive_IT (&huart1, &rxByte, 1);

  //  msgCount = sprintf ((char*) msgBuff, (char*) "R\n");
 //   HAL_UART_Transmit (&huart1, msgBuff, msgCount, 5);

    //AFE Wake-up time from complete power-down : 1000ms TYP
    osDelay (2000); //asegurar que el AFE inicie correctamente
    afe44xxInit ();


    //    usbCount = sprintf ((char*) usbBuff, (char*) "AFE Inicializado\n");

    for (;;)
    {
    	//if(!USART_CR1_PEIE)
    	//{
       		USART_Rev(&huart1);
    	//}
       	if (monitoreoActivo)
       	{
       		getDataIfAvailable (&afe_datos);

       		///////////////////////////////////
       		//SET_BIT(US,1);
       		//HAL_USART_ErrorCallback(&huart1);
       		///////////////////////////////////

       		if (flag_4ms)	//gráfica cada 4ms (250sps)
       		{
       			flag_4ms = false;
       			val_pleth = (afe_datos.red + afe_datos.ir);
       			cont_500ms++;
       			cta_envio++;
       			count_Dato++;
       			if(count_Dato>9)
       			{
       				count_Dato=0;
       			}

       			edo_sonda_ant = edo_sonda;

       			if (val_pleth <= LIM_INF_SONDA)		// valor recibido cuando no se detecta la sonda
       			{
       				edo_sonda = 0;
       				val_pleth_Grafica = 0;

       			}
       			else if (val_pleth >= LIM_SUP_SONDA)	// valor recibido cuando no está colocado el dedo
       			{
       				edo_sonda = 1;
       				val_pleth_Grafica = 0;
       			}
       			else	// sonda y colocación correcta
       			{
       				edo_sonda = 2;
       				val_pleth_Grafica = val_pleth;
       			}

       			if (edo_sonda != edo_sonda_ant)
       			{
       				cta_envio=0;
       			}
       			if (cta_envio == 8)
       			{
       				msgCount = sprintf ((char*) msgBuff, (char*) "O:%i\n",edo_sonda);
       				HAL_UART_Transmit (&huart1, msgBuff, msgCount, 5);
       			}
       			afe_oxigenacion = (int)afe_datos.spo2;
       			if((afe_oxigenacion<60)||(afe_oxigenacion>100))
       			{
       				msgCount = sprintf ((char*) msgBuff, (char*) "B:0,%i\n",count_Dato);
       			}
       			else
       			{
       				msgCount = sprintf ((char*) msgBuff, (char*) "B:%li,%i\n",val_pleth_Grafica / 100,count_Dato);
       			}

       			//msgCount = sprintf ((char*) msgBuff, (char*) "B:%li,%i\n",val_pleth_Grafica / 100,count_Dato);	 //para RAS,

       			HAL_UART_Transmit (&huart1, msgBuff, msgCount, 5);

       			if (edo_sonda == 2) // si se detecta sonda y dedo correctamente
       			{
       				if (cont_m == 1)
       				{
       					valor_0 = val_pleth_Grafica;
       				}
       				else if (cont_m > delta_m)
       				{
       					valor_1 = val_pleth_Grafica;
       					cont_m = 0;
       					m_act = (valor_1 - valor_0) / delta_m;
       				}

       				if(m_act > 0)
       				{
       					if (flag_pls==1)
       					{
       						msgCount = sprintf ((char*) msgBuff, (char*) "S\n");
       						HAL_UART_Transmit (&huart1, msgBuff, msgCount, 5);
       						flag_pls=0;
       						if (tiempo_0>0)
       						{
       							tiempo_1 = millis();
       							delta_t = (float)(tiempo_1-tiempo_0)/1000;
       							frec_bpm = 60/delta_t;
       							cont_pulsos++;
       							flag_init_p = TRUE;
       						}		// fin tiempo_0 > 10
       						tiempo_0 = millis();

       						if (flag_init_p)
       						{
       							if (frec_bpm_0 > 0)
       							{
       								dif_bpm = frec_bpm - frec_bpm_0;
       							}

       							if ((dif_bpm < TOL_FC) && (dif_bpm > -TOL_FC) && (frec_bpm < 220))
       							{
       								array_frec[cont_array] = frec_bpm;
       								cont_array++;
       							}
       							else
       							{
       								array_frec[cont_array] = array_frec[cont_array-1];
       							}

       							frec_bpm_0 = frec_bpm;
       							sum_frec += array_frec[cont_array-1];

       							if (cont_pulsos > (3+lim_array))
       							{
       								//prom_frec = sum_frec /cont_array;
       								afe_frecuenciaCardiaca = (int)prom_fc;
       							}
       							else
       							{
       								if (frec_bpm < 220)
       								{
       									//prom_frec = frec_bpm;
       									afe_frecuenciaCardiaca = (int)frec_bpm;
       								}
       							}

       							if (cont_array > lim_array)
       							{
       								cont_array=1;
       								sum_frec = frec_bpm;
       							}
       							flag_init_p = FALSE;
       						}

       						if (cont_pulsos > 3)
       						{
       							if((afe_oxigenacion<60)||(afe_oxigenacion>100))
       							{
       								afe_frecuenciaCardiaca = 0;
       							}
       							msgCount = sprintf ((char*) msgBuff, "R:%i,%i\n", afe_oxigenacion,
       									afe_frecuenciaCardiaca);
       							HAL_UART_Transmit (&huart1, msgBuff, msgCount, 5);
       	       					//HAL_UART_Receive_IT (&huart1, &rxByte, 1);
       						}
       					}		// fin flag_pls
       					flag_edg=0;
       				}
       				else if(m_act < 0.5*m_min)
       				{
       					if (flag_edg==0)
       					{
       						m_min = m_act;
       						cont_clr = 0;
       						flag_pls = 1;

       						suma_fc = 0;
       						float max_frec=0;
       						float min_frec=400;
       						for (int ij=0;ij<lim_array;ij++)
       						{
       							if (array_frec[ij]>max_frec)
       							{
       								max_frec=array_frec[ij];
       							}
       							if (array_frec[ij]<min_frec)
       							{
       								min_frec=array_frec[ij];
       							}
       							suma_fc+=array_frec[ij];
       						}
       						suma_fc = suma_fc-max_frec-min_frec;
       						prom_fc = suma_fc/(lim_array-2);

       						afe_oxigenacion = (int)afe_datos.spo2;
       						//afe_frecuenciaCardiaca = (int)prom_fc;
       					}
       					flag_edg = 1;
       				}
       				if ((flag_edg == 1) && (m_act < m_min))
       				{
       					m_min=m_act;
       				}

       				cont_m++;

       				if (cont_clr>500)
       				{
       					m_min = m_act;
       					cont_clr = 0;
       				}
       				cont_clr++;

       				//afe_oxigenacion = (int)afe_datos.spo2;
       				//afe_frecuenciaCardiaca = (int)prom_frec;

       			}
       			else		// deconexión de sonda o dedo
       			{
       				afe_oxigenacion = 0;
       				afe_frecuenciaCardiaca = 0;
       				for (int j=0;j>lim_array;j++)
       				{
       					array_frec[j] = 0;
       				}
       				cont_pulsos = 0;
       				cont_array = 0;
       				tiempo_0 = 0;
       				frec_bpm_0 = 0;
       				if (flag_500ms)
       				{
       					//afe_oxigenacion = (int)afe_datos.spo2;
       					if((afe_oxigenacion<70)||(afe_oxigenacion>100))
       					{
       						afe_frecuenciaCardiaca = 0;
       					}
       					else
       					{
       						//msgCount = sprintf ((char*) msgBuff, (char*) "B:%li,%i\n",val_pleth_Grafica / 100,count_Dato);
       						msgCount = sprintf ((char*) msgBuff, "R:%i,%i\n", afe_oxigenacion,
       								afe_frecuenciaCardiaca);
       						HAL_UART_Transmit (&huart1, msgBuff, msgCount, 5);
       					}

       					/*msgCount = sprintf ((char*) msgBuff, "R:%i,%i\n", afe_oxigenacion,
       							afe_frecuenciaCardiaca);
       					HAL_UART_Transmit (&huart1, msgBuff, msgCount, 5);*/
       					flag_500ms = FALSE;
       					//HAL_UART_Receive_IT (&huart1, &rxByte, 1);
       				}
       			}

       			if (cont_500ms >= 150)
       			{
       				cont_500ms = 0;
       				flag_500ms = TRUE;

       			}

       		}	// fin de periodo de 4 ms
       	}
       	else {
       		//coun_ERROR++;
       		//HAL_GPIO_TogglePin(TEST_USART_GPIO_Port, TEST_USART_Pin);
		}

       	if (flag_error)
       	{
       		if (flag_4ms)	//gráfica cada 4ms (250sps)
       		{
       			flag_4ms = false;
       			cont_500ms++;
       			if (cont_500ms >= 5)
       			{
       				cont_500ms = 0;
       				flag_500ms = TRUE;

       			}
       			if(flag_500ms)
       			{
       				//HAL_GPIO_TogglePin(TEST_USART_GPIO_Port, TEST_USART_Pin);
       				msgCount = sprintf ((char*) msgBuff, "U\n");
       				HAL_UART_Transmit (&huart1, msgBuff, msgCount, 5);
       				flag_500ms = FALSE;
       			}

       		}
       	}
       	if (solicitarInfo)
       	{
       		msgCount = sprintf ((char*) msgBuff, "I:MISC,%s\n", versionFirmware);
       		HAL_UART_Transmit (&huart1, msgBuff, msgCount, 5);
       		solicitarInfo = FALSE;
       	}
       	if (flag_Start)
		{
			msgCount = sprintf ((char*) msgBuff, "s\n");
			HAL_UART_Transmit (&huart1, msgBuff, msgCount, 5);
			flag_Start = FALSE;
			//coun_ERROR = 0;
			count_Dato=0;
			//HAL_Delay(3000); /// debug de Plestimografia arrenque
		}
       	if (flag_Ping)
       	{
       		msgCount = sprintf ((char*) msgBuff, "P\n");
       		HAL_UART_Transmit (&huart1, msgBuff, msgCount, 5);
       		flag_Ping = FALSE;
       	}

    }
  /* USER CODE END tarea1 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) // Está en el main.c
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
    if (GPIO_Pin == AFE_DRDY_Pin)
        {
            drdy_trigger = TRUE;
        }
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	//HAL_GPIO_TogglePin(TEST_USART_GPIO_Port, TEST_USART_Pin);
	//HAL_GPIO_WritePin (TEST_USART_GPIO_Port, TEST_USART_Pin, GPIO_PIN_SET);
    if (rxByte == CMD_INFO)
        {
            solicitarInfo = TRUE;
        }
    else if (rxByte == CMD_DETENER)
        {
            monitoreoActivo = FALSE;
        }
    else if (rxByte == CMD_INICIAR)
        {
            monitoreoActivo = TRUE;
    		flag_Start = TRUE;
    		flag_error = FALSE;
    		//HAL_GPIO_WritePin (TEST_USART_GPIO_Port, TEST_USART_Pin, GPIO_PIN_SET);
            __HAL_TIM_SetCounter(&htim2, 0);
        }
    else if (rxByte == CMD_PING)
		{
    		flag_Ping = TRUE;
    		//MX_USART1_UART_Init();
		}
    else
        {
    		//HAL_GPIO_TogglePin(TEST_USART_GPIO_Port, TEST_USART_Pin);
    		//HAL_GPIO_WritePin (TEST_USART_GPIO_Port, TEST_USART_Pin, GPIO_PIN_RESET);
    		monitoreoActivo = FALSE;
    		flag_error = TRUE;
			//monitoreoActivo=0;
        }
    //HAL_UART_Receive_IT (&huart1, &rxByte, 1);
    HAL_UART_Receive_DMA(&huart1, &rxByte, 1);
   // HAL_GPIO_WritePin (TEST_USART_GPIO_Port, TEST_USART_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_TogglePin(TEST_USART_GPIO_Port, TEST_USART_Pin);


}


/*/
void HAL_USART_ErrorCallback (UART_HandleTypeDef *huart)
{
	//if (huart == &huart1){
			__HAL_UART_CLEAR_PEFLAG(&huart1);
			__HAL_UART_CLEAR_OREFLAG(&huart1);
			__HAL_UART_CLEAR_FEFLAG(&huart1);
			__HAL_UART_CLEAR_NEFLAG(&huart1);
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
			//MX_USART1_UART_Init();
			HAL_UART_Init(&huart1);
			__HAL_UNLOCK(huart);

			/// Enable the UART Parity Error Interrupt ///
			__HAL_UART_ENABLE_IT(huart, UART_IT_PE);

			// Enable the UART Error Interrupt: (Frame error, noise error, overrun error) ///
			__HAL_UART_ENABLE_IT(huart, UART_IT_ERR);

			// Enable the UART Data Register not empty Interrupt /
			__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

		//}

}
//*/

void afe44xxWrite (afeRegisters address, uint32_t data)
{
    uint8_t dataArr[3];
    dataArr[0] = (data >> 16) & 0xFF;
    dataArr[1] = (data >> 8) & 0xFF;
    dataArr[2] = data & 0xFF;
    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);            // enable device
    HAL_SPI_Transmit (&hspi1, &address, 1, 10);            // send address to device
    //HAL_SPI_Transmit (&hspi1, &dataArr[0], 1, 10);// write top 8 bits
    //HAL_SPI_Transmit (&hspi1, &dataArr[1], 1, 10);// write middle 8 bits
    //HAL_SPI_Transmit (&hspi1, &dataArr[2], 1, 10);// write bottom 8 bits
    HAL_SPI_Transmit (&hspi1, dataArr, 3, 10);
    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);    // disable device
}

uint32_t afe44xxRead (uint8_t address)
{
    uint8_t datos[3] =
            { 0 };
    uint32_t data = 0;

    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);    // enable device
    HAL_SPI_Transmit (&hspi1, &address, 1, 10);
    HAL_SPI_Receive (&hspi1, datos, 3, 10);
    //         Top8                   | mid 8                   | end 8
    data = ((uint32_t) datos[0] << 16) | (uint32_t) datos[1] << 8 | (uint32_t) datos[2];
    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);    // disable device
    return data; // return with 24 bits of read data
}

void afe44xxInit (void)
{
    //digitalWrite(pwdn, LOW);
    HAL_GPIO_WritePin (AFE_PWDN_GPIO_Port, AFE_PWDN_Pin, GPIO_PIN_RESET);
    osDelay (500);
    HAL_GPIO_WritePin (AFE_PWDN_GPIO_Port, AFE_PWDN_Pin, GPIO_PIN_SET);
    //digitalWrite(pwdn, HIGH);
    osDelay (500);

    afe44xxWrite (CONTROL0, 0x000000);
    afe44xxWrite (CONTROL0, 0x000008);
    afe44xxWrite (TIAGAIN, 0x000000); // CF = 5pF, RF = 500kR
    afe44xxWrite (TIA_AMB_GAIN, 0x000001);
    afe44xxWrite (LEDCNTRL, 0x001414);
    afe44xxWrite (CONTROL2, 0x000000); // LED_RANGE=100mA, LED=50mA
    afe44xxWrite (CONTROL1, 0x010707); // Timers ON, average 3 samples
    afe44xxWrite (PRPCOUNT, 0X001F3F);
    afe44xxWrite (LED2STC, 0X001770);
    afe44xxWrite (LED2ENDC, 0X001F3E);
    afe44xxWrite (LED2LEDSTC, 0X001770);
    afe44xxWrite (LED2LEDENDC, 0X001F3F);
    afe44xxWrite (ALED2STC, 0X000000);
    afe44xxWrite (ALED2ENDC, 0X0007CE);
    afe44xxWrite (LED2CONVST, 0X000002);
    afe44xxWrite (LED2CONVEND, 0X0007CF);
    afe44xxWrite (ALED2CONVST, 0X0007D2);
    afe44xxWrite (ALED2CONVEND, 0X000F9F);
    afe44xxWrite (LED1STC, 0X0007D0);
    afe44xxWrite (LED1ENDC, 0X000F9E);
    afe44xxWrite (LED1LEDSTC, 0X0007D0);
    afe44xxWrite (LED1LEDENDC, 0X000F9F);
    afe44xxWrite (ALED1STC, 0X000FA0);
    afe44xxWrite (ALED1ENDC, 0X00176E);
    afe44xxWrite (LED1CONVST, 0X000FA2);
    afe44xxWrite (LED1CONVEND, 0X00176F);
    afe44xxWrite (ALED1CONVST, 0X001772);
    afe44xxWrite (ALED1CONVEND, 0X001F3F);
    afe44xxWrite (ADCRSTCNT0, 0X000000);
    afe44xxWrite (ADCRSTENDCT0, 0X000000);
    afe44xxWrite (ADCRSTCNT1, 0X0007D0);
    afe44xxWrite (ADCRSTENDCT1, 0X0007D0);
    afe44xxWrite (ADCRSTCNT2, 0X000FA0);
    afe44xxWrite (ADCRSTENDCT2, 0X000FA0);
    afe44xxWrite (ADCRSTCNT3, 0X001770);
    afe44xxWrite (ADCRSTENDCT3, 0X001770);
    osDelay (1000);
}

void sort_ascend (int32_t *pn_x, int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
        {
            n_temp = pn_x[i];
            for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
                pn_x[j] = pn_x[j - 1];
            pn_x[j] = n_temp;
        }
}

void sort_indices_descend (int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
        {
            n_temp = pn_indx[i];
            for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
                pn_indx[j] = pn_indx[j - 1];
            pn_indx[j] = n_temp;
        }
}

void remove_close_peaks (int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
{

    int32_t i, j, n_old_npks, n_dist;

    /* Order peaks from large to small */
    sort_indices_descend (pn_x, pn_locs, *pn_npks);

    for (i = -1; i < *pn_npks; i++)
        {
            n_old_npks = *pn_npks;
            *pn_npks = i + 1;
            for (j = i + 1; j < n_old_npks; j++)
                {
                    n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]); // lag-zero peak of autocorr is at index -1
                    if (n_dist > n_min_distance || n_dist < -n_min_distance)
                        pn_locs[ (*pn_npks)++] = pn_locs[j];
                }
        }

    // Resort indices int32_to ascending order
    sort_ascend (pn_locs, *pn_npks);
}

void find_peak_above (int32_t *pn_locs,
                      int32_t *n_npks,
                      int32_t *pn_x,
                      int32_t n_size,
                      int32_t n_min_height)
{
    int32_t i = 1, n_width;
    *n_npks = 0;

    while (i < n_size - 1)
        {
            if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1])
                {   // find left edge of potential peaks
                    n_width = 1;
                    while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width]) // find flat peaks
                        n_width++;
                    if (pn_x[i] > pn_x[i + n_width] && (*n_npks) < 15)
                        {   // find right edge of peaks
                            pn_locs[ (*n_npks)++] = i;
                            // for flat peaks, peak location is left edge
                            i += n_width + 1;
                        }
                    else
                        i += n_width;
                }
            else
                i++;
            //  Serial.println("beat");
        }
}

void find_peak (int32_t *pn_locs,
                int32_t *n_npks,
                int32_t *pn_x,
                int32_t n_size,
                int32_t n_min_height,
                int32_t n_min_distance,
                int32_t n_max_num)
{
    find_peak_above (pn_locs, n_npks, pn_x, n_size, n_min_height);
    remove_close_peaks (pn_locs, n_npks, pn_x, n_min_distance);
    *n_npks = min(*n_npks, n_max_num);
}

void estimate_spo2 (uint16_t *pun_ir_buffer,
                    int32_t n_ir_buffer_length,
                    uint16_t *pun_red_buffer,
                    int32_t *pn_spo2,
                    int8_t *pch_spo2_valid,
                    int32_t *pn_heart_rate,
                    int8_t *pch_hr_valid)
{
    //uint32_t un_ir_mean, un_only_once;
    uint32_t un_ir_mean;
    int32_t k, n_i_ratio_count;
    //int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t i, n_exact_ir_valley_locs_count, n_middle_idx;
    //int32_t n_th1, n_npks, n_c_min;
    int32_t n_th1, n_npks;
    int32_t an_ir_valley_locs[15];
    int32_t n_peak_interval_sum;

    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc;
    int32_t n_y_dc_max, n_x_dc_max;
    int32_t n_y_dc_max_idx, n_x_dc_max_idx;
    int32_t an_ratio[5], n_ratio_average;
    int32_t n_nume, n_denom;

    // calculates DC mean and subtract DC from ir
    un_ir_mean = 0;
    for (k = 0; k < n_ir_buffer_length; k++)
        un_ir_mean += pun_ir_buffer[k];
    un_ir_mean = un_ir_mean / n_ir_buffer_length;

    // remove DC and invert signal so that we can use peak detector as valley detector
    for (k = 0; k < n_ir_buffer_length; k++)
        an_x[k] = -1 * (pun_ir_buffer[k] - un_ir_mean);

    // 4 pt Moving Average
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE; k++)
        {
            an_x[k] = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / (int) 4;
        }
    // calculate threshold
    n_th1 = 0;
    for (k = 0; k < BUFFER_SIZE; k++)
        {
            n_th1 += an_x[k];
        }
    n_th1 = n_th1 / (BUFFER_SIZE);
    if (n_th1 < 30) n_th1 = 30; // min allowed
    if (n_th1 > 60) n_th1 = 60; // max allowed

    for (k = 0; k < 15; k++)
        an_ir_valley_locs[k] = 0;
    // since we flipped signal, we use peak detector as valley detector
    find_peak (an_ir_valley_locs, &n_npks, an_x, BUFFER_SIZE, n_th1, 4, 15); //peak_height, peak_distance, max_num_peaks
    n_peak_interval_sum = 0;
    if (n_npks >= 2)
        {
            for (k = 1; k < n_npks; k++)
                n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k - 1]);
            n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
            *pn_heart_rate = (int32_t) ( (FS * 60) / n_peak_interval_sum);
            *pch_hr_valid = 1;
        }
    else
        {
            *pn_heart_rate = -999; // unable to calculate because # of peaks are too small
            *pch_hr_valid = 0;
        }

    //  load raw value again for SPO2 calculation : RED(=y) and IR(=X)
    for (k = 0; k < n_ir_buffer_length; k++)
        {
            an_x[k] = pun_ir_buffer[k];
            an_y[k] = pun_red_buffer[k];
        }

    // find precise min near an_ir_valley_locs
    n_exact_ir_valley_locs_count = n_npks;

    //using exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration an_ratio
    //finding AC/DC maximum of raw

    n_ratio_average = 0;
    n_i_ratio_count = 0;
    for (k = 0; k < 5; k++)
        an_ratio[k] = 0;
    for (k = 0; k < n_exact_ir_valley_locs_count; k++)
        {
            if (an_ir_valley_locs[k] > BUFFER_SIZE)
                {
                    *pn_spo2 = -999; // do not use SPO2 since valley loc is out of range
                    *pch_spo2_valid = 0;
                    return;
                }
        }
    // find max between two valley locations
    // and use an_ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2
    for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++)
        {
            n_y_dc_max = -16777216;
            n_x_dc_max = -16777216;
            if (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k] > 3)
                {
                    for (i = an_ir_valley_locs[k]; i < an_ir_valley_locs[k + 1]; i++)
                        {
                            if (an_x[i] > n_x_dc_max)
                                {
                                    n_x_dc_max = an_x[i];
                                    n_x_dc_max_idx = i;
                                }
                            if (an_y[i] > n_y_dc_max)
                                {
                                    n_y_dc_max = an_y[i];
                                    n_y_dc_max_idx = i;
                                }
                        }
                    n_y_ac = (an_y[an_ir_valley_locs[k + 1]] - an_y[an_ir_valley_locs[k]])
                                    * (n_y_dc_max_idx - an_ir_valley_locs[k]); //red
                    n_y_ac = an_y[an_ir_valley_locs[k]]
                                  + n_y_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]);
                    n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac; // subracting linear DC compoenents from raw
                    n_x_ac = (an_x[an_ir_valley_locs[k + 1]] - an_x[an_ir_valley_locs[k]])
                                    * (n_x_dc_max_idx - an_ir_valley_locs[k]); // ir
                    n_x_ac = an_x[an_ir_valley_locs[k]]
                                  + n_x_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]);
                    n_x_ac = an_x[n_y_dc_max_idx] - n_x_ac; // subracting linear DC compoenents from raw
                    n_nume = (n_y_ac * n_x_dc_max) >> 7; //prepare X100 to preserve floating value
                    n_denom = (n_x_ac * n_y_dc_max) >> 7;
                    if (n_denom > 0 && n_i_ratio_count < 5 && n_nume != 0)
                        {
                            an_ratio[n_i_ratio_count] = (n_nume * 100) / n_denom; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
                            n_i_ratio_count++;
                        }
                }
        }
    // choose median value since PPG signal may varies from beat to beat
    sort_ascend (an_ratio, n_i_ratio_count);
    n_middle_idx = n_i_ratio_count / 2;

    if (n_middle_idx > 1)
        n_ratio_average = (an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2; // use median
    else
        n_ratio_average = an_ratio[n_middle_idx];

    if (n_ratio_average > 2 && n_ratio_average < 184)
        {
            n_spo2_calc = uch_spo2_table[n_ratio_average];
            *pn_spo2 = n_spo2_calc;
            *pch_spo2_valid = 1; //  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
        }
    else
        {
            *pn_spo2 = -999; // do not use SPO2 since signal an_ratio is out of range
            *pch_spo2_valid = 0;
        }
}

boolean getDataIfAvailable (afe44xx_output_values *sensed_values)
{
    uint32_t IRtemp, REDtemp;
    boolean afe44xx_data_ready = FALSE;

    int32_t n_spo2;  //SPO2 value
    int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
    int32_t n_heart_rate; //heart rate value
    int8_t ch_hr_valid;  //indicator to show if the heart rate calculation is valid

    if (drdy_trigger)
        {
            //detachInterrupt(intrrpt_num); DESHABILITAR LA INTERUPCIÓN. En STM están en globadas las 9 a 5, yo estoy ocupando la 8
            NVIC_DisableIRQ (EXTI9_5_IRQn);
            afe44xxWrite (CONTROL0, 0x000001);
            IRtemp = afe44xxRead (LED1VAL);
            afe44xxWrite (CONTROL0, 0x000001);
            REDtemp = afe44xxRead (LED2VAL);
            afe44xx_data_ready = TRUE;
        }
    if (afe44xx_data_ready == TRUE)
        {
            IRtemp = (uint32_t) (IRtemp << 10);
            sensed_values->ir = (int32_t) (IRtemp);
            sensed_values->ir = (int32_t) ( (sensed_values->ir) >> 10);
            REDtemp = (uint32_t) (REDtemp << 10);
            sensed_values->red = (int32_t) (REDtemp);
            sensed_values->red = (int32_t) ( (sensed_values->red) >> 10);

            if (dec == 20)
                {
                    aun_ir_buffer[n_buffer_count] = (uint16_t) ( (sensed_values->ir) >> 4);
                    aun_red_buffer[n_buffer_count] = (uint16_t) ( (sensed_values->red) >> 4);
                    n_buffer_count++;
                    dec = 0;
                }

            dec++;

            if (n_buffer_count > 99)
                {
                    estimate_spo2 (aun_ir_buffer, 100, aun_red_buffer, &n_spo2, &ch_spo2_valid,
                                   &n_heart_rate, &ch_hr_valid);
                    sensed_values->spo2 = n_spo2;
                    sensed_values->heart_rate = n_heart_rate;
                }

            sensed_values->calculated_value = TRUE;
            //afe44xx_data_ready = FALSE;
            drdy_trigger = FALSE;
            //attachInterrupt(intrrpt_num, afe44xx_drdy_event, RISING );
            NVIC_EnableIRQ (EXTI9_5_IRQn);
            if (n_buffer_count > 99)
                {
                    n_buffer_count = 0;
                    return TRUE;
                }
            return FALSE;  //evitar una salida sin return

        }
    else
        {

            return FALSE;
        }

}

void selChADC (ADC_HandleTypeDef *hadc, uint8_t channel)
{
    ADC_ChannelConfTypeDef sConfig =
            { 0 };

    if (HAL_ADC_DeInit (&hadc1) != HAL_OK)
        {
            Error_Handler ();
        }

    if (HAL_ADC_Init (&hadc1) != HAL_OK)
        {
            Error_Handler ();
        }

    switch (channel)
    {
        case 0:
            sConfig.Channel = ADC_CHANNEL_0;
            sConfig.Rank = ADC_REGULAR_RANK_1;
            break;
        case 1:
            sConfig.Channel = ADC_CHANNEL_1;
            sConfig.Rank = ADC_REGULAR_RANK_2;
            break;
        case 2:
            sConfig.Channel = ADC_CHANNEL_2;
            sConfig.Rank = ADC_REGULAR_RANK_3;
            break;
        default:
            break;
    }

    sConfig.SamplingTime =  ADC_SAMPLETIME_1CYCLE_5;		// error de lectura por mala definición de velocidad de muestreo, cambiar por: ADC_SAMPLETIME_28CYCLES_5
    if (HAL_ADC_ConfigChannel (hadc, &sConfig) != HAL_OK)
        {
            Error_Handler ();
        }
}

uint16_t analogRead (ADC_HandleTypeDef *hadc)
{

    HAL_ADC_Start (hadc);
    HAL_ADC_PollForConversion (hadc, 5);
    uint16_t valorADCLeido = HAL_ADC_GetValue (hadc);
    HAL_ADC_Stop (hadc);
    return valorADCLeido;
}


void USART_Rev (UART_HandleTypeDef *huart)
{
	//HAL_GPIO_TogglePin(TEST_USART_GPIO_Port, TEST_USART_Pin);
	uint32_t USART1_CR1     = READ_REG(huart->Instance->CR1);
	uint32_t error_flags_U1 = 0x00U;
	error_flags_U1 = (USART1_CR1 &(uint32_t)USART_CR1_PEIE_Msk);
	if(!error_flags_U1)
	{
		/*
		HAL_UART_Init(&huart1);
		__HAL_UART_CLEAR_PEFLAG(&huart1);
		__HAL_UART_CLEAR_OREFLAG(&huart1);
		__HAL_UART_CLEAR_FEFLAG(&huart1);
		__HAL_UART_CLEAR_NEFLAG(&huart1);
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		//MX_USART1_UART_Init();
		HAL_UART_Init(&huart1);
		__HAL_UNLOCK(huart);

		/// Enable the UART Parity Error Interrupt ///
		__HAL_UART_ENABLE_IT(huart, UART_IT_PE);

		// Enable the UART Error Interrupt: (Frame error, noise error, overrun error) ///
		__HAL_UART_ENABLE_IT(huart, UART_IT_ERR);

		// Enable the UART Data Register not empty Interrupt /
		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
		*/
	    SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) |
	                   (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
	                   SCB_AIRCR_SYSRESETREQ_Msk);

	}
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
	{
		monitoreoActivo = TRUE;
		__HAL_RCC_CLEAR_RESET_FLAGS();
	}
	/*
	__HAL_UART_CLEAR_PEFLAG(&huart1);
	__HAL_UART_CLEAR_OREFLAG(&huart1);
	__HAL_UART_CLEAR_FEFLAG(&huart1);
	__HAL_UART_CLEAR_NEFLAG(&huart1);
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	//MX_USART1_UART_Init();
	HAL_UART_Init(&huart1);
	__HAL_UNLOCK(huart);

	/// Enable the UART Parity Error Interrupt ///
	__HAL_UART_ENABLE_IT(huart, UART_IT_PE);

	// Enable the UART Error Interrupt: (Frame error, noise error, overrun error) ///
	__HAL_UART_ENABLE_IT(huart, UART_IT_ERR);

	// Enable the UART Data Register not empty Interrupt /
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
	*/
}

/* USER CODE END Application */

