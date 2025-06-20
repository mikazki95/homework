/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * Autor:       Ing. Alejandra Sakura Bautista Ambrocio
 * Descripción: Código de aplicación FreeRTos para el funcionamiento del
 *              Monitor de Signos Vitales.
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
 * Circuito de marcapasos: Al utilizar los buffers PACE, no requiere ninguna tasa de muestreo en particular (es independiente de la configuración de muestreo).
 *
 * Comandos disponibles                         |            Formato               | Ejemplo
 * S: Iniciar                                   | S\n                              | S\n
 * E: Detener                                   | E\n                              | E\n
 * I: Informar versión de firmware              | I\n                              | I\n
 * G: Seleccionar derivaciones a graficar.      | G:'Derivada1','Derivada2'\n      | G:2,03\n
 * F: Seleccionar filtro para la derivada.      | F:[1|2],[N|D|M|A]                | F:1,A\n
 * 0: Alarma - Prioridad alta, color rojo.      | 0\n                              | 0\n
 * 1: Alarma - Prioridad media, color amarillo. | 1\n                              | 1\n
 * 2: Alarma - Prioridad baja, color blanco.    | 2\n                              | 2\n
 * 3: Alarma - Prioridad media, color azul.     | 3\n                              | 3\n
 * 4: Color sólido - Rojo.                      | 4\n                              | 4\n
 * 5: Color sólido - Amarillo.                  | 5\n                              | 5\n
 * 6: Color sólido - Verde.                     | 6\n                              | 6\n
 * 7: Color sólido - Negro (Apagar Leds).       | 7\n                              | 7\n
 * 8: Color sólido - Blanco.                    | 8\n                              | 8\n
 * B: Desactivar alarma de apnea.               | B\n                              | B\n
 * C: Activar alarma de apnea                   | C:'Tiempo'\n                     | C:10\n
 * P: Ping-Pong. Comunicción constante con PC   | P\n                              | P\n    ->  STM responde P\n
 *
 *
 *
 *
 * Lista de cambios:
 *
 * 1.0.0 - Primer versión estable.
 * 1.1.0 - Se agrega lectura del cargador de batería LTC4162.
 *         Se toma como base la librería que hice en Arduino.
 * 1.1.1 - Se agrega Tarea para el envío de mensajes USB.
 *         Debido a que el firmware de la raspberry está casado con la versión
 *         de firmware se mantiene 1.1.0 para que la raspberry trabaje. Si se
 *         cambia el número, la aplicación de la raspberry no hace nada.
 * 1.2.0 - Se cambia el algoritmo de respiración. Ultra-suavizado y uso de derivada.
 *         Se agrega un umbral (UMBRAL_RESPIRACION) a la derivada para evitar respiraciones fantasma.
 *         Se agrega el envío del estado del LTC4162.
 *         Se separan los mensajes en buffers "temporales" independientes para un correcto envío por USB.
 *         En las pruebas de observa que la detección de electrodos no es 100% fiable, por lo que se buscará mejorar la detección
 * 1.3.0 - Se cambia la configuración del ADS para 500sps en lectura continua,
 * 1.3.1 - ADS en 500sps HR, salida de datos 250 sps
 * 1.3.2 - Se elimina D:G1,G2,RESP\n
 *           * Cadena nueva, gráfica 1:   "A:G1\n"
 *           * Cadena nueva, gráfica 2:   "B:G2\n"
 *           * Cadena nueva, respiración: "C:RP\n"
 * 1.3.3 - Se elimina el monitoreo de la batería y se elimina el código referente a la batería
 *         tarea5 aún no eliminada, pero ya se vació.
 * 1.3.4 - Comienzo ordenación de código para agregar los filtros.
 *         Algunas variables y definiciones renombradas.
 *         Eliminación de la parte de respiración. Posteriormente se agregará el programa
 *         que está realizando Gabriela Morales.
 *         * Se agregan 4 instancias de filtro, 2 filtros "Ambulatorios" y 2 de "Diagnóstico para cada
 *           gráfica de ECG.
 *         * Comando nuevo. Selección del filtro del ECG:
 *           * Filtro de "diagnóstico" a la gráfica X: "F:Gx,D\n"
 *           * Filtro "Ambulatorio" a la gráfica X:    "F:Gx,A\n"
 *           * Sin filtro en la gŕafica X:             "F:Gx,0\n" //Para debug
 * 1.3.5 - Desde la versión 1.3.0 se deshabilitó la tarea de revisión de electrodos por el modo de trabajo del ADS.
 *         En esta versión se habilita
 * 1.3.6 - Se añaden valores dummy para la graficación de respiración. Para prueba de tener todas las cadenas necesarias
 *         para el software de la raspberry.
 *         Se cambia "Ambulatorio" por Monitor en el nombre de los filtros y lo relacionado.
 *         Cadena de Frecuencia respiratoria: "R:FR\n"
 *         Se corrige el color de los leds simples rgb, ya que el diagrama no coincide con loc componentes reales.
 *         A modo de debug, el led RGB1 se ilumina de acuerdo al último tipo de comando recibido como se indica:
 *           * Rojo     - Detener (o tarjeta recién encendida)
 *           * Verde    - Iniciar
 *           * Azul     - Solicitud de firmware
 *           * Cyan     - Selección de derivadas a graficar.
 *           * Magenta  - Selección del filtro a utilizar en una derivada.
 *           * Amarillo - Comando de alerta (color de tira de led y parpadeo). 0 a 3
 *           * Blanco   - Encender la tira led en color sólido (o apagar la tira). 4 a 8
 *         El led RGB2 se utiliza para mostrar el estado del muestreo:
 *           * Rojo  - Muestreo detenido.
 *           * Azul  - En espera de muestra.
 *           * Verde - Comenzando lectura del ADS.
 *         Al ser tan rápido el proceso de muestreo, cuando se activa la lectura del ADS
 *         pareciera que el led tiene un color cyan.
 *         Se acomoda el código del muestreo dentro de un if, para poder cumplir el punto anterior
 * 1.3.7 - Se comienza la modificación y prueba de los filtros ya que en las pruebas con simulador, los filtros parecían no
 *         estar funcionando adecuadamente.
 *         Se cambian los nombres a los componentes de los filtros: Instancias, estados, coeficientes.
 * 1.3.8 - Agrego filtro pasabajos de 60Hz a la señal NULL a petición de Víctor para ver como se comporta el equipo en pruebas.
 *         Probablemente se deba hacer un filtro rechaza-banda de 60Hz pero desconocemos el ancho de banda exacto.
 * 1.4.0 - Comienzo a añadir el código de Gaby Morales:
 *         Mensaje R:   Frecuencia Respiratoria
 *         Mensaje F:   Frecuencia Cardiaca
 *         Añadí buffers de FR y FC
 *         Coloqué los mensajes en la tarea 6
 *
 * 1.5.0 - Separación de FR y FC en archivos individuales
 * 1.5.1 - Comienzo Separación de la librería del ADS1298
 *         Separé las funciones de led simple rgb
 * 1.5.2 - Neopixel separado
 * 1.5.3 - Comienzo a separar tareas
 *
 * 1.6.0 - Tareas Separadas. Nueva estructura de tareas. Cada tarea tiene su
 *         propio archivo cabecera y fuente.
 *         | Tarea    | Nombre             | Archivos (.c/.h)         |
 *         | tarea0() | LeerADS            | tarea_LeerdADS           |
 *         | tarea1() | GraficarECG        | tarea_GraficarECG        |
 *         | tarea2() | RevisarComandos    | tarea_RevisarComandos    |
 *         | tarea3() | RevisarElectrodos  | tarea_RevisarElectrodos  |
 *         | tarea4() | ControlNeopixel    | tarea_ControlNeopixel    |
 *         | tarea5() | Normalización      | tarea_Normalización      |
 *         | tarea6() | ControlUSB         | tarea_ControlUSB         |
 *         | tarea7() | Respiracion        | tarea_Respiracion        |
 *         | tarea8() | FrecuenciaCardiaca | tarea_FrecuenciaCardiaca |
 *
 *         Se Realizan anotaciones actualizadas al código
 * 2 - La cantidad de tareas RTOS se define en main.h: #define TASK_COUNT "9"
 *     A partir de ahora se define 1 sólo número para la versión del archivo.
 *     Se cambia el formato de despliegue de información de firmware.
 *     Ahora es:
 *     I:STM,'VersiónPrincipal','#Tareas','VersionT0','VersionT1','VersionT2','VersionT3','VersionT4','VersionT5','VersionT6','VersionT7','VersionT8'\n
 *     Pej. I:STM,2,9,1,1,1,1,1,1,1,1,1\n
 *     Se cambia La letra de selección de filtro máximo de 'X' a 'A'
 *     Se añaden Queues para las gráficas de ECG 1 y 2, para asegurar que no hay corrupción en el dato a graficar
 *     Se sincronizan las tareas de procesamiento mediante la tarea de LeerADS.
 *     Se ajustan prioridades de tareas.
 *     En la lectura del ADS se agrega detección de error. Al parecer algunas
 *     veces el ADS enviaba los datos corridos 1 byte y eso ocacionaba el pico de ruido,
 *      Compruebo el encabezado de los 27 bytes y que los canales no utilizados contengan valor CERO
 *      Si no tuvieran el valor cero, indicaría que el dato se corrió o se leyó mal.
 *      Por ahora no tengo modo de corregir el dato, simplemente reenvío el último dato válido.
 *     En caso de dato erróneo, envío el último dato válido.
 *     Realizando pruebas, se generan errores cada 22.2 segundos. Agregué un Timer para reiniciar el ADS cada cierto tiempo para evitar que se genere el error
 *     y también permite la recuperación de la señal ECG, con 5 segundos va bien, pero en caso de que se pause la señal, siento que es mucho tiempo.
 *     voy a probar con 2 segundos o 1 segundo. Ocupar ADS_RESET_TIMER
 *
 *     Se trababa por un desbordamiento al leer array en Normalización. Ya se corrigió.
 *
 *     Tona agrega código sin documentar en varias partes del programa. Se intentará encontrar sus cambios
 *     para documentarlos.
 *
 *     ads1298.h:
 *         Se eliminó ads_Init(uint32_t initDelay) se mantiene void ads_Init(void)
 *     ads1298.c:
 *         ads_Init(void) -> Al configurar envío de datos se habilita mantiene osDelay(500)
 *     tarea_ControlUSB.c:
 *         Se eliminó 'extern int32_t ads_II_i; //debug'
 *         Se cambió la cadena de envío de gráfica 1
 *           Anterior:
 *             usbCountTotal += sprintf ((char*)pMSG + usbCountTotal,"dbg,%04li,%08lX,%08lX\nA:%.3f\n",error_cnt,ads_I_i,ads_II_i,g_queue1); //Raw I, deriv_sin filtro, gráfica
 *           Actual:
 *             usbCountTotal += sprintf ((char*)pMSG + usbCountTotal,"e,%li,a,%08lX\nA:%.3f\n",error_cnt,ads_I_i,g_queue1); //Raw I, deriv_sin filtro, gráfica
 *     tarea_LeerADS.c:
 *         en void tarea_LeerADS_Setup(void) se elimina 'osDelay(500);'
 *     tarea_Normalizacion.c:
 *         Seguirá cambiando debido a que aún trabajan con esta tarea.
 *     freertos.c:
 *         Actualización en control de cambios
 *         Se eliminan definiciones
 *             #define ADS_RESET_SECS   2
 *             #define ADS_RESET_TIMER  (ADS_RESET_SECS * 1000)
 *         Se inicia el timer de software con un valor fijo de 5 segundos
 *             'osTimerStart(TimerMinuteHandle, 5000);'
 *         Todo lo anterior se pudo evitar poniendo ADS_RESET_SECS = 5
 *
 *         Nueva variable 'bool flag_marcapasos_norm = false;'
 *         Se habilita 'tarea_Normalizacion_Loop();' en la tarea5 (normalización)
 *
 *        En el callback del timer ya no se reinicia el ads con 'ads_Init()' y 'ads_Start()'
 *        Ahora sólo 'ads_Stop()' y 'ads_Start()'
 *
 *        HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin):
 *            Se establece la bandera 'flag_marcapasos_norm = true'
 *            //////////////////////////////////////tona
 *            se modifico el calculo de la frecuencia cardiaca, se modifico la normalizacion, se modifico
 *            la revision de electrodos tambien se modifico leer ads
 *
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
#include <arm_math.h>
#include "usbd_cdc_if.h"

#include "ads1298.h"         // Librerías de dispositivos
#include "led_rgb_simple.h"
#include "led_neopixel.h"

#include "tarea_LeerADS.h"            // Tarea 0
#include "tarea_GraficarECG.h"        // Tarea 1
#include "tarea_RevisarComandos.h"    // Tarea 2
#include "tarea_RevisarElectrodos.h"  // Tarea 3
#include "tarea_ControlNeopixel.h"    // Tarea 4
#include "tarea_Normalizacion.h"      // Tarea 5
#include "tarea_ControlUSB.h"         // Tarea 6
#include "tarea_Respiracion.h"        // Tarea 7
#include "tarea_FrecuenciaCardiaca.h" // Tarea 8
#include "tarea_CalcularOffset.h"     // Tarea 9

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION "2"
#define TIM_ONE_RESET_SECS   1
#define TIM_ONE_RESET_VALUE  (TIM_ONE_RESET_SECS * 1000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//Dispositivos/variables externas
//extern I2C_HandleTypeDef hi2c1;
//extern I2C_HandleTypeDef hi2c3;
//extern SPI_HandleTypeDef hspi1;
//extern SPI_HandleTypeDef hspi2;

//                Variable                             |        Origen         |  Destino 1         | Destino 2           |
//                                                     |                       |                    |                     |
// Programa principal activo
bool flag_MSV_graficar = false;                        // Revisar Comandos     | EXTI               | Revisar Electrodos  |
// Selección de derivaciones a graficar
ecg_Graficas_s derivacionesAGraficar;                  // Revisar Comandos     | Graficar ECG       |                     |
// Activación de la alarma de apnea
bool flag_activa_apnea = false;                        // Revisar Comandos     | Respiración        |                     |
// Valor para la comprobación de apnea
int valor_apnea = 10;                                  // Revisar Comandos     | Respiración        |                     |
// Identificación del tipo de paceinte
uint8_t paciente_tipo = 0;		//0 - adulto, 1 - pediatrico, 3 - Neonato
// Tiempo para las alarmas intermitentes
neoPixel_RutinaPrioridad_t prioridadNeopixel = P_ALTA; // Revisar Comandos     | Control Neopixel   |                     |
// Modo de iluminación de la tira
neoPixel_RutinaModo_t modoLeds = MODO_NULO;            // Revisar Comandos     | Control Neopixel   |                     |
// Color de la tira
neoPixel_Colors_t colorLeds = C_NEGRO;                 // Revisar Comandos     | Control Neopixel   |                     |
// En modo parpadeo: Encender o apagar Leds
bool blink_ON = false;                                 // Revisar Comandos     | Control Neopixel   |                     |
// Cálculo del offset de la derivada I de ECG (RAW)
int64_t offsetRaw = 0;
// Bandera de latido para calcular offset RAW de Lead I
bool flag_Latido_Offset = false;
// Derivación I RAW
int32_t ads_I_i;                                       // Leer ADS             | Normalización      |                     |
// Derivación II RAW
int32_t ads_II_i;                                      // Leer ADS             | Normalización      |                     |
// Derivación I
float32_t ads_I_f;                                     // Leer ADS             | Graficar ECG       | Frecuencia Cardiaca |
// Derivación II
float32_t ads_II_f;                                    // Leer ADS             | Graficar ECG       |                     |
// Derivación III
float32_t ads_III_f;                                   // Leer ADS             | Graficar ECG       |                     |
// Valor de respiración
//  float32_t ads_Resp;     // YA NO SE USA 02/08/2023
int32_t ads_Resp_int;       // Agregado para dato de respiración
float original_res;// Leer ADS             | Respiración        |                     |
// Mandar información del firmware
bool flag_show_info = false;                           // Revisar Comandos     | Envío USB          |                     |
// Variable con la versión del firmware
char versionMain[] = FIRMWARE_VERSION;                 // -                    | Envío USB          |                     |
// Pulso de marcapasos detectado
bool flag_marcapasos = false;                          // EXTI (INTERUPT1_Pin) | Envío USB          |                     |
bool flag_marcapasos_norm = false;
int cont_flag_mk=0;
// Gráfica ECG 1
//osMessageQId ecgGraph1Handle;                        // Graficar ECG         | Envío USB          |                     |
// Gráfica ECG 2
//osMessageQId ecgGraph2Handle;                        // Graficar ECG         | Envío USB          |
// Información de frecuencia Cardiaca lista
bool flag_FrecC_BEEP_RDY = false;                      // Frecuencia Cardiaca  | Envío USB          |
// frecuencia cardiaca
int16_t frecuenciaCardiaca;                            // Frecuencia Cardiaca  | Envío USB          |
// Información de respiración lista
bool flag_Resp_RDY = false;                            // Respiración          | Envío USB          |
// Gráfica 3
float32_t graph3;                                      // Respiración          | Envío USB          |
int32_t  aux_grap_resp;
// Frecuencia respiratoria
float Respiration_Rate;                                // Respiración          | Envío USB          |
int variable_auxiliar;
float variable_auxiliar2;
// Alarma de apnea
bool  msgApnea;                                        // Respiración          | Envío USB          |
// Información de electrodos lista
bool flag_RevElec_RDY = false;                         // Revisar electrodos   | Envío USB          |
// Estado de RA
bool lead_1_RA;                                        // Revisar electrodos   | Envío USB          |
// Estado de LA
bool lead_2_LA;                                        // Revisar electrodos   | Envío USB          |
// Estado de RL
bool lead_3_RL;                                        // Revisar electrodos   | Envío USB          |
// Estado de LL
bool lead_4_LL;                                        // Revisar electrodos   | Envío USB          |
// Estado de V1
bool lead_5_V1;                                        // Revisar electrodos   | Envío USB          |
// Ping-Pong
bool flag_Ping = false;                                // Revisar Comandos     | Envío USB          |
int32_t aux_der=0;                            // Frecuencia Cardiaca  | Envío USB
int32_t aux_deb_norm;                                    // Leer ADS             | Graficar ECG |

int32_t error_cnt = 0; //debug_error
int      debug_ads=1010;

/* USER CODE END Variables */
osThreadId GraficarECGHandle;
osThreadId RevisarComandosHandle;
osThreadId RevisarElectrodosHandle;
osThreadId ControlNeopixelHandle;
osThreadId NormalizacionHandle;
osThreadId ControlUSBHandle;
osThreadId LeerADSHandle;
osThreadId RespiracionHandle;
osThreadId FrecuenciaCardiacaHandle;
osThreadId CalcularOffsetHandle;
osMessageQId ecgGraph1Handle;
osMessageQId ecgGraph2Handle;
osTimerId timerOneHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void tarea1(void const * argument);
void tarea2(void const * argument);
void tarea3(void const * argument);
void tarea4(void const * argument);
void tarea5(void const * argument);
void tarea6(void const * argument);
void tarea0(void const * argument);
void tarea7(void const * argument);
void tarea8(void const * argument);
void tarea9(void const * argument);
void timerOneCallback(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* Create the timer(s) */
  /* definition and creation of timerOne */
  osTimerDef(timerOne, timerOneCallback);
  timerOneHandle = osTimerCreate(osTimer(timerOne), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  osTimerStart(timerOneHandle, TIM_ONE_RESET_VALUE);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ecgGraph1 */
  osMessageQDef(ecgGraph1, 3, float32_t);
  ecgGraph1Handle = osMessageCreate(osMessageQ(ecgGraph1), NULL);

  /* definition and creation of ecgGraph2 */
  osMessageQDef(ecgGraph2, 3, float32_t);
  ecgGraph2Handle = osMessageCreate(osMessageQ(ecgGraph2), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of GraficarECG */
  osThreadDef(GraficarECG, tarea1, osPriorityAboveNormal, 0, 256);
  GraficarECGHandle = osThreadCreate(osThread(GraficarECG), NULL);

  /* definition and creation of RevisarComandos */
  osThreadDef(RevisarComandos, tarea2, osPriorityAboveNormal, 0, 128);
  RevisarComandosHandle = osThreadCreate(osThread(RevisarComandos), NULL);

  /* definition and creation of RevisarElectrodos */
  osThreadDef(RevisarElectrodos, tarea3, osPriorityNormal, 0, 128);
  RevisarElectrodosHandle = osThreadCreate(osThread(RevisarElectrodos), NULL);

  /* definition and creation of ControlNeopixel */
  osThreadDef(ControlNeopixel, tarea4, osPriorityNormal, 0, 128);
  ControlNeopixelHandle = osThreadCreate(osThread(ControlNeopixel), NULL);

  /* definition and creation of Normalizacion */
  osThreadDef(Normalizacion, tarea5, osPriorityNormal, 0, 256);
  NormalizacionHandle = osThreadCreate(osThread(Normalizacion), NULL);

  /* definition and creation of ControlUSB */
  osThreadDef(ControlUSB, tarea6, osPriorityAboveNormal, 0, 256);
  ControlUSBHandle = osThreadCreate(osThread(ControlUSB), NULL);

  /* definition and creation of LeerADS */
  osThreadDef(LeerADS, tarea0, osPriorityHigh, 0, 256);
  LeerADSHandle = osThreadCreate(osThread(LeerADS), NULL);

  /* definition and creation of Respiracion */
  osThreadDef(Respiracion, tarea7, osPriorityNormal, 0, 256);
  RespiracionHandle = osThreadCreate(osThread(Respiracion), NULL);

  /* definition and creation of FrecuenciaCardiaca */
  osThreadDef(FrecuenciaCardiaca, tarea8, osPriorityNormal, 0, 256);
  FrecuenciaCardiacaHandle = osThreadCreate(osThread(FrecuenciaCardiaca), NULL);

  /* definition and creation of CalcularOffset */
  osThreadDef(CalcularOffset, tarea9, osPriorityAboveNormal, 0, 128);
  CalcularOffsetHandle = osThreadCreate(osThread(CalcularOffset), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_tarea1 */
/**
 * @brief Function implementing the GraficarECG thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_tarea1 */
void tarea1(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN tarea1 */
    tarea_GraficarECG_Setup ();
    for (;;)
        {
            osSignalWait (1, osWaitForever);
            tarea_GraficarECG_Loop ();
        }
  /* USER CODE END tarea1 */
}

/* USER CODE BEGIN Header_tarea2 */
/**
 * @brief Function implementing the RevisarComandos thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_tarea2 */
void tarea2(void const * argument)
{
  /* USER CODE BEGIN tarea2 */
    tarea_RevisarComandos_Setup ();

    for (;;)
        {
            tarea_RevisarComandos_Loop();
        }
  /* USER CODE END tarea2 */
}

/* USER CODE BEGIN Header_tarea3 */
/**
 * @brief Function implementing the RevisarElectrodos thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_tarea3 */
void tarea3(void const * argument)
{
  /* USER CODE BEGIN tarea3 */
    tarea_RevisarElectrodos_Setup();
    for (;;)
        {
            osSignalWait (1, osWaitForever);
            tarea_RevisarElectrodos_Loop();
        }
  /* USER CODE END tarea3 */
}

/* USER CODE BEGIN Header_tarea4 */
/**
 * @brief Function implementing the controlNeopixel thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_tarea4 */
void tarea4(void const * argument)
{
  /* USER CODE BEGIN tarea4 */
    tarea_ControlNeopixel_Setup();
    for (;;)
        {
            tarea_ControlNeopixel_Loop ();
        }
  /* USER CODE END tarea4 */
}

/* USER CODE BEGIN Header_tarea5 */
/**
 * @brief Function implementing the Normalización thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_tarea5 */
void tarea5(void const * argument)
{
  /* USER CODE BEGIN tarea5 */
    tarea_Normalizacion_Setup();
    for (;;)
        {
            osSignalWait(1, osWaitForever);
            tarea_Normalizacion_Loop();
        }
  /* USER CODE END tarea5 */
}

/* USER CODE BEGIN Header_tarea6 */
/**
 * @brief Function implementing the ControlUSB thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_tarea6 */
void tarea6(void const * argument)
{
  /* USER CODE BEGIN tarea6 */

    tarea_EnvioUSB_Setup();
    for (;;)
        {
            tarea_EnvioUSB_Loop();
        }
  /* USER CODE END tarea6 */
}

/* USER CODE BEGIN Header_tarea0 */
/**
 * @brief Function implementing the LeerADS thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_tarea0 */
void tarea0(void const * argument)
{
  /* USER CODE BEGIN tarea0 */
    //Setup
	debug_ads=804;
    tarea_LeerADS_Setup ();
    debug_ads=805;
    //Loop
    for (;;)
        {
    		debug_ads=806;
            osSignalWait (1, osWaitForever);
            debug_ads=807;
            tarea_LeerADS_Loop ();
            debug_ads=808;
            //osSignalWait (1, osWaitForever);
            //osSignalWait (1, 0x80);
            //debug_ads=807;
        }
  /* USER CODE END tarea0 */
}

/* USER CODE BEGIN Header_tarea7 */
/**
 * @brief Function implementing the Respiracion thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_tarea7 */
void tarea7(void const * argument)
{
  /* USER CODE BEGIN tarea7 */
    tarea_Respiracion_Setup ();
    for (;;)
        {
            osSignalWait (1, osWaitForever);
            tarea_Respiracion_Loop ();
        }
  /* USER CODE END tarea7 */
}

/* USER CODE BEGIN Header_tarea8 */
/**
* @brief Function implementing the FrecuenciaCardiaca thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tarea8 */
void tarea8(void const * argument)
{
  /* USER CODE BEGIN tarea8 */
    tarea_FrecuenciaCardiaca_Setup();
  for(;;)
  {
          osSignalWait(1, osWaitForever);
          tarea_FrecuenciaCardiaca_Loop ();
  }
  /* USER CODE END tarea8 */
}

/* USER CODE BEGIN Header_tarea9 */
/**
* @brief Function implementing the CalcularOffset thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tarea9 */
void tarea9(void const * argument)
{
  /* USER CODE BEGIN tarea9 */
    //Tarea cada 250sps en el ciclo don de NO está Calcular Frecuencia Cardiaca
    tarea_CalcularOffset_Setup ();
  for(;;)
  {
    osSignalWait(1, osWaitForever);
    tarea_CalcularOffset_Loop ();
  }
  /* USER CODE END tarea9 */
}

/* timerOneCallback function */
void timerOneCallback(void const * argument)
{
  /* USER CODE BEGIN timerOneCallback */
    /* Este timer se encarga de detener e iniciar las lecturas en el ADS1298R
     * de manera que evitamos el error de cada 22 segundos*/
    if (flag_MSV_graficar)
        {
            // flag_ADS_DRDY es true mientras se está leyendo el ADS
            // flag_ADS_DRDY es false después de leer el dato, así evitamos corromper la lectura del ADS
            // Si aún se está leyendo el ADS, esperamos a que se libere
            // También se pudiera usar un semáforo

            // Deshabilitar la interrupción para LeerADS
            // Con esto evitamos que haya más solicitudes de lectura del ADS, se desahoga la lectura
            // actual en caso de haber, y se procede a un stop-start seguro del ADS.
            /*
            flag_MSV_graficar = false;      // Evitar más interrupciones
            bool tmp_stat = ads_IntStat (); // Esperar que se desocupe el ADS en caso de estar ocupado
            while (tmp_stat)
                {
                    osDelay (1);
                    tmp_stat = ads_IntStat ();
                }
            // Parar-Iniciar el ADS
            ads_Stop ();
            error_cnt = 0;
            ads_Start ();
            // Volver a habilitar interrupciones y lecturas del ADS
            flag_MSV_graficar = true;
            */

            ////////////////////////////////////////
            // Reset completo del ADS

            flag_MSV_graficar = false;      // Evitar interrupciones
            bool tmp_stat = ads_IntStat (); // Esperar que se desocupe el ADS en caso de estar ocupado
            while (tmp_stat)
                {
                    osDelay (1);
                    tmp_stat = ads_IntStat ();
                }
            //ads_Stop ();                    // El ADS está en modo RDATAC, se requiere SDATAC antes de cualquier comando.
            //adsCmd_reset();                 // Reset por software. ya incluye el tiempo requerido por el reset
            //ads_reset_byPin();
            //HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_RESET);
            //HAL_GPIO_TogglePin(RGB_R2_GPIO_Port,RGB_R2_Pin);
            //osDelay(1);
            ads_Init();                     // Inicializar nuevamente el ADS
            error_cnt = 0;
            ads_Start ();                   // Iniciar las lecturas otra vez
            flag_MSV_graficar = true;       // Volver a habilitar interrupciones y lecturas del ADS


            ////////////////////////////////////////


        }

  /* USER CODE END timerOneCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Podemos tener interrupción por el ADS o por el marcapasos */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
    // En caso del marcapasos y ADS, solo se informará cuando se habilite la lectura
	//HAL_GPIO_WritePin(GPIOE,RGB_B1_Pin, GPIO_PIN_SET);
	//HAL_GPIO_TogglePin(GPIOE, RGB_B1_Pin);
    // 'flag_MSV_graficar == true'
    if (flag_MSV_graficar)
        {
            // Interrupción de ADS
            if (GPIO_Pin == SPI1_DRDY_Pin)
                {
                    ads_IntTrue ();
                    osSignalSet (LeerADSHandle, 1);
                }
            //Interrupción por marcapasos
            if (GPIO_Pin == INTERUPT1_Pin)
                {
                    flag_marcapasos = true;
                    flag_marcapasos_norm = true;
                    cont_flag_mk=0;
                }
        }
}
//void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef *hi2c)
//{
//}
//void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef *hi2c)
//{
//}

int8_t findStr (const char *str1, const char *str2)
{
    int8_t indice = -1;
    if (strstr (str1, str2) == NULL)
        {
            return indice;
        }
    else
        {
            char *pos = strstr (str1, str2);
            indice = (pos - str1);
            return indice;
        }
}


int32_t map (int32_t valor, int32_t in_min, int32_t in_max, int32_t out_min,
             int32_t out_max)
{
    uint32_t resultado = (valor - in_min) * (out_max - out_min)
            / (in_max - in_min) + out_min;
    return resultado;
}

/* USER CODE END Application */
