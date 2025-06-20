/*
 * tarea_LeerADS.c
 *
 *  Created on: May 16, 2023
 *      Author: sakura
 */



/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <arm_math.h>

#include "ads1298.h"
#include "tarea_LeerADS.h"



/* Typedef -------------------------------------------------------------------*/


/* Define --------------------------------------------------------------------*/
#define REV_ELEC_TIME 999 //Revisión cada Segundos

/* Macro ---------------------------------------------------------------------*/


/* Variables -----------------------------------------------------------------*/
const char versionLeerADS[] = LIB_VERSION_LEERADS;

// ID's de las tareas para activar si señal
extern osThreadId GraficarECGHandle;
extern osThreadId NormalizacionHandle;
extern osThreadId RespiracionHandle;
extern osThreadId FrecuenciaCardiacaHandle;
extern osThreadId RevisarElectrodosHandle;
extern osThreadId CalcularOffsetHandle;

extern TIM_HandleTypeDef htim13; // Lo ocupa "delay_us_tim13()" en la librería ads1298.h

// Externos por aparición en el programa principal
extern bool flag_MSV_graficar;

// extern float32_t ads_Resp;  // YA NO SE USA  02/08/2023


extern float32_t ads_I_f;
extern float32_t ads_II_f;
extern float32_t ads_III_f;


extern int32_t ads_I_i;
extern int32_t ads_II_i;

uint8_t counter100sps = 0;
bool flag_250sps = false;

uint32_t timeMark;

uint32_t adsWDCnt = 0;
extern int32_t error_cnt; //debug_error
////variables filtro
float cutoff_I = 4000.0;
float prevFiltered_I = 0.0;
int32_t filteredData_I=0;
int32_t aux_ads_I_i;
/// variables agregadas para adquisición de datos de respiración

extern int32_t ads_Resp_int;
int32_t aux_Resp = 0;
int32_t in_Resp_500 = 0;
int32_t array_rawRes[6] = {0};
extern int      debug_ads;
//float cutoff_Resp = 44000;
//#define M_PI 3.14159265358979323846
//#define SAMPLE_RATE_R 441000

/* Private function prototypes -----------------------------------------------*/


/* Private application code --------------------------------------------------*/

void tarea_LeerADS_Setup(void)
{
//    HAL_TIM_Base_Start (&htim13);
//    osDelay(500);
//    ads_Init();
//    timeMark = HAL_GetTick();
	debug_ads=800;
	HAL_TIM_Base_Start (&htim13);
	osDelay(160);                  // POR 128ms  (Typ: 2^18 x Tclk)
	ads_Init();                    // Establece la referencia interna por primera vez, establece electrodos
	debug_ads=801;
	osDelay(160);                  // Start-up-time referencia interna.
	//ads_Init();                  // Configurar canales (otra vez)
	ads_Start();                   // Lecturas basura
	debug_ads=802;
	//flag_MSV_graficar = true;
	osDelay(200);                  // Descartar basura del ADS durante 160ms
	ads_Stop();                    // Detener el ADS
	debug_ads=803;
	timeMark = HAL_GetTick();
}

// las interrupciones deben ser cada 2ms (500sps)
void tarea_LeerADS_Loop(void)
{
	debug_ads=100;
	//    while(!flag_MSV_graficar)
	//        {
	//            debug_ads=101;
	//            osDelay(50);
	//        }
	if (!flag_MSV_graficar) //La función entró, pero no se debe graficar. Simplemente salimos
	{
		debug_ads=101;
	}
	else // Entramos y si se debe graficar, realizamos todo el proceso
	{
		debug_ads=102;
		bool tmp_stat = ads_IntStat (); //Conocer el estado de la interrupción del ADS
		while (!tmp_stat) // Si no se ha activado la interrupción
		{
			debug_ads=103;
			// Esperamos a que se active la interrupción, mientras tanto ejecutamos una
			// sub-tarea tipo watch-dog
			adsWDCnt++; // Contador que indica (aproximadamente) cuantos ms han pasado desde la última lectura
			if(adsWDCnt > 5) // Mas de 5ms sin lecturas del ADS, aplicar reset
			{
				debug_ads=104;
				adsWDCnt = 0;
				flag_MSV_graficar = false;      // Evitar interrupciones
				ads_Init();                     // Inicializar nuevamente el ADS
				error_cnt = -1;
				ads_Start ();                   // Iniciar las lecturas otra vez
				flag_MSV_graficar = true;       // Volver a habilitar interrupciones y lecturas del ADS
			}
			else //No se ha disparado el watchdog
			{
				debug_ads=105;
				osDelay(1);                  // Esperar hasta el seguiente systick
				tmp_stat = ads_IntStat ();   // Comprobar el estado de la interrupción del ADS
			}
		}
		// Si salimos del bucle ya sea de modo normal o por el reset del ads
		// reiniciamos el contador.
		debug_ads=106;
		adsWDCnt = 0;


		lecturaContinuaDatosAds (); //Dato leído
		ads_IntFalse();

		debug_ads=107;
		//Valores Raw para normalización
		////////////filtro//////////////////////////
		aux_ads_I_i = leerValorCanal (2);
		ads_II_i = leerValorCanal (3);
		float filtered_I = prevFiltered_I + (filter(cutoff_I) * ((float)aux_ads_I_i - prevFiltered_I));
		filteredData_I= (int)filtered_I;
		prevFiltered_I = filtered_I;
		ads_I_i = filteredData_I;
		/////////////////////////////////////////// modificado para respiración 31/07/2023
		in_Resp_500 = leerValorCanal (1);
		array_rawRes[counter100sps] = in_Resp_500;
		counter100sps++;
		////////////filtro//////////////////////////
		//Flotante en mV. El 6.0 es la ganancia del ADS.
		ads_I_f = ((ads_I_i/ ADS_RESOLUTION) * ADS_MULT_OUT) / 6.0;
		ads_II_f = ((ads_II_i/ ADS_RESOLUTION) * ADS_MULT_OUT) / 6.0;
		ads_III_f = ads_II_f - ads_I_f;

		//Los valores de ECG se requieren a 500 sps para los filtros, la graficación será a 250 sps
		osSignalSet(GraficarECGHandle, 1); //tarea1

		// Esta bandera genera muestras a 250 sps, podemos repartir el tiempo:
		// la mitad del tiempo para Normalización y la otra mitad para calculo de frecuencia cardiaca
		// ya que tienen la misma velocidad de muestreo, así no interfieren la una con la otra
		if(flag_250sps)
		{ // Normalización
			debug_ads=108;
			flag_250sps = false;
			osSignalSet(NormalizacionHandle, 1);
			osSignalSet(CalcularOffsetHandle, 1);
		}
		else
		{ // Frecuencia cardiaca
			debug_ads=109;
			flag_250sps = true;
			osSignalSet(FrecuenciaCardiacaHandle, 1);
		}
		// respiración 100 sps (omitir 4 muestras)
		if (counter100sps > 4)
		{
			int32_t max_rawRes = -100000;
			int32_t min_rawRes = 100000000;
			int32_t suma_rawRes = 0;
			debug_ads=110;
			for (int k = 0; k < 5; k++)
			{
				if(array_rawRes[k] > max_rawRes)
					max_rawRes = array_rawRes[k];
				if(array_rawRes[k] < min_rawRes)
					min_rawRes = array_rawRes[k];
				suma_rawRes += array_rawRes[k];
			}
			aux_Resp = (suma_rawRes - max_rawRes - min_rawRes)  / 3;     // modificado 01/09/2023
			counter100sps = 0;
			for (int ki = 0; ki < 5; ki++)
				array_rawRes[ki] = 0;

			ads_Resp_int = aux_Resp & 0X00003FFF;
			osSignalSet(RespiracionHandle, 1);
		}
		//Revisar electrodos. Me aseguro de que no lea el valor del ADS cuando se está escribiendo
		if((HAL_GetTick() - timeMark ) > REV_ELEC_TIME )
		{
			debug_ads=111;
			timeMark = HAL_GetTick();
			osSignalSet(RevisarElectrodosHandle, 1);
		}
	}
	debug_ads=120;

}

