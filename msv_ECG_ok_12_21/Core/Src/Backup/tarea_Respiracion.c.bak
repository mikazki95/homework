/*
 * tarea_Respiracion.c
 *
 *  Created on: May 23, 2023
 *
 * Modificación en graficación de señal de respiración  21/08/2022
 */


#include "FreeRTOS.h"
#include "main.h"
#include <arm_math.h>

#include "tarea_Respiracion.h"

const char versionRespiracion[] = LIB_VERSION_RESPIRACION;

//Nuevas variables para los tiempos
extern bool 	flag_Resp_RDY;
extern bool  	msgApnea;     //Mensaje W:0\n o W:1\n
extern float   	Respiration_Rate;
extern float 	original_res;     // solo para debuggear
extern int32_t 	ads_Resp_int;
extern bool 	flag_activa_apnea;   // Informa la activación de la alarma de apnea
extern int  	valor_apnea;       // se establecen 10 segundos para pruebas
extern int      variable_auxiliar;
extern float    variable_auxiliar2;

extern float32_t graph3;
extern int32_t 	 aux_grap_resp;
extern arm_fir_instance_f32 filtroResp_Instancia;    //Instancia del filtro
extern uint8_t	 paciente_tipo;

int tiempo_apnea;

float32_t salidaFiltroRespiracion;
float arraySuavizado[50] = { 0 };
float respiracionSuavizada;
float32_t raw_prefiltred;
float  suavizada_ant;

bool 	flag_alarmApnea = false;
bool 	flag_inhal = false;
bool 	flag_inhal_2 = false;
bool 	flag_inhal_3 = false;

bool  	flag_NoResp = true;
bool  	flag_estaticgraph = false;
bool  	flag_envio = false;
bool  	flag_graficar = true;
bool  	flag_apnea = false;

int32_t 	xnt_sqm[10] = {0};
int32_t 	ynt_sqm = 0;
uint16_t 	cta_apnea_ON = 0;
uint16_t 	cta_ap = 0;
uint8_t  	cta_ciclo = 5;

int  	max_fr = 0, min_fr = 400;
float 	valor_rpm = 0;
int 	array_rpm[12] = {0};
float 	tiempo_rpm1 = 0,tiempo_rpm0 = 0;
int   	sum_fr = 0;
float 	prom_fr = 20;
float 	dxnt [1100] = {0};

float 	max_RespSuave = -100000;        // variables para max y min de la respiración
float 	min_RespSuave = 100000;

float 	cutoff_Resp = 2;
float 	filteredResp = 0;
float 	preFiltResp = 0;
float 	max_Resp_2 = 0; 			// variable para almacenar el valor máximo
float 	min_Resp_2 = 1000000;   	// variable para almacenar el valor mínimo
float 	max_signal,min_signal;
float 	dif_Maxmin = 2000;
float 	delta_RPM = 0;
float 	alpha=0;
float 	max_ciclo = 0;
float 	min_ciclo = 1000000;
float 	delta_Z;
float 	delta_Z0;

int   	cta_maximo2 = 0;
int   	cta_minimo2 = 0;
int   	cuenta_Nograph = 0;
int 	offset_frec=1;
int   	cuenta_maximo = 0;
int   	cta_valida_resp = 0;
int   	cuenta_tiempo = 0;
int   	cta_graph_resp = 0;
int   	val_rpm_1;
int   	val_rpm_0 = 0;
int	  	dif_rpm = 0;
int   	orden_filt;	// = 18;//36;
int		lim_apnea=250;
int		aux_dif = 300;
int  	tiempo_array = 500;
int 	flag_max=0;


arm_fir_instance_f32 filtroResp_Instancia;    //Instancia del filtro
float32_t filtroResp_Estados[FILTRO_RESP_TAM_BLOQUE + FILTRO_RESP_ORDEN - 1];
float32_t filtroResp_Coeficientes[FILTRO_RESP_ORDEN] = // Coeficientes del filtro
{
		0.00301243476954899f,0.00160824311592055f,-0.00666305297536139f,-0.023056940847839f,-0.0251341255323886f,0.023361800448292f,0.130328339006718f,
		0.247280201779489f,0.298526200471241f,0.247280201779489f,0.130328339006718f,0.023361800448292f,-0.0251341255323886f,-0.023056940847839f,
		-0.00666305297536139f,0.00160824311592055f,0.00301243476954899f
};


/* Private function prototypes -----------------------------------------------*/


void tarea_Respiracion_Setup (void)
{
    //Inicialización del filtro de respiración
    arm_fir_init_f32 (&filtroResp_Instancia, FILTRO_RESP_ORDEN, (float32_t*) &filtroResp_Coeficientes[0], &filtroResp_Estados[0], FILTRO_RESP_TAM_BLOQUE);
    alpha = 2*PI_R*cutoff_Resp / 100;
}


void tarea_Respiracion_Loop (void)
{
	tiempo_apnea = (valor_apnea * 100) - tiempo_array;

	if (paciente_tipo == 0)
	{
		orden_filt = FILTRO_ADULTO;
		lim_apnea=1500;
	}
	else if (paciente_tipo == 1)
	{
		orden_filt = FILTRO_PEDIAT;
		lim_apnea=1050;
	}
	else if (paciente_tipo == 2)
	{
		orden_filt = FILTRO_NEONATO;
		lim_apnea=850;
	}


	cuenta_tiempo++;
	if (cuenta_tiempo > 300)		// 3 segundos
	{
		flag_envio = true;
		cuenta_tiempo = 0;
	}
    // preprocesamiento de señal de entrada (suavizado y pre-filtrado)


    for (int is = 8; is > 0; is--)
    {
    	xnt_sqm[is] = xnt_sqm[is-1];
    }
    xnt_sqm[0] = ads_Resp_int;

    ynt_sqm = (-21*xnt_sqm[0] + 14*xnt_sqm[1] + 39*xnt_sqm[2] + 54*xnt_sqm[3] + 59*xnt_sqm[4] + 54*xnt_sqm[5]+ 39*xnt_sqm[6] + 14*xnt_sqm[7] -21*xnt_sqm[8])/231;

    float respira_in = (float)ynt_sqm;

    //////filtro
    filteredResp = preFiltResp + (alpha * (respira_in - preFiltResp));
    preFiltResp = filteredResp;

    raw_prefiltred = filteredResp;

    //filtrado de respiración
    arm_fir_f32 (&filtroResp_Instancia,&raw_prefiltred,&salidaFiltroRespiracion,FILTRO_RESP_TAM_BLOQUE);

    //Suavizado de la respiración filtrada
    respiracionSuavizada = 0;
    float max_suav = -340282346600000000;
    float min_suav = 3402823466000000000;//10000000;
    //float min_suav = 340282346600000000000000000000000000000000000000;//10000000;
    for (int i = (orden_filt - 1); i > 0; i--)
    {
    	arraySuavizado[i] = arraySuavizado[i - 1];
    	respiracionSuavizada += arraySuavizado[i];
    	if (arraySuavizado[i] > max_suav)
    		max_suav = arraySuavizado[i];
    	if (arraySuavizado[i] < min_suav)
    		min_suav = arraySuavizado[i];
    }

    arraySuavizado[0] = salidaFiltroRespiracion;
    respiracionSuavizada += arraySuavizado[0] - max_suav - min_suav;
    respiracionSuavizada = respiracionSuavizada / (orden_filt - 2);

    dxnt[cuenta_maximo] = respiracionSuavizada;
    cuenta_maximo++;
    if (cuenta_maximo > tiempo_array)
    {
    	cuenta_maximo=0;
    }
    max_RespSuave = -100000;
    min_RespSuave = 100000;

    for (int idr = 0; idr < tiempo_array; idr++)
    {
    	if(dxnt[idr] > max_RespSuave)
    	{
    		max_RespSuave = dxnt[idr];
    	}
    	if(dxnt[idr] < min_RespSuave)
    	{
    		min_RespSuave = dxnt[idr];
    	}
    }

    dif_Maxmin = max_RespSuave - min_RespSuave;

    aux_dif = (int)dif_Maxmin;
    //delta_RPM = fabs((dif_rpm)*100 / val_rpm_0);
/*
   if (((delta_RPM < 20) || (fabs(dif_rpm) < 5)) && (aux_dif > 100))
    {
    	cta_valida_resp--;
    	if(cta_valida_resp <= 0)
    	{
    		cta_valida_resp = 0;
    		flag_apnea = false;

    	}
    	//flag_graficar = true;
    	cta_graph_resp = 0;
    }
    else
    {
    	cta_valida_resp++;
    	if(cta_valida_resp >= 3)
    	{
    		cta_valida_resp = 3;
    	}
    	flag_apnea = flag_activa_apnea;

    /*
        cta_graph_resp++;
    	if (cta_graph_resp >= 200)
    	{
    		flag_graficar = false;
    		cta_graph_resp = 200;
    	}


    }

*/
    //////////////////  Identificación de fase se inhalación y exhalación
    if (((respiracionSuavizada > ((dif_Maxmin*0.6) + min_RespSuave))) && (!flag_inhal_2))
    {
    	flag_inhal_2 = true;
    }
    else if (((respiracionSuavizada < ((dif_Maxmin*0.4) + min_RespSuave))) && (flag_inhal_2))
    {
    	flag_inhal_2 = false;
    }

    if (flag_inhal_2)
    {
    	if(flag_max==0)
    	{
    		flag_max=1;
    		delta_Z=max_Resp_2-min_Resp_2;
    		variable_auxiliar=100*fabs(delta_Z-delta_Z0)/delta_Z;
    		//Respiration_Rate=100*(fabs(dif_rpm)/val_rpm_0);
			//variable_auxiliar=delta_Z0;
			if (((fabs(delta_Z-delta_Z0)/delta_Z)<.20)&&(100*(fabs(dif_rpm)/val_rpm_0)<20))
			{
				cuenta_Nograph--;
				if (cuenta_Nograph <=0)
				{
					flag_graficar=true;
					flag_apnea = false;
					cuenta_Nograph = 0;
				}
			}
			else
			{
				cuenta_Nograph++;
				if (cuenta_Nograph > (3+(offset_frec/20)))
				{
					flag_graficar=false;
					flag_apnea = flag_activa_apnea;
					//if (cuenta_Nograph > 4)
						cuenta_Nograph=(3+(offset_frec/20));//4;
				}
			}
			delta_Z0=delta_Z;
    		//if(delta_Z>190)//lim_apnea
    		/*
    		if(delta_Z>lim_apnea)//lim_apnea
    		{
    			cta_maximo2++;
    			if(cta_maximo2>=3)
    			{
    				cta_maximo2=3;
    				flag_graficar=true;
    			}
    		}
    		else
    		{
    			//cta_maximo2=0;
    			//flag_graficar=false;
    			cta_maximo2--;
				if(cta_maximo2<=0)
				{
					cta_maximo2=0;
					flag_graficar=false;
				}
    		}
    		*/
    		max_Resp_2 = respiracionSuavizada;
    		min_Resp_2 = respiracionSuavizada;
    	}
    	if (respiracionSuavizada > max_Resp_2)
		{
			max_Resp_2 = respiracionSuavizada;
		}

    	/*
    	if (respiracionSuavizada > max_Resp_2)
    	{
    		max_Resp_2 = respiracionSuavizada;
    		cta_maximo2 = 0;
    	}
    	else
    	{
    		if (cta_minimo2 == 8)
    		{
    			min_signal = min_Resp_2;
    			max_ciclo = max_Resp_2;
    		}
    		cta_minimo2++;
    	}
*/
    }
    else	// exhalación
    {
    	flag_max=0;
    	if (respiracionSuavizada < min_Resp_2)
		{
			min_Resp_2 = respiracionSuavizada;
			cta_minimo2 = 0;
		}
    	/*
    	 *
    	if (respiracionSuavizada < min_Resp_2)
		{
			min_Resp_2 = respiracionSuavizada;
			cta_minimo2 = 0;
		}
    	else
    	{
    		if (cta_maximo2 == 8)
    		{
    			max_signal = max_Resp_2;
    		}
    		cta_maximo2++;
    	}
    	*/
    }

    ////////////// Calculo de frecuencia respiratoria
    if ((flag_inhal_2) && (!flag_inhal))  //
    {
    	flag_inhal = true;
    	flag_inhal_3 = true;
    	tiempo_rpm1 = HAL_GetTick();
    	valor_rpm = 60000/(tiempo_rpm1-tiempo_rpm0);
    	tiempo_rpm0 = HAL_GetTick();
    	if (valor_rpm < 200)
    	{
    		val_rpm_1 = round(valor_rpm);
    		dif_rpm = val_rpm_1 - val_rpm_0;
    		if (val_rpm_0 > 0)
    			delta_RPM = fabs((dif_rpm)*100 / val_rpm_0);

    		val_rpm_0 = val_rpm_1;
    		if ((dif_rpm < TOL_RPM) && (dif_rpm > -TOL_RPM))
    		{
    			for (int irp = (LIM_AR_RESP-1); irp > 0; irp--)
    			{
    				array_rpm[irp] = array_rpm[irp - 1];
    			}
    			array_rpm[0] = round(valor_rpm);
    		}
    		sum_fr = 0;
    		max_fr = 0;
    		min_fr = 500;
    		for (int ij = 0; ij < LIM_AR_RESP; ij++)
    		{
    			if (array_rpm[ij] > max_fr)
    			{
    				max_fr = array_rpm[ij];
    			}
    			if (array_rpm[ij] < min_fr)
    			{
    				min_fr = array_rpm[ij];
    			}
    			sum_fr += array_rpm[ij];
    		}
    		prom_fr = (sum_fr - max_fr - min_fr) / (LIM_AR_RESP-2);
    	}

    	//delta_Z = max_ciclo - min_Resp_2;
    	//variable_auxiliar = cta_maximo2;
    	//variable_auxiliar = (int)delta_Z;
    	max_ciclo = 0;
    	min_ciclo = 1000000;
    }
    else if (!flag_inhal_2)
    {
    	flag_inhal = false;
    }


    //if (cta_maximo2>=1)
    if (flag_graficar)
    {
    	graph3 = respiracionSuavizada;
    }

    else
    {
    	graph3 = respiracionSuavizada;
    }

    //////////////   Detección y aviso de apnea

    if (flag_apnea)		// si esta activada la alarma de apnea & la bandera de apnea
    {
    	cta_apnea_ON++;
    	if (cta_apnea_ON > tiempo_apnea)
    	{
    		flag_alarmApnea = true;
    		cta_ap++;
    		if (cta_ap == 1)
    		{
    			msgApnea = true;
    			for (int irp = (LIM_AR_RESP-1); irp >= 0; irp--)
    			{
    				array_rpm[irp] = 0;
    			}
    	//		Respiration_Rate = 0;
    			cta_ciclo = 0;
    		}
    		Respiration_Rate = 0;
    		if (cta_ap > 25000)
    			cta_ap = 2;
    	}
    }
    else
    {
    	if(flag_inhal_3)
    	{
    		flag_inhal_3 = false;
    		cta_ciclo++;
    		if (cta_ciclo > 5000)
    			cta_ciclo = 0;
    		cta_apnea_ON = 0;
    		//flag_graficar = true;
    	}
    	if ((cta_ciclo > 3) && (flag_alarmApnea))
    	{
    		flag_alarmApnea = false;
    		msgApnea = false;
    		cta_ap = 0;
    	}
    }


    /////////// Envio de variables para graficación


    // Información generada:

    if (flag_envio)	// actualización de valor RPM cada 3 segundos
    {
    	Respiration_Rate = round(prom_fr);
    	flag_envio = false;
    }

  //  variable_auxiliar = cta_ciclo;
    original_res = dif_Maxmin;
    aux_grap_resp = round(prom_fr);		//round(min_RespSuave);//ads_Resp_int;  round(min_signalResp)
    variable_auxiliar2 = (int) flag_apnea * 10;	//dif_Maxmin;

    flag_Resp_RDY = true; // Indicar que el cálculo ha concluido

}


