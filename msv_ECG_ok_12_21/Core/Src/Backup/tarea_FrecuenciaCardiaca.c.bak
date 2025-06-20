/*
 * frecuenciaCardiaca.c
 *
 *  Created on: May 16, 2023
 *      Author: Tonatiuh Velazquez Rojas.
 *
 *      Versiones:
 *      1.0.0
 */


#include "FreeRTOS.h"
#include "main.h"
#include <arm_math.h>

#include <tarea_FrecuenciaCardiaca.h>

const char versionFrecuenciaCardiaca[] = LIB_VERSION_FRECUENCIACARDIACA;

//extern float32_t graph1;
extern float32_t ads_II_f;
extern bool flag_FrecC_BEEP_RDY;
extern int16_t frecuenciaCardiaca;
extern int32_t ads_II_i;
extern int32_t aux_der;

extern bool flag_Latido_Offset;

float   valor_fc0=0,valor_fc1=0;
//uint8_t delta_T_ecg         =3;
uint8_t delta_T_ecg         =2;
float   grad_ecg=0,dif_tfc  =0;
bool    flag_ini_pulso      = false;
float   t_fc0=0,t_fc1       =0;
float   frec_card           =0;
float   array_fc[20]        = {60};
int     cont_array_fc       = 0;
int     cont_pos            = 0;
int     cont_neg            = 0;
int     index_array_fc      =0;

float   sum_fc=0;
float prom_fc;



void tarea_FrecuenciaCardiaca_Setup(void)
{

}

void tarea_FrecuenciaCardiaca_Loop (void)
{
    //Cálculo de frecuencia cardiaca
  ///////////////////////nuevo calculo
    valor_fc0 = ads_II_f;
    aux_der=DerivadaFuncion(ads_II_i);
    grad_ecg = (float)flag_ini_pulso;
    if (aux_der>-1000)
        {
            cont_neg=0;
            if (flag_ini_pulso)
                {
                    if (t_fc0 > 0.02)
                        {
                            t_fc1 = HAL_GetTick();
                            dif_tfc = (t_fc1 - t_fc0) / 1000;
                            //array_fc[index_array_fc]=dif_tfc;
                            //index_array_fc++;
                            frec_card = 60 / dif_tfc;
                            flag_ini_pulso = false;

                            if (frec_card < 300)
                                {
                                    //array_fc[cont_array_fc] = frec_card;
                                    array_fc[cont_array_fc] = dif_tfc;
                                    cont_array_fc++;
                                }

                            if (cont_array_fc >= LIM_AR_CARD)
                                {
                                    cont_array_fc = 0;
                                }
                             ///////////////******************************************************//////////////////
                            sum_fc = 0;
                            float max_fc = 0;
                            for (int ij = 0; ij < LIM_AR_CARD; ij++)
                                {
                                    if (array_fc[ij] > max_fc)
                                        {
                                            max_fc = array_fc[ij];
                                        }
                                    sum_fc += array_fc[ij];
                                }
                            //sum_fc = sum_fc - max_fc;
                            prom_fc = sum_fc / (LIM_AR_CARD );
                            prom_fc = 60/prom_fc;
                            ///////////////////////nuevo calculo
                            valor_fc1=valor_fc0;
                            ///////////////******************************************************//////////////////
                            //Envío de señal de sincronización
                            //msgFrecuenciaCardiaca_cnt = sprintf ((char*) msgFrecuenciaCardiaca, "F:%.1f\n",round (prom_fc));
                            frecuenciaCardiaca =  (int16_t)round(prom_fc);
                            flag_FrecC_BEEP_RDY = true;
                            flag_Latido_Offset = true;
                        }
                    t_fc0 = HAL_GetTick();
                }

        }
    else if (aux_der<-1500)
        {
            cont_neg++;
            if(cont_neg>2)
                {
                    flag_ini_pulso = true;
                }
        }



}
int32_t DerivadaFuncion (int derivacion)
{
    int y, i;
    static int fx_derv[4];

    /*y = 1/8 (2x( nT) + x( nT - T) - x( nT - 3T) - 2x( nT - 4T))*/
    y = (derivacion << 1) + fx_derv[3] - fx_derv[1] - (fx_derv[0] << 1);
    y >>= 3;
    for (i = 0; i < 3; i++)
        fx_derv[i] = fx_derv[i + 1];
    fx_derv[3] = derivacion;

    return(y);
}
