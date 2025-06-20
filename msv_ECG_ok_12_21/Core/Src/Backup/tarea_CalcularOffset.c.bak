/*
 * CalcularOffset.c
 *
 *  Created on: Jul 10, 2023
 *      Author: sakura
 */
#include "main.h"
#include "tarea_CalcularOffset.h"     // Tarea 9

extern bool flag_Latido_Offset;
extern int32_t ads_I_i;

int64_t ads_Lead_I_SUM = 0;
uint32_t ads_count = 1;
extern int64_t offsetRaw;

void tarea_CalcularOffset_Setup (void)
{

}



void tarea_CalcularOffset_Loop (void)
{
    // Caĺculo de offset en el latido actual
    // Promedio de la señal en el periodo de 1 latido completo
    if(flag_Latido_Offset)
        {
            // Borrar bandera
            flag_Latido_Offset = false;
            // Calcular offset
            offsetRaw = (int64_t)((float)ads_Lead_I_SUM /(float)ads_count);
            // Reiniciar variables del cálculo
            ads_Lead_I_SUM = 0;
            ads_count = 1; //Evitar división entre 0, en grandes cantidades de muestras, este 1 es despreciable
        }
    else
        {
            ads_Lead_I_SUM += ads_I_i;
            ads_count++;
        }

}
