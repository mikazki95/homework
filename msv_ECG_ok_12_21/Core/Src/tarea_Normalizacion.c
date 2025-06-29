/*
 * tarea_Normalizacion.c
 *
 *  Created on: 25 may 2023
 *      Author: Tonatiuh Velazquez Rojas.
 */


#include "FreeRTOS.h"
#include "main.h"

#include "tarea_Normalizacion.h"

const char versionNormalizacion[] = LIB_VERSION_NORMALIZACION;

extern UART_HandleTypeDef huart2;

extern int32_t ads_delta;
extern int32_t ads_I_i;
extern int32_t ads_II_i;
extern bool flag_marcapasos_norm;       // flag de marcapasos para normalizado | Envío USB          |                     |
extern int32_t aux_deb_norm;
//extern bool flag_marcapasos_int;       // flag de marcapasos para normalizado | Envío USB          |
int32_t aux_1;
int32_t aux_2;
int32_t index_array = 0;
int32_t array_aux_1_max[20] = { -2147483647 };
int32_t array_aux_1_min[20] = { 2147483647 };
int32_t array_aux_2_max[20] = { -2147483647 };
int32_t array_aux_2_min[20] = { 2147483647 };
int index_max_min=0;
int long_act = 0;
int index_long_act = 0;

int32_t long_array = 150;
bool flag_array = false;
int32_t Dnorm_1;
int32_t Dnorm_2;
int32_t Dnorm_2_ant;

int32_t cont_max_1      = 0;
int32_t cont_max_2      = 0;
int32_t value_1_max     = -2147483647;
int32_t value_2_max     = -2147483647;
int32_t cont_min_1      = 0;
int32_t cont_min_2      = 0;
int32_t value_1_min     = 2147483647;
int32_t value_2_min     = 2147483647;
int32_t aux_1_min_norm  = 2147483647;
int32_t aux_1_max       = -2147483647;
int32_t max_1           = -2147483647;
int32_t aux_2_min_norm  = 2147483647;
int32_t aux_2_max       = -2147483647;
int32_t max_1_abs       = -2147483647;
int32_t max_2_abs       = -2147483647;
int32_t min_1_abs       = 2147483647;
int32_t min_2_abs       = 2147483647;
int32_t max_1_full       = -2147483647;
int32_t max_2_full       = -2147483647;
int32_t min_1_full       = 2147483647;
int32_t min_2_full       = 2147483647;
int32_t min_1_fa        = 2147483647;
int32_t min_2_fa        = 2147483647;
int32_t prom_1          = 0;
int32_t prom_2          = 0;
int32_t aux_send        = 0;
int     cont_max        = 4;
int     cont_min        = 9;
int     delta_max       = 50;
int 	flag_asistole 	= 0;
int32_t factor_x = 1000;
uint8_t Dato_2[24];

//uint16_t ch1Raw;  //Valor instantáneo leído del canal 1 (respiración)
float aux_graph = 0, aux_graph_2 = 0;       //para debuggear, borrar al terminar

int32_t i_array = 0;

/////////////////////////////////variables filtro
float cutoff = 4000.0;
float prevFiltered_norm_I = 0.0;
float prevFiltered_norm_II = 0.0;
int i = 0;
int norm_Count = 0;
int dataCount = 0;
//float data[900];
//int data[900];
int32_t data_I=0;
int32_t data_II=0;
int32_t delta_array[4]={0};
int delta_int =0;
//int filteredData[900];
int32_t filteredData_norm_I=0;
int32_t filteredData_norm_II=0;
int NUM_POINTS = 900;
int r=0;
int span_delta = 3;
int derv_norm_I=0;
int derv_norm_II=0;
#define M_PI 3.14159265358979323846
#define SAMPLE_RATE 44100
extern int cont_flag_mk;
//#define SAMPLE_RATE 250

/*   Normalizacion definido en tarea 5
 Necesita las variables de int 32 del adc 2 y 3 que corresponden a las derivaciones I y II
 Tiene como salida que va al analizador de arritmias por usart2 con un vector Dato_2 que convierte los int32 en uint8 para su envió
 */
void tarea_Normalizacion_Setup(void)
{

}
float filter(float cutofFreq);


int32_t DerivadaNormFuncion_I (int derivacion);
int32_t DerivadaNormFuncion_II (int derivacion);

void tarea_Normalizacion_Loop(void)
{
    /////////////////////////////////////////inicio filtro
    /*
    aux_1 = ads_I_i;
    aux_2 = ads_II_i;
    ////////////////////////////////////////////////////////////////////////////*/
    data_I  = ads_I_i;
    data_II  = ads_II_i;
    //aux_2 = ads_II_i;
    //aux_2  = ads_I_i;
    float filtered_I = prevFiltered_norm_I + (filter(cutoff) * ((float)data_I - prevFiltered_norm_I));
    float filtered_II = prevFiltered_norm_II + (filter(cutoff) * ((float)data_II - prevFiltered_norm_II));

    //////////////////////////////////////////////////////////////////////////////////////*/
    filteredData_norm_I= (int32_t)filtered_I;
    prevFiltered_norm_I = filtered_I;
    aux_1 = filteredData_norm_I;

    filteredData_norm_II= (int32_t)filtered_II;
    prevFiltered_norm_II = filtered_II;
    aux_2 = filteredData_norm_II;

    derv_norm_I=abs(DerivadaNormFuncion_I(aux_1));
    derv_norm_II=abs(DerivadaNormFuncion_II(aux_2));
    /////////////////////////////////////////////////////////////////fin filtro
    index_array++;
    if (index_array >= long_array)
        {
            if (aux_1_max!=-2147483647)
                {
                    array_aux_1_max[index_max_min] = aux_1_max;
                }
            if (aux_2_max!=-2147483647)
                {
                    array_aux_2_max[index_max_min] = aux_2_max;
                }
            if (aux_1_min_norm!=2147483647)
                {
                    array_aux_1_min[index_max_min] = aux_1_min_norm;
                }
            if (aux_2_min_norm!=2147483647)
                {
                    array_aux_2_min[index_max_min] = aux_2_min_norm;
                }
            //array_aux_1_max[index_max_min] = aux_1_max;
            //array_aux_2_max[index_max_min] = aux_2_max;
            //array_aux_1_min[index_max_min] = aux_1_min_norm;
            //array_aux_2_min[index_max_min] = aux_2_min_norm;
            index_max_min++;
            if (index_max_min>9)
                index_max_min=0;

            index_long_act = 0;
            value_1_max     = -2147483547;
            value_2_max     = -2147483547;
            aux_1_max       = -2147483647;
            aux_2_max       = -2147483647;
            aux_1_min_norm  = 2147483647;
            aux_2_min_norm  = 2147483647;
            index_array=0;
            //aux_deb_norm=0;
        }


    //if (aux_1 > (value_1_max-delta_max))
    if ((derv_norm_I>800)&&(aux_1>min_1_abs))
        {
            cont_max_1++;
            if((aux_1 > aux_1_max)&&(cont_max_1>cont_max))
                {
                    aux_1_max = aux_1;
                }
            value_1_max = aux_1;

        }
    else
        {
            value_1_max     = -2147483547;
            cont_max_1=0;
        }
        if (derv_norm_I<100)
        {
            cont_min_1++;
            if((aux_1 < aux_1_min_norm)&&(cont_min_1>cont_min))
                {
                    aux_1_min_norm = aux_1;
                    //aux_deb_norm=1;
                }
            value_1_min = aux_1;

        }
    else
        {
            value_1_min     = 2147483647;
            cont_min_1=0;
        }

    //if (aux_2 > (value_2_max-delta_max))
    if ((derv_norm_II>800)&&(aux_2>min_2_abs)&&(!flag_marcapasos_norm))
        {
            cont_max_2++;
            if((aux_2 > aux_2_max)&&(cont_max_2>cont_max))
                {
                    aux_2_max = aux_2;
                }
            value_2_max = aux_2;

        }
    else
        {
            value_2_max     = -2147483547;
            cont_max_2=0;
        }

    if (derv_norm_II<100)
        {
            cont_min_2++;
            if((aux_2 < aux_2_min_norm)&&(cont_min_2>cont_min))
                {
                    aux_2_min_norm = aux_2;
                }
            value_2_min = aux_2;

        }
    else
        {
            value_2_min     = 2147483647;
            cont_min_2=0;
        }
////////////////////////////////////////////////////////////////////////////////////////////////
    if (aux_1 > max_1_full)
        {
            max_1_full = aux_1;

        }
    if (aux_2 > max_2_full)
        {
            max_2_full = aux_2;

        }

    if (aux_1<min_1_full)
        {
            min_1_full = aux_1;
        }
    if (aux_2<min_2_full)
        {

            min_2_full = aux_2;

        }

///////////////////////////////////////////////////////////////////////////////////////////
    /*
    if((aux_2 < aux_2_min_norm))
        {
            aux_2_min_norm = aux_2;
        }*/
    min_1_abs       = 2147483647;
    min_2_abs       = 2147483647;
    max_1_abs       = -2147483647;
    max_2_abs       = -2147483647;
    int ij=0;
    for (ij = 0; ij < 9; ij++)
        {
            if(array_aux_1_max[ij]>max_1_abs)
                max_1_abs=array_aux_1_max[ij];
            if(array_aux_2_max[ij]>max_2_abs)
                max_2_abs=array_aux_2_max[ij];
            if(array_aux_1_min[ij]<min_1_abs)
                min_1_abs=array_aux_1_min[ij];
            if(array_aux_2_min[ij]<min_2_abs)
                min_2_abs=array_aux_2_min[ij];
        }
    //aux_deb_norm=array_aux_1_max[0];
    dataCount++;
    if(dataCount>500)
        {
            dataCount=0;
            min_1_fa=min_1_full;
            min_2_fa=min_2_full;
            max_1_full      = -2147483647;
            max_2_full      = -2147483647;
            min_1_full      = 2147483647;
            min_2_full      = 2147483647;
        }
    prom_1=min_1_abs;
    prom_2=min_2_abs;

//////////////////////////////////////////////////////////////////////////////////////////////*/
    Dnorm_1=0;//
    Dnorm_2=0;//
    aux_send=max_1_full-min_1_full;
    if ((max_1_full-min_1_full<5000)&&(norm_Count<5000))
        {
            norm_Count++;
        }
    if(max_1_full-min_1_full>3500)
        norm_Count=0;
/////////////////////////asistole noralizacion///////////////////////////////////
    /*
    //if (((max_2_abs-prom_2)>4500)&&((max_1_abs-prom_1)>300))//10000
    for (int var = 0; var < 2; var++)
        {
            if((prom_2-min_2_fa)>20000)
                {
                    //prom_2=prom_2-1000;
                    var=0;
                }
        }
    for (int var_1 = 0; var_1 < 2; var_1++)
        {
            if((prom_1-min_1_fa)>6000)
                {
                    //prom_1=prom_1-1000;
                    var_1=0;
                }
        }
        */
    /*
    if((prom_2-min_2_fa)>20000)
        prom_2-3000;*/
    if (norm_Count<375)//10000
        {
            Dnorm_2 = (aux_2 - prom_2) * (factor_x) / (max_2_abs - prom_2);
            Dnorm_1=(aux_1 - prom_1)*(factor_x) / (max_1_abs - prom_1);
            flag_asistole = 0;
        }
    else
    {
        flag_asistole = 1;
    }
    //aux_deb_norm=prom_1-min_1_fa;
    aux_deb_norm=min_2_abs;//derv_norm_I;//
    //Dnorm_1=max_2_abs;
    Dato_2[10] = 0XAA ;
    Dato_2[11] = 0X00 ;

    //if (flag_marcapasos_int)
    if (flag_marcapasos_norm)
    //if (Dnorm_2>1400)
        {
            cont_flag_mk++;
            if(cont_flag_mk>3)
                {
                    //flag_marcapasos_int     = false;
                    flag_marcapasos_norm    = false;
                }
            Dato_2[11] =  0x55;
            Dnorm_2 = 0;
        }
    Dnorm_2_ant = Dnorm_2;


    Dato_2[0] = 0xA5;

    Dato_2[1] = ((Dnorm_1 & 0xFF000000) >> 24); // rec de los bytes r y combinarlos con la mascara de comando
    Dato_2[2] = ((Dnorm_1 & 0xFF0000) >> 16); // rec de los bytes r y combinarlos con la mascara de comando
    Dato_2[3] = ((Dnorm_1 & 0xFF00) >> 8); // rec de los bytes r y combinarlos con la mascara de comando
    Dato_2[4] = (Dnorm_1 & 0xFF); // rec de los bytes r y combinarlos con la mascara de comando

    Dato_2[5] = 0x0A; // rec de los bytes r y cmbinarlos con la mascara de comando

    Dato_2[6] = ((Dnorm_2 & 0xFF000000) >> 24); // rec de los bytes r y combinarlos con la mascara de comando
    Dato_2[7] = ((Dnorm_2 & 0xFF0000) >> 16); // rec de los bytes r y combinarlos con la mascara de comando
    Dato_2[8] = ((Dnorm_2 & 0xFF00) >> 8); // rec de los bytes r y combinarlos con la mascara de comando
    Dato_2[9] = (Dnorm_2 & 0xFF); // rec de los bytes r y combinarlos con la mascara de comando




    //Dato_2[12] = 0x0A;
   // HAL_UART_Transmit_IT (&huart2, Dato_2, 13);

    Dato_2[12]=Dato_2[0];
    Dato_2[13]=Dato_2[1];
    Dato_2[14]=Dato_2[2];
    Dato_2[15]=Dato_2[3];
    Dato_2[16]=Dato_2[4];
    Dato_2[17]=Dato_2[5];
    Dato_2[18]=Dato_2[6];
    Dato_2[19]=Dato_2[7];
    Dato_2[20]=Dato_2[8];
    Dato_2[21]=Dato_2[9];
    Dato_2[22]=Dato_2[10];
    Dato_2[23]=Dato_2[11];
    HAL_UART_Transmit_IT (&huart2, Dato_2, 24);


}

float filter(float cutofFreq)
{
    float RC = 1.0 / (cutofFreq * 2 * M_PI);
    float dt = 1.0 / SAMPLE_RATE;
    float alpha = dt / (RC + dt);
    return alpha;
}

int32_t DerivadaNormFuncion_I (int derivacion)
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
int32_t DerivadaNormFuncion_II (int derivacion)
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


