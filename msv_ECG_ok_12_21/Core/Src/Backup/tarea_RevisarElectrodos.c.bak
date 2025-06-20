/*
 * tarea_RevisarElectrodos.c
 *
 *  Created on: 24 may 2023
 *      Author: sakura
 *
 *
 *      //Cadena de información L:1 2 3 4 5\n (sin espacios) electrodos del 1 al 5
    //0 -> Desconectado
    //1 -> Conectado
    //Sólo el estado de 5 electrodos
    //P.ej: L:11111\n

     * Canal IN_P IN_N
     * 1 V6/RESP+ WCT/ RESP-
     * 2 LA RA
     * 3 LL RA
     * 4 V2 WCT
     * 5 V3 WCT
     * 6 V4 WCT
     * 7 V5 WCT
     * 8 V1 WCT

 24 de mayo de 2023: Gandy me indica que no necesita saber en específico qué electrodo se desconectó.
                     Al software de la raspberry al ver agún electrodo desconectado sin importar cúal,
                     ya define que hacer, No necesita conecer el electrodo en particular
 */

#include "main.h"
#include "cmsis_os.h"
#include "ads1298.h"

#include "tarea_RevisarElectrodos.h"

const char versionRevisarElectrodos[] = LIB_VERSION_REVISARELECTRODOS;
// Indica si el programa principal está activo
extern bool flag_MSV_graficar;
// Electrodos
extern bool flag_RevElec_RDY;
extern bool lead_1_RA;
extern bool lead_2_LA;
extern bool lead_3_RL;
extern bool lead_4_LL;
extern bool lead_5_V1;

// Estado desde el ADS
uint32_t statusWord;   // Estado reportado por el ADS
uint8_t loff_statP;    // Registro con los estados de los electrodos conectados a las terminales P de los canales del ADS
uint8_t loff_statN;    // Registro con los estados de los electrodos conectados a las terminales N de los canales del ADS

void tarea_RevisarElectrodos_Setup(void)
{

}
void tarea_RevisarElectrodos_Loop(void)
{
    // Sólo reportar cuando se está graficando
    if (flag_MSV_graficar)
        {
            // Suponemos que los electrodos están conectados, lo comprobaremos con statusWord
            lead_1_RA = true;
            lead_2_LA = true;
            lead_3_RL = true;
            lead_4_LL = true;
            lead_5_V1 = true;

            statusWord = leerValorCanal (0);            //Tomo el último valor reportado de statusword en el ADS
            loff_statP = (statusWord & 0x0ff000) >> 12; //Aplicar máscara del byte útil y recorrer 12 pos
            loff_statN = (statusWord & 0x000ff0) >> 4;  //Aplicar máscara del byte útil y recorrer 4 pos

            if (loff_statN & 0b00000010) // RA, CH2_N
                {
                    lead_1_RA = false;
                }
            if (loff_statP & 0b00000010) // LA, CH2_P
                {
                    lead_2_LA = false;
                }
            if (loff_statP & 0b00000100) // LL, CH3_P
                {
                    lead_4_LL = false;
                }
            if (loff_statP & 0b10000000) // V1, CH8_P
                {
                    lead_5_V1 = false;
                }
            /*if ((lead_1_RA)&&(lead_2_LA)&&(lead_4_LL)&&(lead_5_V1))
                HAL_GPIO_WritePin(MK_Reset_GPIO_Port, MK_Reset_Pin, GPIO_PIN_RESET);
            else
                HAL_GPIO_WritePin(MK_Reset_GPIO_Port, MK_Reset_Pin, GPIO_PIN_SET);*/
            flag_RevElec_RDY = true;
            //osDelay (5000);
        }
    else
        {
            osDelay (500);
        }
}
