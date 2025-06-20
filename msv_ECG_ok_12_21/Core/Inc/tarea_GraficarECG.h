/*
 * tarea_GraficarECG.h
 *
 *  Created on: May 25, 2023
 *      Author: Ing. Alejandra Sakura Bautista Ambrocio
 * Description: Librería y fuente para la tarea de graficación de 2 señales de
 *              ECG (derivadas ECG). Incluye el filtrado de las mismas.
 *              Recibe 500 sps para cálculo de filtrado, pero envía 250sps para
 *              graficación.
 *
 * Lista de cambios:
 *   1 - Primer versión estable
 */

#ifndef INC_TAREA_GRAFICARECG_H_
#define INC_TAREA_GRAFICARECG_H_

#define LIB_VERSION_GRAFICARECG "1"
// Definiciones para los filtros ECG
//#define FILTRO_ECG_ORDEN 257 //Cerca de 1 segundo de retraso
#define FILTRO_ECG_ORDEN 129 //Cerca de 1/2 segundo de retraso
//#define FILTRO_ECG_ORDEN 65 //Cerca de 1/4 segundo de retraso
#define FILTRO_ECG_TAM_BLOQUE 1 //filtrar al vuelo

typedef enum
{
    DERIVACION_NULL,
    DERIVACION_I,
    DERIVACION_II,
    DERIVACION_III,
    DERIVACION_AVR,
    DERIVACION_AVL,
    DERIVACION_AVF,
    DERIVACION_V1,
    DERIVACION_V2,
    DERIVACION_V3,
    DERIVACION_V4,
    DERIVACION_V5,
    DERIVACION_V6,
    DERIVACION_RESPIRACION
} ecg_Derivaciones_t;

typedef struct
{
    ecg_Derivaciones_t g1;
    ecg_Derivaciones_t g2;
    uint8_t ecg_FiltroG1;
    uint8_t ecg_FiltroG2;
} ecg_Graficas_s;

typedef enum
{
    FILTRO_NULL = 0U,        // No filtrar la señal
    FILTRO_DIAGNOSTICO = 1U, // Filtrado 0.05 ~ 150Hz
    FILTRO_MONITOR = 2U,     // Filtrado 0.3 ~ 40Hz
    FILTRO_MAXIMO = 3U       // Filtrado 1 ~ 18 Hz

} filtroECG_t;


void tarea_GraficarECG_Setup(void);
void tarea_GraficarECG_Loop(void);


#endif /* INC_TAREA_GRAFICARECG_H_ */
