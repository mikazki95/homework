/*
 * tarea_RevisarComandos.h
 *
 *  Created on: May 25, 2023
 *      Author: Ing. Alejandra Sakura Bautista Ambrocio
 * Description: Librería y fuente para la tarea de Revisar Comandos USB.
 *              Esta tarea no requiere ser invocada muy seguido, pero debido a
 *              las configuraciones que envía el PC al encender, se acuerda con
 *              Gandy una revisión cada 50 ms.
  *
 * Lista de cambios:
 *   1 - Primer versión estable
 */

#ifndef INC_TAREA_REVISARCOMANDOS_H_
#define INC_TAREA_REVISARCOMANDOS_H_

#define LIB_VERSION_REVISARCOMANDOS "1"

#define CMD_INICIAR                  "S" // Comandos del ECG
#define CMD_DETENER                  "E"
#define CMD_INFO                     "I"
#define CMD_SEL_GRAFICAS             "G"
#define CMD_SEL_FILTRO               "F"
#define CMD_NEOPIXEL_PALTA_ROJO      "0"
#define CMD_NEOPIXEL_PMEDIA_AMARILLO "1"
#define CMD_NEOPIXEL_PBAJA_BLANCO    "2"
#define CMD_NEOPIXEL_PMEDIA_AZUL     "3"
#define CMD_NEOPIXEL_SOLIDO_ROJO     "4"
#define CMD_NEOPIXEL_SOLIDO_AMARILLO "5"
#define CMD_NEOPIXEL_SOLIDO_VERDE    "6"
#define CMD_NEOPIXEL_SOLIDO_NEGRO    "7"
#define CMD_NEOPIXEL_SOLIDO_BLANCO   "8"
#define CMD_APNEA_OFF                "B"
#define CMD_APNEA_ON                 "C"
#define CMD_TIPO_PACIENTE			 "T"
#define CMD_PING                     "P"
#define CMD_REP_LA                   "J" ////COMANDO PARA ELEJIR DERIVACION DE RESPIRACION
#define CMD_REP_LL                   "K"	////COMANDO PARA ELEJIR DERIVACION DE RESPIRACION


#define TIME_IDLE                    50

ecg_Graficas_s selectGraph (char *cadena);
void tarea_RevisarComandos_Setup(void);
void tarea_RevisarComandos_Loop(void);


#endif /* INC_TAREA_REVISARCOMANDOS_H_ */
