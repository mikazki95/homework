/*
 * tarea_RevisarElectrodos.h
 *
 *  Created on: May 25, 2023
 *      Author: Ing. Alejandra Sakura Bautista Ambrocio
 * Description: Librería y fuente para la tarea de Revisar Electrodos.
 *              Esta tarea no requiere ser invocada muy seguido: Al inicio
 *              tiene un llamado cada 500ms (esperando activar la lectura del
 *              ECG), posteriormente se llama cada 5 segundos.
 *
 *              Gandy me indica que no requiere exactamente qué electrodo se
 *              desconecta, por lo que simplifico la función a simplemente
 *              revisar el estado de los electrodos RA, LA, LL y V1.
 *              Al ser desconectado RL, aparecen como desconectados otros
 *              electrodos por lo que es detectado indirectamente.
 *
 * Lista de cambios:
 *   1 - Primer versión estable
 */

#ifndef INC_TAREA_REVISARELECTRODOS_H_
#define INC_TAREA_REVISARELECTRODOS_H_

#define LIB_VERSION_REVISARELECTRODOS "1"

void tarea_RevisarElectrodos_Setup(void);
void tarea_RevisarElectrodos_Loop(void);

#endif /* INC_TAREA_REVISARELECTRODOS_H_ */
