/*
 * tarea_Respiracion.h
 *
 *  Created on: May 25, 2023
 *      Author: Ing. Alejandra Sakura Bautista Ambrocio
 *              Ing. Gabriela Morales
 * Description: Librería y fuente para la tarea Respiración.
 *              Se acuerda con Gabriela y Víctor, que no hace falta un muestreo
 *              de alta velocidad, por lo que se cambia de 500sps a 100sps, es
 *              decir, en vez de ser invocada cada 2ms, será invocada cada 10ms.
 *
 *              La tarea LeerADS se encarga de manejar este tiempo.
 *
 *
 * Lista de cambios:
 *   1 - Primer versión estable
 */

#ifndef INC_TAREA_RESPIRACION_H_
#define INC_TAREA_RESPIRACION_H_

#define LIB_VERSION_RESPIRACION "1"

#define BUFF_MSG_AP 5   //W:1\n activación, W:0\n desactivación para alarma de Apnea
#define FILTRO_RESP_TAM_BLOQUE 1
//#define ENTRADA_RESP 	26	//32
#define FILTRO_NEONATO 	8		//22	// 50
#define FILTRO_PEDIAT	16
#define FILTRO_ADULTO	40
#define LIM_AR_RESP  	10
#define TOL_RPM      	20
#define PI_R    3.14159265358979323846

#define FILTRO_RESP_ORDEN 17	//	16+1

void tarea_Respiracion_Setup(void);
void tarea_Respiracion_Loop(void);


#endif /* INC_TAREA_RESPIRACION_H_ */
