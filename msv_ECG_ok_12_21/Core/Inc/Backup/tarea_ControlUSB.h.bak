/*
 * tarea_EnvioUSB.h
 *
 *  Created on: May 25, 2023
 *      Author: Ing. Alejandra Sakura Bautista Ambrocio
 * Description: Librería y fuente para la tarea Control USB.
 *              Esta tarea se encarga de enviar la información generada por
 *              todas las tareas a través del puerto USB.
 *              Esta tarea se invoca cada System Tick utilizando osDelay(1) que
 *              según documentación, puede ser menor a 1ms y máximo 1ms.
 *              Teniendo en cuenta que el envío más rápido es 250sps (2ms)
 *              permite que si alguna tarea terminó de calcular sus datos, estos
 *              puedan ser enviados antes de los 2ms, lo que permite a su vez
 *              que el siguiente mensaje sea más corto y por lo tanto se envíe
 *              más rápido.
 *
 *
 * Lista de cambios:
 *   1 - Primer versión estable
 */

#ifndef INC_TAREA_CONTROLUSB_H_
#define INC_TAREA_CONTROLUSB_H_

#define LIB_VERSION_CONTROLUSB "1"


/* Tamaño del buffer
Mensaje         | Peor Caso                                    | Tamaño
info            | I:STM,99,99,99,99,99,99,99,99,99,99,99\n     | 40
MarcPas         | M:\n                                         | 3
g1              | A:-9999.999\n                                | 12
g2              | B:-9999.999\n                                | 12
frec C          | F:999\n                                      | 6
g3              | C:-9999.999\n                                | 12
Resp            | R:-9999.999\n                                | 12
Apnea           | W:0\n                                        | 4
elect           | L:11111\n                                    | 8
Ping            | P\n                                          | 2
Final de cadena | NULL                                         | 1

total       112
margen        8



 * */

#define BUFF_MSG_TOT  256 //120


void tarea_EnvioUSB_Setup(void);
void tarea_EnvioUSB_Loop(void);
#endif /* INC_TAREA_CONTROLUSB_H_ */
