/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "FreeRTOS.h"
#include "task.h"

/* Programa de ejemplo tomado desde ipc_echo
 * En este programa espero recibir algo con RPMessage. Aún estoy en modo blocking
 * Lo recibido mandarlo por la UART1 que en linux sería /dev/ttyUSB2
 */


/* Características de la tarea principal.
 * Según la documentación, este stack tiene 32KiB, por lo que este se encarga de
 * generar lasdemás tareas con su propio stack y prioridades
 */

#define MAIN_TASK_PRI  (configMAX_PRIORITIES-1)                                 // La más alta prioridad
#define MAIN_TASK_SIZE (16384U/sizeof(configSTACK_DEPTH_TYPE))                  // Stack de la tarea principal


StackType_t  gMainTaskStack[MAIN_TASK_SIZE] __attribute__((aligned(32)));       // Tipo de stack: uint32_t
StaticTask_t gMainTaskObj;                                                      // Objeto de instancia
TaskHandle_t gMainTask;                                                         // Manejador de la tarea


void generarTareas(void *args);



void tarea0(void *args) //Función que se convertirá en tarea
{
    generarTareas(NULL);
    vTaskDelete(NULL);
}


int main(void)
{
    /* Inicialización de los módulos específicos del SOC */
    System_init();
    Board_init();

    /* Creación de la tarea */
    gMainTask = xTaskCreateStatic( tarea0,                                      // Función que implementa la tarea
                                  "Tarea0",                                     // Nombre de la tarea. Esto sólo facilita el Debug
                                  MAIN_TASK_SIZE,                               // Tamaño del Stack depth en unidades de StackType_t (típico en uint32_t)
                                  NULL,                                         // No usamos parámetros para la tarea
                                  MAIN_TASK_PRI,                                // Prioridad de la tarea, 0 es la menor, configMAX_PRIORITIES-1 es la mayor
                                  gMainTaskStack,                               // Puntero al stack base
                                  &gMainTaskObj );                              // Puntero al objeto de memoria de tarea estáticamente alojada
    configASSERT(gMainTask != NULL);

    /* Iniciar el planificador para iniciar la ejecución de las tareas. */
    vTaskStartScheduler();

    /* Nunca debería alcanzar la siguiente línea gracias a vTaskStartScheduler()
    Sólo retornará en caso de que no haya suficiente memoria para FreeRTOS disponible
    para crear la tarea Idle y tareas Timer (si hubiera).*/
    DebugP_assertNoLog(0);

    return 0;
}
