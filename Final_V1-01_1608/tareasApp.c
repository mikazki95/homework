 /*  Programa desarrollado para la detección de arritmias
 *
 *  Se recibe el dato normalizado por el STM32
 *  Se procesa la información para detectar el complejo QRS
 *  Se detecta el punto R
 *  Se mide el segmento RR
 *  Se saca el promedio del mismo para determinar el ritmo cardiaco.
 *  Se define un codigo para diferentes tipos de arritmias.
 *  29/05/2023  Se identifica bradicardia, taquicardia y el latido perdido
 *  30/05/2023  Se implementa la segunda tarea
 *  31/05/2023  Se inicia implementación de red neuronal
 *  02/06/2023  Se realiza la identificación de latidos correspondientes a arritmia
 *  07/06/2023  Se diferencian latido buenos y malos a partir del umbral de la integral.
 *  09/06/2023  Se identificó una falla en el envío de datos de ECG que está afectando la respuesta
 *      de la red neuronal
 *  13/06/2023  Se inicia implementación de tarea 3 para establecer protocolo de comunicación con la App
 *  20/06/2023  Se realiza la identificación de 9 arritmias a través de la red neuronal
 *  21/06/2023  Se integra señal de marcapasos en trama de datos ECG
 *  23/06/2023  Se realiza integración de arritmias de arritmias debidas al ritmo
 *  26/06/2023  Se implementa protocolo de comunicación para activación / desactivación de alarmas de arritmias
 *  Se establece número de versión: 1.00
 *  06/07/2023  Se implimentan arritmias de marcapasos
 *  11/07/2023  Se realizan ajustes en pesos de red y en detección de punto R
 *  12/07/2023  Se identifican 14 arritmias correcta y oportunamente, pendiente implementar en versión de envio de comandos a Raspberry
 *  13/07/2023  Se integra detección de arritmias con algoritmo de comunicación
 *  17/07/2023  Se agregan tramas de configuración para los parametros de los limites de las arritmias.
 *      Se establece envio de limpieza de cvp mientras se detecta ritmo normal.
 *  25/07/2023  Se corrigen algunos bug identificados en la detección de arritmias
 *  11/08/2023  Se modifica el criterio para detección de pulsos tipo marcapasos, lo que afectaba la detección del resto de las arritmias.
 *  15/08/2023  Se modifica detercción de cvp en los primeros latidos detectados.
 *  16/08/2023  Se modifica condición de entrenamiento, a su ejecución solo con comando.
 *
 *  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>              // Para acceder al API IPC RPMessage
#include <kernel/dpl/TaskP.h>               // para acceder a las tareas
#include <drivers/ipc_notify.h>
#include <drivers/ipc_rpmsg.h>
#include "ti_drivers_config.h"             // Para acceder al API UART
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "Definiciones.h"

//  ------------------ DEFINICIÓN DE LAS TAREAS  -------------------

/* Este es el servicio usado para el test de eco en el espacio de usuario */
#define IPC_RPMESSAGE_SERVICE_CHRDEV      "rpmsg_chrdev"                        // Nombre para que linux genere el remoteproc adecuado
#define IPC_RPMESSAGE_ENDPT_NUMBER        (14U)                                 // Dirección del EndPoint
#define IPC_RPMESSAGE_MAX_MSG_SIZE        (96u)                                 // Tamaño máximo que un mensaje puede tener en este ejemplo

/* Los objetos RPMessage DEBEN SER globales*/
RPMessage_Object gIpcRecvMsgObject;                                             // Objeto RPMessage usado para recibir y enviar mensajes

/* Prioridad de la tarea, stack, tamaño del stack y objetos de tareas, estos DEBEN SER globales */
#define IPC_RPMESSAFE_TASK_PRI            (8U)
#define IPC_RPMESSAFE_TASK_STACK_SIZE     (8*1024U)                             // 8KiB
uint8_t gIpcTaskStack[IPC_RPMESSAFE_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gIpcTask;

/* Task priority, stack, stack size and task objects, these MUST be global's */
#define RED_NEURO_TASK_PRI         (8U)
#define RED_NEURO_TASK_STACK_SIZE (512*1024U)
uint8_t gRedNeuroStack[RED_NEURO_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gRedNeuroTask;

/* Task priority, stack, stack size and task objects, these MUST be global's */
#define TX_RX_APP_TASK_PRI         (8U)
#define TX_RX_APP_TASK_STACK_SIZE  (8*1024U)
uint8_t gTxRxAppStack[TX_RX_APP_TASK_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gTxRxAppTask;

/* Application specific task arguments */
typedef struct {
    uint32_t value;
} RedNeuro_Args;

RedNeuro_Args gRedNeuro_args;

typedef struct {
    uint32_t value_App;
} TxRxApp_Args;

TxRxApp_Args gTxRxApp_args;


/*  Funciones principales      */
void generarTareas(void *args);
void tarea1(void *args);
void tarea2(void *args);
void tarea3(void *args);

void generarTareas(void *args) /* función main */
{
    //Levantar los servicios que usarán las tareas
    int32_t status;
    int32_t statusT3;

    Drivers_open();
    Board_driversOpen();

    /* Esta API DEBE SER llamada por las aplicaciones para comunicarse con linux */
    status = RPMessage_waitForLinuxReady(SystemP_WAIT_FOREVER);
    DebugP_assert(status==SystemP_SUCCESS);

    /* Iniciar el servicio RPMessage con el objeto RPMessage_Object. Este servicio se encargará de la recepción y envío de los mensajes IPC */
    RPMessage_CreateParams createParams;
    RPMessage_CreateParams_init(&createParams);                                 // Valores por default. Método recomendado por la API
    createParams.localEndPt = IPC_RPMESSAGE_ENDPT_NUMBER;                       // Cambiar el valor por default al que queremos
    status = RPMessage_construct(&gIpcRecvMsgObject, &createParams);            // Servicio RPMsg listo
    DebugP_assert(status==SystemP_SUCCESS);

    /* Anunciar el EndPoint al cliente linux. Linux no sabe que el servicio existe hasta que lo anunciamos. */
    status = RPMessage_announce(CSL_CORE_ID_A53SS0_0,                           // Núcleo corriendo linux
                                IPC_RPMESSAGE_ENDPT_NUMBER,                     // LocalEndPt
                                IPC_RPMESSAGE_SERVICE_CHRDEV);                  // Nombre       rpmsg_chrdev
    DebugP_assert(status==SystemP_SUCCESS);

    /* Crear la tarea que manejará el servicio de ping */
    TaskP_Params taskParams1;                                                    // Parámetros para generar la tarea
    TaskP_Params_init(&taskParams1);                                             // Iniciar parámetros. Recomendado por la API
    taskParams1.name = "TAREA_01";                                               // Nombre para Debug
    taskParams1.stackSize = IPC_RPMESSAFE_TASK_STACK_SIZE;                       // 8KiB
    taskParams1.stack = gIpcTaskStack;                                           // Puntero al stack (*uint8_t)
    taskParams1.priority = IPC_RPMESSAFE_TASK_PRI;                               // Prioridad de la tarea
    taskParams1.args = &gIpcRecvMsgObject;                                       // Acá estamos pasando el RPMessage_Object como argumento de la función
    taskParams1.taskMain = tarea1;                                               // Función que implementa la tarea

    /* Construir la tarea */
    status = TaskP_construct(&gIpcTask, &taskParams1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Crear la tarea que manejará el servicio de ping */
    TaskP_Params taskParams2;                                                // Parámetros para generar la tarea
    TaskP_Params_init(&taskParams2);                                         // Iniciar parámetros. Recomendado por la API
    taskParams2.name = "TAREA_02";                                           // Nombre para Debug
    taskParams2.stackSize = RED_NEURO_TASK_STACK_SIZE;                       // 8KiB
    taskParams2.stack = gRedNeuroStack;                                      // Puntero al stack (*uint8_t)
    taskParams2.priority = RED_NEURO_TASK_PRI;                               // Prioridad de la tarea
    taskParams2.args = &gRedNeuro_args;                                   // Acá estamos pasando el RPMessage_Object como argumento de la función
    taskParams2.taskMain = tarea2;                                          // Función que implementa la tarea

    /* Construir la tarea */
    status = TaskP_construct(&gRedNeuroTask, &taskParams2);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Tarea cuya función es establecer la comunicación con la App */
    TaskP_Params taskParams3;                                           // Parámetros para generar la tarea
    TaskP_Params_init(&taskParams3);                                    // Iniciar parámetros. Recomendado por la API
    taskParams3.name = "TAREA_03";                                      // Nombre para Debug
    taskParams3.stackSize = TX_RX_APP_TASK_STACK_SIZE;                  // 8KiB
    taskParams3.stack = gTxRxAppStack;                                  // Puntero al stack (*uint8_t)
    taskParams3.priority = TX_RX_APP_TASK_PRI;                          // Prioridad de la tarea
    taskParams3.args = &gTxRxApp_args;                                  // Acá estamos pasando el RPMessage_Object como argumento de la función
    taskParams3.taskMain = tarea3;                                      // Función que implementa la tarea

    /* Construir la tarea */
    statusT3 = TaskP_construct(&gTxRxAppTask, &taskParams3);
    DebugP_assert(statusT3 == SystemP_SUCCESS);

    /* Esperar que todos los núcleos No-Linuxs estén listos. Esto asegura que cuando
     * Enviemos los mensajes, ellos no se pierdan debido a que el EndPoint
     * no se haya creado en el núcleo remoto.
     */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);


    /* Salida de la tarea. vTaskDelete() se llama desde fuera de la función, así que simplemente acaba */

    //Board_driversClose();
    /* No cerramos los drivers porque otros procesos quedan en segundo plano */
    /* Drivers_close(); */
}

void tarea1(void *args)
{
    /* Tarea que recibe datos normalizados del STM32 y los procesa. */

    char    recvMsg[IPC_RPMESSAGE_MAX_MSG_SIZE+1];                  // Buffer para recibir mensajes, +1 para caracter NULL en el peor caso
    uint16_t recvMsgSize, remoteCoreId, remoteCoreEndPt;            // Características del mensaje: tamaño, ProcID, EndPoint
    RPMessage_Object *pRpmsgObj = (RPMessage_Object *)args;         // Adquirir el RPMessage_Object (que era global) por medio de los argumentos de la función

    int32_t     transferOK,status;
    UART_Transaction trans;

    uint8_t     gUartBuffer[APP_UART_BUFSIZE];
    char        gUartReceiveBuffer[APP_UART_RECEIVE_BUFSIZE] = {0};
    char        bufferdatos_ECG[APP_UART_RECEIVE_BUFSIZE];


    #define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
            do { \
                if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
                { \
                    DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
            } \
    } while(0) \


    bool    flag_dato_1 = false;
    bool    flag_aux_delay = false;
    bool    flag_aviso_learn = false;
    int     i=0,k=0;    //,cta_envio=0;
    float   suma_fs = 0;
    int8_t  valor_ecg_act = 0;
    int8_t  ecg_ant = 0,flanco_ecg = 0;
    int32_t aux_raw_d1 = 0;
    int32_t aux_raw_d2 = 0;

    status = SemaphoreP_constructBinary(&gUartWriteDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gUartReadDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    UART_Transaction_init(&trans);                                              // Inicializar la estructura UART_Transaction

    flag_continuar = true;
    flag_start_ready = true;

    while(1)
    {
        /* Establecer 'recvMsgSize' al tamaño del buffer de recepción, después
         * se devolverá 'recvMsgSize' con el tamaño actual de los datos válidos
         * recibidos */
        recvMsgSize = IPC_RPMESSAGE_MAX_MSG_SIZE;
        status = RPMessage_recv(pRpmsgObj,                                      // RPMessage_Object * obj
                                recvMsg,                                        // void *             data
                                &recvMsgSize,                                   // uint16_t *         dataLen
                                &remoteCoreId,                                  // uint16_t *         rmeoteCoreId
                                &remoteCoreEndPt,                               // uint16_t *         remoteEndPt
                                SystemP_WAIT_FOREVER);                          // uint32_t           timeout
        DebugP_assert(status==SystemP_SUCCESS);
        recvMsg[recvMsgSize] = 0;                                              // Caracter NULL al final del mensaje para el envío de cadenas


        do {

            /* Lectura de 10 caracteres enviados por STM32 */
            gNumBytesRead = 0U;
            trans.buf   = &gUartReceiveBuffer[0U];
            trans.count = APP_UART_RECEIVE_BUFSIZE;
            transferOK = UART_read(gUartHandle[CONFIG_UART1], &trans);
            APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

            /* Wait for read completion */
            SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
            DebugP_assert(gNumBytesRead == APP_UART_RECEIVE_BUFSIZE);

            //calculo de periodo de muestreo
            if (flag_define_fs)
            {
                tiempo_fs0 = ClockP_getTimeUsec();
                if (inicio_muestreo == 0)
                    delta_fs = 4;
                else
                    delta_fs = (float) (tiempo_fs0 - tiempo_fs1)/1000;

                tiempo_fs1 = tiempo_fs0;
                inicio_muestreo++;
                suma_fs += delta_fs;

                if (inicio_muestreo == 10)
                {
                    cont_latido = 0;
                    flag_learning_1 = true;
                }
                if (inicio_muestreo > 50)
                {
                    media_fs = suma_fs / 49;
                    flag_define_fs = false;
                    inicio_muestreo = 0;
                    suma_fs = 0;
                    flag_inicio_datos = true;
                    flag_trained = true;
                    cont_init_frec = 0;
                    int i0 = 0;
                    for (i0 = 0; i0 < 20; i0++)
                    {
                        dato_frec[i0][0]=0;
                        dato_frec[i0][1]=0;
                    }
                }
            }

            for (i = 0; i <= APP_UART_RECEIVE_BUFSIZE; i++) // Validaciòn de inicio de cadena
            {
                if ((gUartReceiveBuffer[i] == (char)valida_d1) && (flag_initTraining))
                {
                        flag_dato_1 = true;
                        flag_dato_rdy = true;
                        k = i;
                        i = APP_UART_RECEIVE_BUFSIZE + 1;
                }
            }

            if (flag_dato_1)
            {
                for (i = 0; i < BUFSIZE_ECG; i++)
                {
                    bufferdatos_ECG[i] = gUartReceiveBuffer[k+i];
                    if ((i > 0)&& (i < 5))
                        dato_in_d1[i-1] = (uint8_t)bufferdatos_ECG[i];
                    else if (i > 5)
                        dato_in_d2[i-6] = (uint8_t)bufferdatos_ECG[i];
                }
                pulso_marcapasos = (uint8_t) gUartReceiveBuffer[k + BUFSIZE_ECG +1];        // señal de marcapasos

                if (pulso_marcapasos > 100)
                    pulso_marcapasos = 0;
                else if ((pulso_marcapasos == valido_pace) && ((flag_info_arr_13 == 1) || (flag_info_arr_14 == 1)))   // cambiar 85 = 0X55
                {
                    flag_detec_mkpasos = true;
                    cta_mkpasos = 0;
                    flag_tipo_pulso = true;
                    cuenta_pulso_mk = 0;
                }

                if ((dato_in_d1[0] == 0x00)||(dato_in_d1[0] == 0xFF))
                {
                    aux_raw_d1 = ((dato_in_d1[0]<<24)|(dato_in_d1[1]<<16)|(dato_in_d1[2]<<8)|(dato_in_d1[3]));
                    aux_raw_d2 = ((dato_in_d2[0]<<24)|(dato_in_d2[1]<<16)|(dato_in_d2[2]<<8)|(dato_in_d2[3]));
                    if (aux_raw_d1 < 1500)
                        dato_raw_d1 = aux_raw_d1;
                    if (aux_raw_d2 < 1500)
                        dato_raw_d2 = aux_raw_d2;
                }

                flag_dato_1 = false;
            }

            /* Eco en TX de caracteres recibidos (Eliminar al finalizar)  */
            gNumBytesWritten = 0U;
            trans.buf   = &bufferdatos_ECG[0U];
            trans.count = BUFSIZE_ECG;
            transferOK = UART_write(gUartHandle[CONFIG_UART1], &trans);
            APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

            // Wait for write completion
            SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
            DebugP_assert(gNumBytesWritten == BUFSIZE_ECG);     // */

            valor_ecg_act = 1;
            if (dato_raw_d1 < -150)       // cambio 27/07/2023
                valor_ecg_act = 0;
            flanco_ecg = valor_ecg_act - ecg_ant;
            ecg_ant = valor_ecg_act;

            if (flanco_ecg == 1)   //flanco de subida
            {
                cuenta_edge++;
                if (cuenta_edge >= 150)
                    cuenta_edge = 150;
            }


            //**** Detección QRS  ***

            ecg_filt_ind1 = FiltroPasaBanda(dato_raw_d1);
            ecg_filt_ind2 = FiltroPasaBanda(dato_raw_d2);
            deriv_ecg_d1 = Derivativa(ecg_filt_ind1);
            deriv_ecg_d2 = Derivativa(ecg_filt_ind2);
            integral_ecg_d1 = MovingWindowIntegral(deriv_ecg_d1);
            integral_ecg_d2 = MovingWindowIntegral(deriv_ecg_d2);

            EncontrarPuntoR(integral_ecg_d1);

            derivada_raw_d2 = DerivadaFuncion(dato_raw_d2);

            if (flag_trained)
            {
                if (flag_aviso_learn)
                {
                    gNumBytesWritten_1 = 0U;
                    trans.buf   = &gUartBuffer[0U];
                    trans.count = sprintf(trans.buf, "L\n");
                    transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
                    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

                    //   Wait for write completion
                    SemaphoreP_pend(&gUartWriteDoneSem_1, SystemP_WAIT_FOREVER);
                    DebugP_assert(gNumBytesWritten_1 == strlen(trans.buf));       //   */
                    flag_aviso_learn = false;
                }

            }
            else
            {
                if ((flag_aux_delay) && (flag_info_arr_06 == 1))
                {
                    if (flag_cvp == 1)
                    {
                        gNumBytesWritten_1 = 0U;
                        trans.buf   = &gUartBuffer[0U];
                        trans.count = sprintf(trans.buf, "B:06,%i\n",cuenta_cvp);       //,punto_heartrate
                     //   trans.count = sprintf(trans.buf, "t:%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\n",tipo_arritmia,flag_info_arr_01,flag_info_arr_02,flag_info_arr_03,flag_info_arr_04,
                       //                       flag_info_arr_05,flag_info_arr_07,flag_info_arr_08,flag_info_arr_09,flag_info_arr_10,flag_info_arr_11);
                        transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
                        APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

                        //   Wait for write completion
                        SemaphoreP_pend(&gUartWriteDoneSem_1, SystemP_WAIT_FOREVER);
                        DebugP_assert(gNumBytesWritten_1 == strlen(trans.buf));       //   */
                    }
                    else
                    {
                        if (flag_beat_ok)       // cambio de estado en cuenta CVP's
                        {
                            gNumBytesWritten_1 = 0U;
                            trans.buf   = &gUartBuffer[0U];
                            if (tipo_arritmia < 10)
                                trans.count = sprintf(trans.buf, "B:0%i,%i\n",tipo_arritmia,cuenta_cvp);
                            else
                                trans.count = sprintf(trans.buf, "B:%i,%i\n",tipo_arritmia,cuenta_cvp);
                         //   trans.count = sprintf(trans.buf, "t:%i,%i,%i,%i,%i\n",tipo_arritmia,cont_ritmo_irregular,time_RR_actual,(int)Thr_RR_miss,variable_aux);

                            transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
                            APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

                            //   Wait for write completion
                            SemaphoreP_pend(&gUartWriteDoneSem_1, SystemP_WAIT_FOREVER);
                            DebugP_assert(gNumBytesWritten_1 == strlen(trans.buf));       //   */
                            flag_beat_ok = false;
                        }
                    }
                    flag_aux_delay = false;

                }
            }


      /*      if (flag_training_ready)
            {
                gNumBytesWritten_1 = 0U;
                trans.buf   = &gUartBuffer[0U];
                trans.count = sprintf(trans.buf, "RL: %i\n",time_RR_valido);
                transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
                APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

                //   Wait for write completion
                SemaphoreP_pend(&gUartWriteDoneSem_1, SystemP_WAIT_FOREVER);
                DebugP_assert(gNumBytesWritten_1 == strlen(trans.buf));
                flag_training_ready = false;
            }  //  */

            if (flag_punto_R)     // si se detecto punto R
            {
                ProcesamientoSegmentoRR();
                DefinicionUmbrales(valor_punto_R);
                flag_punto_R = false;
                cuenta_edge = 0;
                flag_aux_delay = true;
                flag_aviso_learn = true;
            }

            point_in[0] = contador_muestras;
            point_in[1] = integral_ecg_d1 >> 5;


           /**** envío de datos a la terminal **/

      /*      gNumBytesWritten = 0U;
            trans.buf   = &gUartBuffer[0U];
            trans.count = sprintf(trans.buf,"d:%i,%i,%i,%i,%i,%i,\r\n",dato_raw_d1,pulso_marcapasos,integral_ecg_d1,
                                  flag_cvp,tipo_arritmia,contador_muestras);    //variable_aux);   // contador_muestras,flag_cvp,(int)flag_heartbeat
            transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
            APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

            //   Wait for write completion
            SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER);
            DebugP_assert(gNumBytesWritten == strlen(trans.buf));       //   */

            if (flag_detecta)
            {

                gNumBytesWritten_1 = 0U;
                trans.buf   = &gUartBuffer[0U];
                if (tipo_arritmia < 10)
                    trans.count = sprintf(trans.buf, "B:0%i\n",tipo_arritmia);
                else
                    trans.count = sprintf(trans.buf, "B:%i\n",tipo_arritmia);
                transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
                APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

                      //   Wait for write completion
                SemaphoreP_pend(&gUartWriteDoneSem_1, SystemP_WAIT_FOREVER);
                DebugP_assert(gNumBytesWritten_1 == strlen(trans.buf));       //   */
                flag_detecta = false;
            }

            SemaphoreP_destruct(&gUartWriteDoneSem);
            SemaphoreP_destruct(&gUartReadDoneSem);


        }    while (flag_continuar);

    }
            /* Tarea permanente */
}

void tarea2(void *args)
{
    tipo_arritmia = NORMAL;
    int array_cvp [250][2] = {0};
    int cont_0 = 0;
    int cont_1 = 0;
    int cont_beat = 0;
    int promedio_hr = 0;
    int valor_hr_k_1 = 0,valor_hr_k_0 = 0;
    int tipo_ant = 0;

    int cont_hr =0;
    int cuenta_cvp_ant = 0;
    int tiempo_asistole =  limite_asistole * 250;   // numero de muestras
    int conta_fib = 0;
    int tiempo_fibr = 1000;

    uint16_t cont_raw_cero = 0;

    bool flag_vent_tach = false;
    bool flag_desborde = false;
    bool flag_asistole = false;
    bool flag_no_pulso = false;
    bool flag_normal_beat = true;
    bool flag_conta_fib = false;
    bool flag_arrit_pnp = false;
    bool flag_arrit_pnc = false;

    float  lim_TachVent = 15000/limite_TachVent;
    int    lim_TachVent_int = (int)lim_TachVent;
    int derivada_d2_ant = 0;

    while (1)
    {
        if (flag_dato_rdy)
        {
            flag_dato_rdy = false;
            if (flag_trained)
            {
                if (!flag_data_aux)
                {
                    data_aux[cont_task_3] = point_in[1];
                    cont_task_3++;
                    if (cont_task_3 > 399)
                        cont_task_3 = 0;
                }
                else
                {
                    flag_data_aux = false;
                    if (flag_heartbeat)
                    {
                        if (cont_init_frec < 20)
                        {
                            dato_frec[cont_init_frec][0] = cont_task_3;
                            dato_frec[cont_init_frec][1] = point_in[1];
                            cont_init_frec++;
                        }
                        flag_heartbeat = false;

                        int k;
                        for (k = 0; k <= cont_task_3; k++)
                        {
                            data_pre[contador_tarea_2][0] = k;
                            data_pre[contador_tarea_2][1] = data_aux[k];

                            contador_tarea_2++;
                            if(contador_tarea_2 >= NUM_POINTS)
                            {
                                num_clusters_1 = dbscan(data_pre, eps, minPts, clusters, len_clusters, no_clusters, &len_no_clusters,NUM_POINTS);
                                dbscan_2(dato_frec, eps_3, minPts_3, clusters_3, len_clusters_3, no_clusters_3, &len_no_clusters_3,NUM_POINTS_3);
                                contador_tarea_2 = 0;
                                flag_trained = false;
                                k = cont_task_3+1;
                                flag_training_ready = true;
                            }
                        }
                        for (k = 0; k< 250; k++)
                        {
                            array_cvp [k][0] = 0;
                            array_cvp [k][1] = 0;
                        }
                        cont_ok = 0;
                        cuenta_cvp = 0;
                        index_p_arritmia = 0;
                        flag_RenT_RV = false;
                        flag_vent_tach = false;
                        tipo_arritmia = NORMAL;  //0;
                        flag_asistole = false;
                        flag_no_pulso = false;
                        flag_normal_beat = true;
                        cont_InitBeat = 0;
                    }
                    cont_task_3=0;
                }
            }
            else        // cuando el entrenamiento ha terminado
            {
                if ((flag_info_arr_13 == 1) || (flag_info_arr_14 == 1))
                    flag_aviso_mk = true;
                else
                    flag_aviso_mk = false;

                if ((flag_aviso_mk) && (flag_cvp == 3))
                    cuenta_pulso_mk++;

                if (cuenta_pulso_mk > tiempo_asistole)
                    cuenta_pulso_mk = tiempo_asistole;

                int point_to_search[2];
                point_to_search[0] = point_in[0];
                point_to_search[1] = point_in[1];

                if (contador_muestras > tiempo_asistole)        // periodo en el que se desborda el contador, limite de tiempo para asistole
                {
                    flag_asistole = true;
                    flag_no_pulso = true;
                    contador_muestras = 0;
                    flag_cvp = 2;
                    flag_normal_beat = false;
                    cont_ok = 0;
                }
                if (flag_no_pulso)  // si se ha debordado el contador
                {
                    flag_no_pulso = false;
                    if (((cuenta_edge < 3) || (cont_raw_cero > 100)) && (!flag_conta_fib))
                    {
                        if (flag_info_arr_01 == 1)
                        {
                            tipo_arritmia = ASYSTOLE;             //asistole
                            cont_ok = 7;
                        }

                    }
                }

                conta_fib++;
                if (cuenta_edge > 2)
                    flag_conta_fib = true;
                else
                {
                    if (contador_muestras == 0)
                    {
                        conta_fib = 0;
                    }
                }

                if ((conta_fib > tiempo_fibr) && (flag_conta_fib))
                {
                    flag_cvp = 2;
                    if (flag_info_arr_02 == 1)
                    {
                        tipo_arritmia = VENT_FIBR;           // fibrilación ventricular
                        cont_ok = 6;
                    }
                    conta_fib = tiempo_fibr;
                }

                if (dato_raw_d2 == 0)
                {
                    cont_raw_cero++;
                    if (cont_raw_cero >= 376)
                    {
                        cuenta_edge = 0;
                        cont_raw_cero = 376;
                    }
                }
                else
                    cont_raw_cero = 0;



                if (flag_asistole)
                {
                    punto_heartrate = tiempo_asistole;
                    flag_desborde = true;
                    flag_asistole = false;
                }

                pulso[contador_muestras][0]= point_in[0];    //point_to_search[0];
                pulso[contador_muestras][1]= point_in[1];   //point_to_search[1];   */

                /////////////////  Modificacion 10/08/2023 para marcapasos

                if ((derivada_raw_d2 < -50) && (dato_raw_d2 < 0))
                {
                    flag_cvp_mk = true;
                }

                if (derivada_raw_d2 > 70)
                {
                    flag_cvp_mk = false;
                    flag_detec_mkpasos = false;
                }
                if ((flag_cvp_mk) && (derivada_raw_d2 > 10))
                {
                    if (derivada_raw_d2 < (derivada_d2_ant + 15))
                    {
                        flag_tipo_pulso = true;
                    }
                }

                derivada_d2_ant = derivada_raw_d2;

                if (contador_muestras == 40)
                {
                    flag_cvp_mk = false;
                }

                 ///// si se detecta punto Q

                if (flag_punto_Q)
                {

                    cont_InitBeat++;
                    if (cont_InitBeat > 32700)
                        cont_InitBeat = 4;

                    point_to_search[0] = punto_heartrate;
                    pulso[punto_heartrate][0]= point_to_search[0];

                    flag_punto_Q = false;

                    if (flag_desborde)
                    {
                        if (punto_heartrate < 375)
                            cont_hr++;
                        else
                            cont_hr = 0;
                        if (cont_hr >= 5)
                        {
                            cont_hr = 0;
                            flag_desborde = false;
                        }
                        flag_cvp = 2;
                        index_p_arritmia = 0;
                    }
                    else
                    {
                        flag_cvp = find_arritmia(pulso,clusters,len_clusters,num_clusters_1,punto_heartrate);

                        if (flag_tipo_pulso)
                        {
                            flag_cvp = 3;
                        }
                        flag_tipo_pulso = false;

                        if (cont_InitBeat < 3)
                            flag_cvp = 0;

                    }

                    if (flag_cvp < 2)
                        array_cvp[cont_0][0]= flag_cvp;
                    else
                        array_cvp[cont_0][0]= 0;

                    array_cvp[cont_0][1]= punto_heartrate;
                    valor_hr_k_1 = valor_hr_k_0;
                    valor_hr_k_0 = punto_heartrate;
                    cont_0++;

                    cont_1 += punto_heartrate;

                    if ((cont_1 > 15000) || (cont_0 >= 250))
                    {
                        cont_0 = 0;
                        cont_1 = 1;
                    }
                    int cont_3 = 0;
                    int x_a = 0;

                    promedio_hr = 0;
                    cuenta_cvp = 0;
                    for (x_a = 0; x_a < 250; x_a++)
                    {
                        if ((flag_cvp == 2) || (flag_detec_mkpasos))    //(flag_cvp == 3) ||
                            array_cvp[x_a][0] = 0;

                        cuenta_cvp += array_cvp[x_a][0];        // ******** CVP/min
                        cont_3 += array_cvp[x_a][1];

                        if (cont_3 > 15000)
                            break;
                    }
                    promedio_hr = time_RR_valido/4;

                    if (!flag_arritmia_sinus)
                    {
                        if (flag_cvp == 1)
                        {
                            flag_arritmia = true;
                            flag_normal_beat = false;
                            flag_beat_ok = false;
                            cont_beat = 0;
                        }
                        else if (flag_cvp == 0)
                        {
                            cont_beat++;
                            if (cont_beat > 3)
                            {
                                if (flag_beat_miss)
                                {
                                    if (flag_info_arr_11 == 1)
                                        tipo_arritmia = MISS_BEAT;

                                    flag_beat_miss = false;
                                    cont_ok = 0;
                                }
                                cont_beat = 0;
                            }
                        }
                    }
                    else
                    {
                        cont_beat++;
                        if (cont_beat > 3)
                        {
                            if (flag_bradicardia)
                            {
                                if (flag_cvp == 0)
                                {
                                    if (flag_info_arr_05 == 1)
                                        tipo_arritmia = BRADY;
                                }
                                else
                                    flag_arritmia_sinus = false;

                                flag_bradicardia = false;
                            }
                            if (flag_taquicardia)
                            {
                                if (flag_info_arr_04 == 1)
                                    tipo_arritmia = TACHY;

                                flag_taquicardia = false;
                            }
                            cont_beat = 0;
                        }
                    }

                    if (flag_arritmia)
                    {
                        pulsos_arritmia[index_p_arritmia][0] = flag_cvp;
                        pulsos_arritmia[index_p_arritmia][1] = punto_heartrate;
                        index_p_arritmia++;
                        fin_array += punto_heartrate;

                        if (index_p_arritmia == 1)
                        {
                            if (valor_hr_k_1 < ((promedio_hr / 3) + 13))
                            {
                                if (valor_hr_k_0 > (promedio_hr * 5/4))
                                {
                                    fin_array = 0;
                                    index_p_arritmia = 0;
                                    flag_arritmia = false;
                                    if (flag_info_arr_12 == 1)
                                        tipo_arritmia = R_EN_T;

                                    cont_ok = 0;
                                    flag_RenT_RV = false;
                                }
                                else if (valor_hr_k_0 < promedio_hr/3)
                                    flag_RenT_RV = true;
                            }
                        }   // fin index_p_arritmia =1
                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        if (index_p_arritmia == 2)
                        {
                            if ((valor_hr_k_0 > (promedio_hr * 5/4)) && (flag_RenT_RV))
                            {
                                fin_array = 0;
                                index_p_arritmia = 0;
                                flag_arritmia = false;
                                if (flag_info_arr_12 == 1)
                                    tipo_arritmia = R_EN_T;

                                cont_ok = 0;
                                flag_RenT_RV = false;
                            }
                        }

                        if (index_p_arritmia == 6)       // PAIR & TV > 2
                        {
                            int k_a = 0;
                            int flag_a = 0;
                            bool flag_evaluar = false;
                            bool flag_taq = true;

                            if (punto_heartrate > lim_TachVent_int)
                                flag_taq = false;

                            for(k_a = 0;k_a <= 6;k_a++)
                            {
                                flag_a += pulsos_arritmia[k_a][0];

                                if (flag_a < k_a)
                                    flag_evaluar = true;

                                if ((flag_evaluar)&&(pulsos_arritmia[k_a][0]==1))
                                    break;

                                if(k_a==6)
                                {
                                    if ((flag_evaluar) && (pulsos_arritmia[k_a][1] < (promedio_hr+13))
                                            && (pulsos_arritmia[k_a][1] > (promedio_hr-13)) && (!flag_vent_tach))
                                    {
                                        if (flag_a == 2)
                                        {
                                            if (flag_info_arr_07 == 1)
                                                tipo_arritmia = PAIRED_CVP;

                                        }
                                        else if (flag_a > 2)
                                        {
                                            if (flag_info_arr_08 == 1)
                                                tipo_arritmia = TV_2;

                                        }

                                        fin_array = 0;
                                        index_p_arritmia = 0;
                                        flag_arritmia = false;
                                        cont_ok = 0;
                                        flag_RenT_RV = false;
                                    }
                                    else
                                    {
                                        if (flag_taq)
                                        {
                                            fin_array = 0;
                                            index_p_arritmia = 0;
                                            flag_arritmia = false;
                                            flag_vent_tach = true;
                                            if (flag_info_arr_03 == 1)
                                                tipo_arritmia = VENT_TACH;

                                            cont_ok = 0;
                                            flag_RenT_RV = false;
                                            flag_normal_beat = false;
                                        }
                                    }
                                }
                            }
                        } // fin index_p_arritmia =6
                        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        if (index_p_arritmia == 8)
                        {
                            int k_a = 0;
                            int flag_a;

                            for(k_a = 0;k_a <= 7;k_a++)
                            {
                                flag_a = (k_a) & 0x0001;
                                if (flag_a == pulsos_arritmia[k_a][0])
                                    break;

                                if (k_a == 7)
                                {
                                    fin_array = 0;
                                    index_p_arritmia = 0;
                                    flag_arritmia = false;
                                    if (flag_info_arr_09 == 1)
                                        tipo_arritmia = BIGEMINY;

                                    cont_ok = 0;
                                    flag_RenT_RV = false;
                                }
                            }
                        }
                        if (index_p_arritmia == 10 )      // trigeminy
                        {
                            int k_a = 0;
                            int flag_a;

                            for(k_a = 0;k_a <= 9;k_a++)
                            {
                                flag_a = (k_a)%3;
                                if(flag_a == 0)
                                {
                                    if ((pulsos_arritmia[k_a][0]) == 0)
                                        break;
                                }
                                else
                                {
                                    if ((pulsos_arritmia[k_a][0]) == 1)
                                        break;
                                }
                                if(k_a == 9)
                                {
                                    fin_array = 0;
                                    index_p_arritmia = 0;
                                    flag_arritmia = false;
                                    if (flag_info_arr_10 == 1)
                                        tipo_arritmia = TRIGEMINY;

                                    cont_ok = 0;
                                    flag_RenT_RV = false;
                                }
                            }
                        }
                        // *****************  finalización de búsqueda de patrones válidos de arritmias
                        if (index_p_arritmia >=  12)
                        {
                            fin_array = 0;
                            index_p_arritmia = 0;
                            flag_arritmia = false;
                            //   tipo_arritmia = CVP_pMIN;
                            cont_ok = 0;
                            flag_RenT_RV = false;
                        }

                        if (fin_array > 7500)
                        {
                            fin_array = 0;
                            index_p_arritmia = 0;
                            flag_arritmia = false;
                            //         tipo_arritmia = CVP_pMIN;
                            cont_ok = 0;
                            flag_RenT_RV = false;
                        }
                    }       // fin flag_arritmia
                    else
                    {
                        if ((flag_cvp == 0) && (!flag_arritmia_sinus))
                            cont_ok++;

                        if (cont_ok >= 8)
                        {
                            cont_ok = 0;
                            index_p_arritmia = 0;
                            flag_RenT_RV = false;
                            flag_vent_tach = false;
                            tipo_arritmia = NORMAL;  //0;
                            flag_asistole = false;
                            flag_no_pulso = false;
                            flag_conta_fib = false;
                            flag_normal_beat = true;
                            cta_mkpasos = 0;
                            flag_arrit_pnp = false;
                        }

                        if (flag_normal_beat)
                        {
                            if (cuenta_cvp_ant != cuenta_cvp)
                                flag_beat_ok = true;

                            cuenta_cvp_ant = cuenta_cvp;
                        }
                    }

                    if ((!flag_arrit_pnc) && (!flag_arrit_pnp) && (flag_cvp == 3) && (!flag_arritmia) && (!flag_arritmia_sinus))
                        tipo_arritmia = NORMAL;

                    if ((flag_info_arr_13 == 1) && (flag_arrit_pnp) && (flag_cvp == 3))
                    {
                        tipo_arritmia = PNP;
                        flag_arrit_pnp = false;
                    }

                    if ((flag_info_arr_14 == 1) && (flag_arrit_pnc) && (flag_cvp == 3))
                    {
                        tipo_arritmia = PNC;
                        flag_arrit_pnc = false;
                    }

                }       // fin flag_punto Q

                //////// Arrirmias de marcapasos
                if ((cuenta_pulso_mk > (promedio_hr * 7 / 4)) && (flag_cvp == 3))
                {
                    flag_arrit_pnp = true;
                }

                if ((flag_detec_mkpasos) && (flag_cvp == 3)) //&& (flag_tipo_pulso))
                {
                    cta_mkpasos++;

                    if ((cta_mkpasos == 75) && (flag_detec_mkpasos))     // Equivale a 300 ms
                    {
                        cta_mkpasos = 75;
                        flag_arrit_pnc = true;
                        flag_detec_mkpasos = false;

                    }

                }

                if (cont_raw_cero > 375)
                {
                    if (flag_info_arr_01 == 1)
                    {
                        tipo_arritmia = ASYSTOLE;
                        cont_ok = 7;
                    }

                }

                if (tipo_arritmia != tipo_ant)
                    flag_detecta = true;
                tipo_ant = tipo_arritmia;
            }
        }
    }
}

void tarea3(void *args)
{
    /* Tarea que recibe comandos y configuración  desde la App . */

    int32_t     transferOK,status;
    UART_Transaction trans;
    uint8_t     gUartBuffer[APP_UART_BUFSIZE];
    char        gCmdUartReceiveBuffer = 0;  //[CMD_UART_RECEIVE_BUFSIZE] = {0};
    char        BuffCmdApp[CMD_UART_BUFSIZE] = {0};

    #define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
        do { \
            if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
            { \
                DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
            } \
        } while(0) \

    bool    flag_permanente = true;
    bool    flag_info_version = false;
    bool    flag_envio_activo = false;      // activar desacivar el envio de alarmas
    bool    flag_ping = false;
    bool    flag_inactivacion = false;

    bool flag_aviso_parcial = false;
    bool flag_aviso_total = false;

    uint8_t flag_aviso_asystole = 0;
    uint8_t flag_aviso_fib_vent = 0;
    uint8_t flag_aviso_tach_vent = 0;
    uint8_t flag_aviso_tachy = 0;
    uint8_t flag_aviso_brady = 0;
    uint8_t flag_aviso_cvp_min = 0;
    uint8_t flag_aviso_paired = 0;
    uint8_t flag_aviso_tv_2 = 0;
    uint8_t flag_aviso_bigeminy = 0;
    uint8_t flag_aviso_trigeminy = 0;
    uint8_t flag_aviso_missbeat = 0;
    uint8_t flag_aviso_RenT = 0;
    uint8_t flag_aviso_pnp = 0;
    uint8_t flag_aviso_pnc = 0;

    int i = 0;  //,k = 0;
    char fin_cadena = '\n';

    char parametro[5] = {0};

    status = SemaphoreP_constructBinary(&gUartWriteDoneSem_1, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&gUartReadDoneSem_1, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    UART_Transaction_init(&trans);                                              // Inicializar la estructura UART_Transaction


    while(1)
    {
        gNumBytesWritten_1 = 0U;        // borrar para pruebas con raspberry
        trans.buf   = &gUartBuffer[0U];
        trans.count = sprintf(trans.buf,"Tarea 3: \n");
        transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
        APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

        //  * Wait for write completion
        SemaphoreP_pend(&gUartWriteDoneSem_1, SystemP_WAIT_FOREVER);
        DebugP_assert(gNumBytesWritten_1 == strlen(trans.buf));    //   */

        do {
            //  --------------- lectura de comandos enviados por la App
            for (i = 0;i < CMD_UART_BUFSIZE; i++)
            {
                gNumBytesRead_1 = 0U;
                trans.buf   = &gCmdUartReceiveBuffer;   //[0U];
                trans.count = CMD_UART_RECEIVE_BUFSIZE;
                transferOK = UART_read(gUartHandle[CONFIG_UART0], &trans);
                APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

                ///    Wait for read completion
                SemaphoreP_pend(&gUartReadDoneSem_1, SystemP_WAIT_FOREVER);
                DebugP_assert(gNumBytesRead_1 == CMD_UART_RECEIVE_BUFSIZE);

                if (gCmdUartReceiveBuffer == fin_cadena)
                {
                    i = CMD_UART_BUFSIZE +1;
                    break;
                    //      k = i;
                }

                BuffCmdApp[i] = gCmdUartReceiveBuffer;
            }       //    */


            if (BuffCmdApp[0] == 'I') {
                flag_info_version = true;
            }
            else if (BuffCmdApp[0] == 'R') {
                flag_relearning = true;
                flag_initTraining = true;
            }
            else if (BuffCmdApp[0] == 'P') {
                flag_ping = true;
            }
            else if ((BuffCmdApp[0] == 'D') && (BuffCmdApp[1] == ':') && (BuffCmdApp[2] == 'A')) {
                flag_envio_activo = true;
            }
            else if ((BuffCmdApp[0] == 'D') && (BuffCmdApp[1] == ':') && (BuffCmdApp[2] == 'I')) {
                flag_envio_activo = false;
                flag_initTraining = false;
            }
            else if ((BuffCmdApp[0] == 'T') && (BuffCmdApp[1] == ':') && (BuffCmdApp[2] == 'A')) {
                flag_aviso_parcial = true;
            }
            else if ((BuffCmdApp[0] == 'T') && (BuffCmdApp[1] == ':') && (BuffCmdApp[2] == 'M')) {
                flag_aviso_total = true;
            }
            else if ((BuffCmdApp[0] == 'T') && (BuffCmdApp[1] == ':') && (BuffCmdApp[2] == 'I')) {
                flag_inactivacion =  true;

            }
            else if ((BuffCmdApp[0] == '0') && (BuffCmdApp[1] == '1') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_asystole = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_asystole = 0;
                else if ((BuffCmdApp[3] >= 48) && (BuffCmdApp[3] <= 57))
                {
                    for (int j = 0; j < 2; j++)
                        parametro[j] = BuffCmdApp[j + 3];
                    limite_asistole = atoi(parametro);
                }
            }
            else if ((BuffCmdApp[0] == '0') && (BuffCmdApp[1] == '2') && (BuffCmdApp[2] == ':'))
            {
                if(BuffCmdApp[3] == 'A')
                    flag_aviso_fib_vent = 1;
                else if(BuffCmdApp[3] == 'I')
                    flag_aviso_fib_vent = 0;
            }
            else if ((BuffCmdApp[0] == '0') && (BuffCmdApp[1] == '3') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_tach_vent = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_tach_vent = 0;
                else if ((BuffCmdApp[3] >= 48) && (BuffCmdApp[3] <= 57))
                {
                    for (int j = 0; j < 3; j++)
                        parametro[j] = BuffCmdApp[j + 3];
                    limite_TachVent = atoi(parametro);
                }
            }
            else if ((BuffCmdApp[0] == '0') && (BuffCmdApp[1] == '4') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_tachy = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_tachy = 0;
                else if ((BuffCmdApp[3] >= 48) && (BuffCmdApp[3] <= 57))
                {
                    for (int j = 0; j < 3; j++)
                        parametro[j] = BuffCmdApp[j + 3];
                    limite_Tachy = atoi(parametro);
                }
            }
            else if ((BuffCmdApp[0] == '0') && (BuffCmdApp[1] == '5') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_brady = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_brady = 0;
                else if ((BuffCmdApp[3] >= 48) && (BuffCmdApp[3] <= 57))
                {
                    for (int j = 0; j < 2; j++)
                        parametro[j] = BuffCmdApp[j + 3];
                    limite_brady = atoi(parametro);
                }
            }
            else if ((BuffCmdApp[0] == '0') && (BuffCmdApp[1] == '6') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_cvp_min = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_cvp_min = 0;
            }
            else if ((BuffCmdApp[0] == '0') && (BuffCmdApp[1] == '7') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_paired = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_paired = 0;
            }
            else if ((BuffCmdApp[0] == '0') && (BuffCmdApp[1] == '8') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_tv_2 = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_tv_2 = 0;
            }
            else if ((BuffCmdApp[0] == '0') && (BuffCmdApp[1] == '9') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_bigeminy = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_bigeminy = 0;
            }
            else if ((BuffCmdApp[0] == '1') && (BuffCmdApp[1] == '0') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_trigeminy = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_trigeminy = 0;
            }
            else if ((BuffCmdApp[0] == '1') && (BuffCmdApp[1] == '1') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_missbeat = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_missbeat = 0;
            }
            else if ((BuffCmdApp[0] == '1') && (BuffCmdApp[1] == '2') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_RenT = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_RenT = 0;
            }
            else if ((BuffCmdApp[0] == '1') && (BuffCmdApp[1] == '3') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_pnc = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_pnc = 0;
            }
            else if ((BuffCmdApp[0] == '1') && (BuffCmdApp[1] == '4') && (BuffCmdApp[2] == ':'))
            {
                if (BuffCmdApp[3] == 'A')
                    flag_aviso_pnp = 1;
                else if (BuffCmdApp[3] == 'I')
                    flag_aviso_pnp = 0;
            }   //  */

            for (int j = 0; j < 4; j++)
                parametro[j] = 0;

            gNumBytesWritten_1 = 0U;
            trans.buf   = &gUartBuffer[0U];
            if (flag_info_version)
            {
                trans.count = sprintf(trans.buf,"I:%s\n",VERSION);
                flag_info_version = false;
            }

            else
            {
                if (flag_start_ready)
                    trans.count = sprintf(trans.buf,"%s\n",BuffCmdApp);
            }


            transferOK = UART_write(gUartHandle[CONFIG_UART0], &trans);
            APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

                //  * Wait for write completion
            SemaphoreP_pend(&gUartWriteDoneSem_1, SystemP_WAIT_FOREVER);
            DebugP_assert(gNumBytesWritten_1 == strlen(trans.buf));

            if (flag_relearning)
            {
                flag_trained = true;
                flag_inicio_datos = true;
                flag_learning_1 = true;
                cont_latido = 0;
                cont_init_frec = 0;
                flag_relearning = false;
                tipo_arritmia = 0;
                cuenta_cvp = 0;
            }

            // ---------- Limpieza de buffer de comando
            for (i = 0;i < CMD_UART_BUFSIZE;i++)
                BuffCmdApp[i] = 0;
          // ----------------- Identifación de los tipos de arritmias de acuerdo a las alarmas activadas

            if (flag_aviso_total)
            {
                flag_aviso_asystole = 1;
                flag_aviso_fib_vent = 1;
                flag_aviso_tach_vent = 1;
                flag_aviso_tachy = 1;
                flag_aviso_brady = 1;
                flag_aviso_cvp_min = 1;
                flag_aviso_paired = 1;
                flag_aviso_tv_2 = 1;
                flag_aviso_bigeminy = 1;
                flag_aviso_trigeminy = 1;
                flag_aviso_missbeat = 1;
                flag_aviso_RenT = 1;
                flag_aviso_pnc = 1;
                flag_aviso_pnp = 1;

                flag_aviso_total = false;
            }

            if (flag_aviso_parcial)
            {
                flag_aviso_asystole = 1;
                flag_aviso_fib_vent = 1;
                flag_aviso_tach_vent = 1;
                flag_aviso_tachy = 1;
                flag_aviso_brady = 1;
                flag_aviso_cvp_min = 1;
                flag_aviso_paired = 1;
                flag_aviso_tv_2 = 1;
                flag_aviso_bigeminy = 1;
                flag_aviso_trigeminy = 1;
                flag_aviso_missbeat = 1;
                flag_aviso_RenT = 1;

                flag_aviso_parcial = false;
            }

            if (flag_inactivacion)
            {
                flag_aviso_parcial = 0;
                flag_aviso_total = 0;
                flag_aviso_asystole = 0;
                flag_aviso_fib_vent = 0;
                flag_aviso_tach_vent = 0;
                flag_aviso_tachy = 0;
                flag_aviso_brady = 0;
                flag_aviso_cvp_min = 0;
                flag_aviso_paired = 0;
                flag_aviso_tv_2 = 0;
                flag_aviso_bigeminy = 0;
                flag_aviso_trigeminy = 0;
                flag_aviso_missbeat = 0;
                flag_aviso_RenT = 0;
                flag_aviso_pnc = 0;
                flag_aviso_pnp = 0;

                flag_inactivacion = false;
            }

            if (flag_envio_activo)
            {
                flag_info_arr_01 = flag_aviso_asystole;
                flag_info_arr_02 = flag_aviso_fib_vent;
                flag_info_arr_03 = flag_aviso_tach_vent;
                flag_info_arr_04 = flag_aviso_tachy;
                flag_info_arr_05 = flag_aviso_brady;
                flag_info_arr_06 = flag_aviso_cvp_min;
                flag_info_arr_07 = flag_aviso_paired;
                flag_info_arr_08 = flag_aviso_tv_2;
                flag_info_arr_09 = flag_aviso_bigeminy;
                flag_info_arr_10 = flag_aviso_trigeminy;
                flag_info_arr_11 = flag_aviso_missbeat;
                flag_info_arr_12 = flag_aviso_RenT;
                flag_info_arr_13 = flag_aviso_pnc;
                flag_info_arr_14 = flag_aviso_pnp;
            }
            else
            {
                flag_info_arr_01 = 0;
                flag_info_arr_02 = 0;
                flag_info_arr_03 = 0;
                flag_info_arr_04 = 0;
                flag_info_arr_05 = 0;
                flag_info_arr_06 = 0;
                flag_info_arr_07 = 0;
                flag_info_arr_08 = 0;
                flag_info_arr_09 = 0;
                flag_info_arr_10 = 0;
                flag_info_arr_11 = 0;
                flag_info_arr_12 = 0;
                flag_info_arr_13 = 0;
                flag_info_arr_14 = 0;
            }

            SemaphoreP_destruct(&gUartWriteDoneSem_1);
            SemaphoreP_destruct(&gUartReadDoneSem_1);

        } while (flag_permanente);   //   */

    }
}

int32_t FiltroPasaBanda(int32_t data_in)
{
    static int yl1 = 0, yl2 = 0, xl[26], nl = 12;
    static int yh1 = 0, xh[66], nh = 32;
    int yl0;
    int yh0;

    xl[nl] = xl[nl + 13] = data_in;
    yl0 = (yl1 << 1) - yl2 + xl[nl] - (xl[nl + 6] << 1) + xl[nl + 12];
    yl2 = yl1;
    yl1 = yl0;
    yl0 >>= 5;

    if(--nl < 0)
        nl = 12;

    xh[nh] = xh[nh + 33] = yl0;
    yh0 = yh1 + xh[nh] - xh[nh + 32];
    yh1 = yh0;
    if(--nh < 0)
        nh = 32;

    return(xh[nh + 16] - (yh0 >> 5));

}

int32_t Derivativa(int32_t data_in)
{
    int y, i;
    static int x_derv[4];
    int32_t sqr_y;
    /*y = 1/8 (2x( nT) + x( nT - T) - x( nT - 3T) - 2x( nT - 4T))*/
    y = (data_in << 1) + x_derv[3] - x_derv[1] - ( x_derv[0] << 1);
    y >>= 3;
    for (i = 0; i < 3; i++)
        x_derv[i] = x_derv[i + 1];
    x_derv[3] = data_in;

    sqr_y = y * (y >> 2);

    return(sqr_y);
}

int32_t MovingWindowIntegral(int32_t data_in)
{
    static int x[32];
    int64_t ly;
    int32_t y;

    if(++ptr == 32)
        ptr = 0;
    sum_integral -= x[ptr];
    sum_integral += data_in;
    x[ptr] = data_in;
    ly = sum_integral >> 5;
    if(ly > 10000)
        y = 10000;
    else
    y = (int) ly;

    return(y);
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

void EncontrarPuntoR(int dato_in)
{       // función dedicada a encontrar el punto R en cada onda

    int y, i, pendiente_I;
    static int x_derv[4];
    int valor_mayor = 0;
    int  umbral_asc = 12;
    int  umbral_desc = 6;

    contador_muestras++;
    // ***** Calculo derivada de la señal de entrada  ****
    y = (dato_in << 1) + x_derv[3] - x_derv[1] - ( x_derv[0] << 1);
    y >>= 3;
    for (i = 0; i < 3; i++)
        x_derv[i] = x_derv[i + 1];
    x_derv[3] = dato_in;
    pendiente_I = y;

    if (dato_in >= umbral_asc)
    {
        if (pendiente_I > 2)
        {
            if (flag_reset_cta)
            {
                punto_heartrate = contador_muestras;
                flag_reset_cta = false;
                flag_punto_Q = true;
                contador_muestras = 0;
                flag_data_aux  = flag_trained;

                if (!flag_trained)
                    flag_heartbeat = false;
            }
            valor_mayor = MAX(dato_in,valor_pico);
            valor_pico = valor_mayor;
        }
        else
        {
            if (dato_in < valor_pico)
            {
                cta_negativo_S++;
                if (cta_negativo_S == 2)
                {
                    flag_punto_R = true;
                    valor_punto_R = valor_pico;
                }
                else if (cta_negativo_S > 2)
                {
                    if (flag_punto_R)
                        flag_punto_R = false;

                }
            }
        }
    }
    else if (dato_in < umbral_desc)
    {
        valor_pico = 0;
        cta_negativo_S = 0;
        if (abs(pendiente_I)< 5)
            flag_reset_cta = true;

    }
    gradiente = pendiente_I;

}

void ProcesamientoSegmentoRR()
{
    int32_t delta_RR_actual;
    int64_t suma_RR_actual = 0;
    int32_t media_RR_actual = 0;
    int8_t i;
    float   T_RR_actual = 0;    //periodo de RR para referencia (milisegundos)
    float   delta_RR_real = 0;

    float lim_taquicardia = 60000/limite_Tachy;       // [milisegundos]
    float lim_bradicardia = 60000/limite_brady;       // [milisegundos]

    int lim_tachy_int = (int)lim_taquicardia;
    int lim_brady_int = (int)lim_bradicardia;

    int lim_asistole =  limite_asistole * 1000;

    tiempo_RR_1 = ClockP_getTimeUsec();
    delta_RR_actual = (tiempo_RR_1 - tiempo_RR_2);
    delta_RR_real = (float)delta_RR_actual / 1000;
    tiempo_RR_2 = tiempo_RR_1;

    // promedio de últimos 8 pulsos detectados
    array_RR_actual[0] = delta_RR_actual;
    for (i = 8; i > 0; i--)
    {
        suma_RR_actual += array_RR_actual[i-1];
        array_RR_actual[i] = array_RR_actual[i-1];
    }
    media_RR_actual = suma_RR_actual >> 3;
    T_RR_actual = (float)media_RR_actual / 1000;   // periodo en milisegundos

    if (flag_inicio_datos)
    {
        contador_aprendizaje++;
        if (contador_aprendizaje >= 9)
        {
            flag_inicio_datos = false;
            contador_aprendizaje = 0;
            for (i = 8; i > 0; i--)
                array_RR_valido[i] = array_RR_actual[i];

            T_RR_valido = T_RR_actual;   // periodo en milisegundos

            Thr_RR_bajo = T_RR_valido * 0.92;       // define valores iniciales del rango T_RR_valido
            Thr_RR_alto = T_RR_valido * 1.16;
            cont_ritmo_regular = 0;
            cont_ritmo_irregular = 0;
        }
    }
    else    // despues de los primeros 8 latidos analizados
    {
        if ((delta_RR_real >= Thr_RR_bajo) && (delta_RR_real <= Thr_RR_alto))
        {
            cont_ritmo_regular++;
            suma_RR_valido = 0;

            cont_ritmo_irregular = 0;
            if (cont_ritmo_regular == 4)    // después de 4 pulsos en ritmo regular
            {
                flag_arritmia_sinus = false;
                flag_bradicardia = false;
                flag_taquicardia = false;
            }

            for (i = 8; i > 0; i--)
            {
                suma_RR_valido += array_RR_valido[i-1];
                array_RR_valido[i] = array_RR_valido[i-1];
            }
            array_RR_valido[0] = delta_RR_actual;
            if (cont_ritmo_regular > 7)
            {
                media_RR_valido = suma_RR_valido >> 3;
                T_RR_valido = (float)media_RR_valido / 1000;   // periodo en milisegundos

                Thr_RR_bajo = T_RR_valido * 0.90;       // define valores iniciales del rango
                Thr_RR_alto = T_RR_valido * 1.18;
                Thr_RR_miss = T_RR_valido * 1.66;
            }
        }
        else    // si un latido se encuentra fuera del rango normal de entrenamiento...
        {
            cont_ritmo_irregular++;
            cont_ritmo_regular = 0;
            if (cont_ritmo_irregular >= 3)
                flag_arritmia_sinus = true;
            if (cont_ritmo_irregular == 1)
            {
                if ((delta_RR_real > Thr_RR_miss) && (delta_RR_real < lim_asistole) && (flag_cvp < 1))
                    flag_beat_miss = true;
                // no entra en rango aceptable, pero tampoco está en el rango de latido perdido
                else if ((delta_RR_real < Thr_RR_miss) || (delta_RR_real < lim_brady_int))
                    flag_arritmia_sinus = false;
            }
            else if (cont_ritmo_irregular > 2)
            {
                if ((delta_RR_real > lim_brady_int) && (delta_RR_real < lim_asistole) && (flag_cvp < 1))// && (delta_RR_real < LIMITE_BRADICARDIA))       // DETECCIÓN DE LATID PERDIDO
                {
                    flag_bradicardia = true;
                    flag_beat_miss = false;
                    flag_taquicardia = false;

                }
                else if ((delta_RR_real <= lim_tachy_int) && (flag_cvp < 1))
                {
                    flag_taquicardia = true;
                    flag_bradicardia = false;
                }
                else
                {
                    flag_arritmia_sinus = false;
                    flag_taquicardia = false;
                    flag_bradicardia = false;
                }
            }
        }

    }

  //  variable_aux = round(Thr_RR_miss);
    time_RR_actual = (int)delta_RR_real;
    time_RR_valido = (int)T_RR_valido;
}


void DefinicionUmbrales(int valor_R)       // cuando detecta pico R
{
    int pico_actual;

    pico_actual =  valor_R;
    if (flag_learning_1)
    {
        cont_latido++;

        if (cont_latido == 2)
            suma_pico = pico_actual;
        else if (cont_latido > 2)
        {
            umbral_ref = (suma_pico + pico_actual) >> 1;
            cont_latido = 0;
            flag_learning_1 = false;
            flag_actualiza = true;
        }
    }
    else
    {
        if (flag_actualiza)
            flag_actualiza = false;
        else
        {
            if ((pico_actual > ((umbral_ref >> 1) - 10)) && (pico_actual < (2 * umbral_ref)))
            {
                umbral_ref = (umbral_ref >> 3) + (pico_actual >> 3) + (pico_actual >> 2) + (pico_actual >> 1);
                flag_heartbeat = true;
            }
        }
    }

}

float euclidean_distance(int point1[2], int point2[2])
{
    return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
}

int get_neighbors(int data[NUM_POINTS][2], int point[2], float eps, int neighbors[NUM_POINTS][2],bool visited[NUM_POINTS])
{
    int other_point[2] = {0};
    int index = 0;
    int i=0;
    int delta_abs=0;
    int delta_abs2 =0;

    for (i = 0; i < NUM_POINTS; i++)
    {
        other_point[0] = data[i][0];
        other_point[1] = data[i][1];
        delta_abs = abs(point[1]-other_point[1]);
        delta_abs2 = abs(point[0]-other_point[0]);

        if ((!visited[i]) && (delta_abs <= eps)&& ((delta_abs2 < width_neigh)))
        {
            neighbors[index][0] = data[i][0];
            neighbors[index][1] = data[i][1];
            index++;
        }
    }
    return index;
}

int get_neighbors_2(int data[NUM_POINTS_3][2], int point[2], float eps, int neighbors[NUM_POINTS_3][2])
{
    int other_point[2] = {0};
    int index = 0;
    int i=0;
    for (i = 0; i < NUM_POINTS_3; i++)
    {
        other_point[0] = data[i][0];
        other_point[1] = data[i][1];
        if ((euclidean_distance(point, other_point) <= eps))
        {
            neighbors[index][0] = data[i][0];
            neighbors[index][1] = data[i][1];
            index++;
        }
    }
    return index;
}

int expand_cluster(int data[NUM_POINTS][2],int neighbors[NUM_POINTS][2],float eps,bool visited[NUM_POINTS],int cluster[NUM_POINTS][2],int len_neighbors)
{
    int i,j;
    int len_neighbor_new=0;
    int len_neighbor_max=0;
    int new_neighbors[NUM_POINTS][2]={0};
    int point_aux[2];


    for(i=0;i<len_neighbors;i++)
    {
        int new_point[2] = {neighbors[i][0],neighbors[i][1]};
        len_neighbor_new = get_neighbors(data,new_point,eps,new_neighbors,visited);
        if (len_neighbor_new > len_neighbor_max)
        {
            len_neighbor_max = len_neighbor_new;
            point_aux[0] = new_point[0];
            point_aux[1] = new_point[1];
        }
    }

    len_neighbor_new = get_neighbors(data,point_aux,eps,cluster,visited);
    for(i=0;i<len_neighbor_new;i++)
    {
        for(j=0;j<NUM_POINTS;j++)
        {
            if ((cluster[i][0]==data[j][0])&&(cluster[i][1]==data[j][1])&&(!visited[j]))
            {
                visited[j] = true;
            //    break;
            }
        }

    }
    return len_neighbor_new;
}

int expand_cluster_2(int data[NUM_POINTS_3][2],int neighbors[NUM_POINTS_3][2],float eps,bool visited[NUM_POINTS_3],int cluster[NUM_POINTS_3][2],int len_neighbors)//,int point[2],,   int minPts,  )
{
    int i,j;
    int len_neighbor_new=0;
    int len_neighbor_max=0;
    int new_neighbors[NUM_POINTS_3][2]={0};
    int point_aux[2];


    for(i=0;i<len_neighbors;i++)
    {
        int new_point[2] = {neighbors[i][0],neighbors[i][1]};
        len_neighbor_new = get_neighbors_2(data,new_point,eps,new_neighbors);
        if (len_neighbor_new > len_neighbor_max)
        {
            len_neighbor_max = len_neighbor_new;
            point_aux[0] = new_point[0];
            point_aux[1] = new_point[1];
        }
    }

    len_neighbor_new = get_neighbors_2(data,point_aux,eps,cluster);
    for(i=0;i<len_neighbor_new;i++)
    {
        //int point_aux2
        for(j=0;j<NUM_POINTS_3;j++)
        {
            if ((cluster[i][0]==data[j][0])&&(cluster[i][1]==data[j][1])&&(!visited[j]))
            {
                visited[j] = true;
            }
        }
    }
    return len_neighbor_new;
}

int dbscan(int data[NUM_POINTS][2], float eps, int minPts, int clusters[NUM_POINTS_2][NUM_POINTS][2], int len_clusters[NUM_POINTS_2], int no_clusters[NUM_POINTS][2], int *len_no_clusters, int len)
{
    bool visited[NUM_POINTS] = {false};
    int index=0;
    int i = 0;
    int no_cluster_aux=0;
    int aux_cont=0;

    for (i = 0; i < len; i++)
    {
        int point[2] = {data[i][0], data[i][1]};
        if (!visited[i])
        {
            int neighbors[NUM_POINTS][2] = {0};
            int len_neighbors = get_neighbors(data, point, eps, neighbors,visited);
            if (len_neighbors >= minPts)
            {
                index =  expand_cluster(data, neighbors,eps, visited, clusters[*len_no_clusters],len_neighbors);
                len_clusters[*len_no_clusters] = index;
                (*len_no_clusters)++;
                aux_cont++;
            }
            else
            {
                no_clusters[no_cluster_aux][0] = point[0];
                no_clusters[no_cluster_aux][1] = point[1];
                (no_cluster_aux)++;
            }
        }
    }
    *len_no_clusters=no_cluster_aux;
    return aux_cont;
}

void dbscan_2(int data_2[NUM_POINTS_3][2], float eps, int minPts, int clusters[NUM_POINTS_3][NUM_POINTS_3][2], int len_clusters[NUM_POINTS_3], int no_clusters[NUM_POINTS_3][2], int *len_no_clusters, int len)
{
    bool visited[NUM_POINTS_3] = {false};
    int index=0;
    int i = 0;
    int no_cluster_aux=0;

    for (i = 0; i < len; i++)
    {
        int point[2] = {data_2[i][0], data_2[i][1]};
        if (!visited[i])
        {
            int neighbors[NUM_POINTS_3][2] = {0};
            int len_neighbors = get_neighbors_2(data_2,point, eps, neighbors);
            if (len_neighbors >= minPts)
            {
                index =  expand_cluster_2(data_2, neighbors,eps, visited, clusters[*len_no_clusters],len_neighbors);
                len_clusters[*len_no_clusters] = index;
                (*len_no_clusters)++;
            }
            else
            {
                no_clusters[no_cluster_aux][0] = point[0];
                no_clusters[no_cluster_aux][1] = point[1];
                (no_cluster_aux)++;
            }
        }
    }
    *len_no_clusters=no_cluster_aux;

}

int find_cluster(int point[2], int clusters[NUM_POINTS_2][NUM_POINTS][2], int len_clusters[NUM_POINTS_2])
{
    int i = 0;
    for (i = 0; i < NUM_POINTS_2; i++)
    {
        if (len_clusters[i] > 0)
        {
            int j = 0;
            for (j = 0; j < len_clusters[i]; j++)
            {
                if ((euclidean_distance(clusters[i][j],point) <= 10))
                {
                    return i;
                }
            }
        }
    }
    return -1;
}

int find_arritmia(int pulso[250][2], int clusters[NUM_POINTS_2][NUM_POINTS][2], int len_clusters[NUM_POINTS_2], int long_clusters, int long_pulso)
{
    int aux_cont=0;
    int resultado=0;
    int aux_cont_2 = 0;
    int aux_cont_3 = 0;
    int lim_int = 0;

    for (int i = 0; i < long_pulso; i++)
    {
        if (pulso[i][1] > 5)
            aux_cont_3++;
    }
    for (int i = 0; i < long_clusters; i++)
    {

        if (clusters[i][1][1] > 5)
        {
            int min_cluster0 = 20000;
            int max_cluster0 = 0;
            int min_cluster1 = 20000;
            int max_cluster1 = 0;
            aux_cont_2++;
            for (int j = 0; j < len_clusters[i]; j++)
            {
                if (clusters[i][j][0] > max_cluster0)
                {
                    max_cluster0 = clusters[i][j][0];
                    if (max_cluster0 > lim_int)
                        lim_int = max_cluster0;
                 }
                if (clusters[i][j][0] < min_cluster0)
                    min_cluster0 = clusters[i][j][0];

                if (clusters[i][j][1] > max_cluster1)
                    max_cluster1 = clusters[i][j][1];

                if (clusters[i][j][1] < min_cluster1)
                    min_cluster1 = clusters[i][j][1];
            }

            for (int j = 0; j < long_pulso; j++)
            {
                if ((pulso[j][0] < max_cluster0) && (pulso[j][0] > min_cluster0) &&
                        (pulso [j][1] < max_cluster1) && (pulso[j][1] > min_cluster1))
                {
                    aux_cont++;
                    break;
                }
            }
        }
    }

    /*resultado = (aux_cont_3 - lim_int);
    if ((resultado > 5) && (flag_aviso_mk))
        return 3;
    else
    {       */
         resultado = (aux_cont_2 - aux_cont);
         if (resultado <= 1)
             return 0;

         return 1;
   // }
   // return resultado;

}


void uart_echo_write_callback(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_SUCCESS == trans->status);
    gNumBytesWritten = trans->count;
    SemaphoreP_post(&gUartWriteDoneSem);

    return;
}

void uart_echo_read_callback(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_SUCCESS == trans->status);
    gNumBytesRead = trans->count;
    SemaphoreP_post(&gUartReadDoneSem);

    return;
}

void uart_echo_write_callback_1(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_SUCCESS == trans->status);
    gNumBytesWritten_1 = trans->count;
    SemaphoreP_post(&gUartWriteDoneSem_1);

    return;
}       //  */

void uart_echo_read_callback_1(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_SUCCESS == trans->status);
    gNumBytesRead_1 = trans->count;
    SemaphoreP_post(&gUartReadDoneSem_1);

    return;
}
