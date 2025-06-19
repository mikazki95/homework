
/*
 * Definiciones.h
 *
 *  Created on: 5 jun 2023
 *      Author: Gabriela Morales Timal
 *
 *  Version 1.00   primera version entregada
 *  Version 1.01   correcciones en detección de arritmia debidas a fallas en marcapasos. 14/08/2023
 *
 */


#ifndef DEFINICIONES_H_
#define DEFINICIONES_H_

/*typedef enum {
    false = 0, true = 1
} bool;
typedef bool bolean;        //      */

#define VERSION ("ARRITMIA,1.01")

// Definiciones para datos recibidos por UART_1 datos ECG  //
#define APP_UART_BUFSIZE            (200U)
#define APP_UART_RECEIVE_BUFSIZE    (24U)      // Numero de bytes recibidos del STM32
// datos y enviados a la Aplicación para comandos
#define CMD_UART_BUFSIZE            (8U)       //
#define CMD_UART_RECEIVE_BUFSIZE    (1U)// (8U)      // Numero de bytes recibidos

static SemaphoreP_Object gUartWriteDoneSem;     // para MCU_UART1  (Datos_ECG)
static SemaphoreP_Object gUartReadDoneSem;      // para MCU_UART1  (Datos_ECG)
static SemaphoreP_Object gUartWriteDoneSem_1;    // para UART1  (Comandos Raspberry)
static SemaphoreP_Object gUartReadDoneSem_1;     // para UART1  (Comandos Raspberry)

#define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
        do { \
            if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
            { \
                DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
        } \
} while(0) \

#define BUFSIZE_ECG                  (10U)
#define UMBRAL_EQUIPOTENCIAL        (100U)
#define LIMITE_TAQUICARDIA           (375)  // Periodo correspondiente a 160bpm
#define LIMITE_BRADICARDIA          (60000/50)  // Periodo correspondiente a 50bpm      ms
#define LIMITE_ASISTOLE             (4000)      // ms
#define MAX_SIZE_CMD                (8U)    //tamaño máximo de cadena para comandos


//   constantes para tarea 2 //
#define NUM_POINTS                  (1000U)    // tamaño de array
#define NUM_POINTS_2                (210U)     // tamaño de cluster = NUM_POINTS_2 / Minimo de puntos mínimo
#define NUM_POINTS_3                (20U)
#define NUM_POINTS_4                (400U)
#define MAX(a,b) ((a)>(b)?(a):(b))

#define prueba  (0U)

//  *******************  V A R I A B L E S   G L O B A L E S   **********************  /


volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;
volatile uint32_t gNumBytesRead_1 = 0U, gNumBytesWritten_1 = 0U;

// --------------------------- Definición de arritmias ---------------------------

enum Arritmias {
    NORMAL,
    ASYSTOLE,
    VENT_FIBR,
    VENT_TACH,
    TACHY,
    BRADY,
    CVP_pMIN,
    PAIRED_CVP,
    TV_2,
    BIGEMINY,
    TRIGEMINY,
    MISS_BEAT,
    R_EN_T,
    PNC,        // 13  verificado con Gandy 15/08/2023
    PNP
};

bool  flag_start_ready = false;
bool  flag_relearning = false;
bool  flag_alarma_arritmia = false;
bool  flag_detecta = false;
bool  flag_detec_cvp = false;
bool  flag_continuar = false;
bool  flag_punto_R = false;
bool  flag_define_fs =  false;
bool  flag_reset_cta = false;
bool  flag_inicio_datos = false;
bool  flag_data_aux = false;
bool  flag_heartbeat = false;     // bandera para identificar latidos validos para entrenamiento
bool  flag_pacemaker = false;
bool  flag_arritmia_cvp = false;
bool  flag_arritmia_sinus = false;
bool  flag_beat_miss = false;
bool  flag_bradicardia = false;
bool  flag_taquicardia = false;
bool  flag_detec_mkpasos = false;     // 05/07/23
bool  flag_pulso_mk = false;
bool  flag_aviso_mk = false;
bool  flag_beat_ok = false;
bool  flag_initTraining = false;

bool flag_aux = false;
bool flag_cvp_mk = false;
bool flag_tipo_pulso = false;

uint8_t   flag_info_arr_01 = prueba;
uint8_t   flag_info_arr_02 = prueba;
uint8_t   flag_info_arr_03 = prueba;
uint8_t   flag_info_arr_04 = prueba;
uint8_t   flag_info_arr_05 = prueba;
uint8_t   flag_info_arr_06 = prueba;
uint8_t   flag_info_arr_07 = prueba;
uint8_t   flag_info_arr_08 = prueba;
uint8_t   flag_info_arr_09 = prueba;
uint8_t   flag_info_arr_10 = prueba;
uint8_t   flag_info_arr_11 = prueba;
uint8_t   flag_info_arr_12 = prueba;
uint8_t   flag_info_arr_13 = prueba;
uint8_t   flag_info_arr_14 = prueba;

int     limite_brady = 50;     // bpm
int     limite_Tachy = 150;    // bpm
int     limite_TachVent = 150;      // bpm
int     limite_asistole = 4;        // segundos

uint8_t valida_d1 = 0XA5;  //0x11;      //  validación dato derivación 1
uint8_t signo_dato = 0X00;       // cambiar a 0x00
uint8_t valida_d2 = 0x5A;        // validación dato derivación 2
uint8_t valido_pace = 0X55;

uint8_t dato_in_d1[4];      // bytes del dato recibido
uint8_t dato_in_d2[4];      // bytes del dato recibido
uint8_t pulso_marcapasos = 0;       // aviso de detección del pulso de marcapasos
int32_t dato_raw_d1,dato_raw_d2;        //dato concatenado de los bytes recibidos

int valor_punto_R = 0,valor_pico=0;

int32_t ecg_filt_ind1 = 0,ecg_filt_ind2 = 0;
int32_t deriv_ecg_d1 = 0,deriv_ecg_d2 = 0;
int32_t integral_ecg_d1 = 0,integral_ecg_d2 = 0;
int32_t derivada_raw_d2 = 0;

int  contador_aprendizaje = 0;
int  cont_latido = 0;
int  cuenta_edge = 0;
uint32_t cont_ritmo_irregular = 0;
uint32_t cont_ritmo_regular = 0;
int cont_InitBeat = 0;

// ******   Variables gloabales para funciones
static int ptr = 0;
static int64_t sum_integral = 0;

int  cta_negativo_S = 0;
int  inicio_muestreo = 0;

int32_t gradiente=0;
int32_t array_RR_actual[9],array_RR_valido[9]={0};
int32_t suma_RR_valido = 0;
int32_t media_RR_valido = 0;

int  time_RR_valido = 0;
int  time_RR_actual = 0;
float Thr_RR_alto = 0,T_RR_valido = 0;
float Thr_RR_bajo = 0;
float Thr_RR_miss = 0;

int umbral_ref = 0;
int suma_pico = 0;
int32_t   variable00 = 0;

int32_t tiempo_RR_1=0,tiempo_RR_2=0;
int64_t tiempo_fs0=0,tiempo_fs1 =0;

int  contador_muestras = 0;      // para envio de identificador de inicio de ciclo
float delta_fs = 0,media_fs = 0;     //frecuencia de muestreo
//************** variables para tarea 2
int  contador_tarea_2 = 0;
int  cont_task_3 = 0;

int  point_in[2]={0};
int  data_aux[NUM_POINTS_4]={0};
int  data_pre[NUM_POINTS][2]={0};
float eps = 15;
int  minPts = 9;
int  clusters[NUM_POINTS_2][NUM_POINTS][2] = {0};
int  len_clusters[NUM_POINTS_2] = {0};
int  no_clusters[NUM_POINTS][2] = {0};
int  len_no_clusters = 0;
int  width_neigh = 5;

int tipo_arritmia = NORMAL;      // normal

      // variables cambio frecuencia
float eps_3 =   10;
int   minPts_3 =  2;
int   clusters_3[NUM_POINTS_3][NUM_POINTS_3][2] = {0};
int   len_clusters_3[NUM_POINTS_3] = {0};
int   no_clusters_3[NUM_POINTS_3][2] = {0};
int   len_no_clusters_3 = 0;

int   dato_frec [20][2] = {0};
int   cont_init_frec = 0;
int   punto_heartrate = 0;

bool  flag_learning_1 = false;
bool  flag_trained = false;      // indicar tiempo de entrenamiento para la red
bool  flag_training_ready = false;
bool  flag_dato_rdy = false;     // indica dato valido de entrada para la red
bool  flag_punto_Q = false;
bool  flag_actualiza = false;
bool  flag_arritmia = false;
bool  flag_RenT_RV = false;

int   flag_cvp = 0;
int8_t index_p_arritmia = 0;

int   pulso[750][2]={0};
int   num_clusters_1=0;
int   num_clusters_2=0;
int   variable_aux = 0;
int   variable_aux_2 = 0;
int   array_aux_1[30][2] = {0};
int   array_aux_2[30][2] = {0};
int   pulsos_arritmia [90][2] = {0};
int   fin_array = 0;
int   limite_taq = 150;
int   cuenta_cvp = 0;
int   cont_ok = 0;
int   cta_mkpasos = 0;       // 05/07/23  variables para marcapasos
int   cuenta_pulso_mk = 0;


// * Prototipos de funciones **************************************************** /

int32_t FiltroPasaBanda(int32_t data_in);
int32_t Derivativa(int32_t data_in);
int32_t MovingWindowIntegral(int32_t data_in);
int32_t DerivadaFuncion (int derivacion);
void    EncontrarPuntoR(int dato_in);
void    ProcesamientoSegmentoRR();
void    DefinicionUmbrales(int valor_R);
// funciones para red
float euclidean_distance(int point1[2], int point2[2]);
int get_neighbors(int data[NUM_POINTS][2], int point[2], float eps, int neighbors[NUM_POINTS][2],bool visited[NUM_POINTS]);
int get_neighbors_2(int data[NUM_POINTS_3][2], int point[2], float eps, int neighbors[NUM_POINTS_3][2]);
int expand_cluster(int data[NUM_POINTS][2],int neighbors[NUM_POINTS][2],float eps,bool visited[NUM_POINTS],
                   int cluster[NUM_POINTS][2],int len_neighbors);
int expand_cluster_2(int data[NUM_POINTS_3][2],int neighbors[NUM_POINTS_3][2],float eps,bool visited[NUM_POINTS_3],
                     int cluster[NUM_POINTS_3][2],int len_neighbors);
int dbscan(int data[NUM_POINTS][2], float eps, int minPts, int clusters[NUM_POINTS_2][NUM_POINTS][2],
            int len_clusters[NUM_POINTS_2], int no_clusters[NUM_POINTS][2], int *len_no_clusters, int len);
void dbscan_2(int data[NUM_POINTS_3][2], float eps, int minPts, int clusters[NUM_POINTS_3][NUM_POINTS_3][2],
              int len_clusters[NUM_POINTS_3], int no_clusters[NUM_POINTS_3][2], int *len_no_clusters, int len);
int find_cluster(int point[2], int clusters[NUM_POINTS_2][NUM_POINTS][2], int len_clusters[NUM_POINTS_2]);
//int find_cluster_2(int point[2], int clusters[NUM_POINTS_3][NUM_POINTS_3][2], int len_clusters[NUM_POINTS_3]);
int find_arritmia(int pulso[250][2], int clusters[NUM_POINTS_2][NUM_POINTS][2], int len_clusters[NUM_POINTS_2], int long_clusters, int long_pulso);
//void clean_Flags_Arr (void);

#endif /* DEFINICIONES_H_ */
