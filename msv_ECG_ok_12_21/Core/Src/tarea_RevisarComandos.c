/*
 * tarea_RevisarComandos.c
 *
 *  Created on: May 23, 2023
 *      Author: sakura
 */

//#include "main.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "led_rgb_simple.h"
#include "ads1298.h"               // Funciones ADS
#include "led_neopixel.h"          // Estrucuras y valores neopixel

#include "tarea_GraficarECG.h"     // Estructuras para graficar

#include "tarea_RevisarComandos.h"

const char versionRevisarComandos[] = LIB_VERSION_REVISARCOMANDOS;

extern bool flag_MSV_graficar;
extern ecg_Graficas_s derivacionesAGraficar;

extern bool flag_show_info;

extern bool flag_activa_apnea;
extern int valor_apnea;
extern uint8_t	paciente_tipo;

extern neoPixel_RutinaPrioridad_t prioridadNeopixel;
extern neoPixel_RutinaModo_t modoLeds;
extern neoPixel_Colors_t colorLeds;
extern bool blink_ON;

//Esta bandera se colocó en usbd_cdc_if.c en la sección de variables privadas
extern bool USB_RxFlag;     // Indica cuando se ha recibido un mensaje por USB
extern uint8_t* USB_Rx_Buf; // Puntero al buffer de recepción propio de la librería USB
extern uint32_t USB_Rx_Len; // Cantidad de caracteres recibidos

extern bool flag_Ping;

//Indicadores de los comandos recibidos
ledRGB_Instancia_t led1;
ledRGB_Instancia_t led2;

ecg_Graficas_s selectGraph (char *cadena)
{
    ecg_Graficas_s gtmp;
    gtmp.ecg_FiltroG1 = derivacionesAGraficar.ecg_FiltroG1; //Mantener el valor de los modos de filtrado
    gtmp.ecg_FiltroG2 = derivacionesAGraficar.ecg_FiltroG2;
    // strtol: [String to Long] Parámetros:
    // 1: Puntero al array string, la posición debe ser al primer dígito
    // 2: opcional, puntero donde se guardará la suiguiente letra (no número) del string, (para continuar buscando en la cadena.
    // NULL si no se usa
    // 3: Base para interpretar el número, yo uso base 10
    // La cadena de donde está MSV_SEL_GRAFICAS (G) tiene la forma ~G:X,Y\n (puede haber caracteres de ruido antes de la G)
    // Encontrar donde está la G con "pos_indice"
    // Encontrar la coma con "pos_coma"

    int8_t pos_indice = findStr (cadena, CMD_SEL_GRAFICAS);
    int8_t pos_coma = findStr (cadena, ",");
    uint8_t x_val = (uint8_t) strtoul ((char*) &cadena[pos_indice + 2], NULL, 10);
    uint8_t y_val = (uint8_t) strtoul ((char*) &cadena[pos_coma + 1], NULL, 10);
    // Seleccionar las graficas a utilizar
    if ((x_val == 0) || (x_val > 12))
        {
            gtmp.g1 = DERIVACION_NULL;
        }
    else
        {
            gtmp.g1 = x_val;
        }
    if ((y_val == 0) || (y_val > 12))
        {
            gtmp.g2 = DERIVACION_NULL;
        }
    else
        {
            gtmp.g2 = y_val;
        }
    return gtmp;
}

void tarea_RevisarComandos_Setup (void)
{
    //Inicializar los leds
    led1.PuertoR = RGB_R1_GPIO_Port;
    led1.PinR = RGB_R1_Pin;
    led1.PuertoG = RGB_G1_GPIO_Port;
    led1.PinG = RGB_G1_Pin;
    led1.PuertoB = RGB_B1_GPIO_Port;
    led1.PinB = RGB_B1_Pin;
    led1.R = true;
    led1.G = true;
    led1.B = true;
    led2.PuertoR = RGB_R2_GPIO_Port;
    led2.PinR = RGB_R2_Pin;
    led2.PuertoG = RGB_G2_GPIO_Port;
    led2.PinG = RGB_G2_Pin;
    led2.PuertoB = RGB_B2_GPIO_Port;
    led2.PinB = RGB_B2_Pin;
    led2.R = true;
    led2.G = true;
    led2.B = true;
    rgbSetLedColor (&led1, RGB_C_ROJO);
    rgbSetLedColor (&led2, RGB_C_ROJO);
}

void tarea_RevisarComandos_Loop(void)
{
    if (USB_RxFlag)
        {
            if (strstr ((char*) USB_Rx_Buf, CMD_INICIAR))
                {
                    rgbSetLedColor (&led1, RGB_C_VERDE);
                    //rgbSetLedColor (&led2, RGB_C_CYAN);
                    flag_MSV_graficar = true;
                    ads_Start ();
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_DETENER))
                {
                    rgbSetLedColor (&led1, RGB_C_ROJO);
                    //rgbSetLedColor (&led2, RGB_C_MAGENTA);
                    flag_MSV_graficar = false;
                    ads_Stop ();
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_INFO))
                {
                    rgbSetLedColor (&led1, RGB_C_AZUL);
                    flag_show_info = true;
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_SEL_GRAFICAS))
                {
                    rgbSetLedColor (&led1, RGB_C_CYAN);
                    derivacionesAGraficar = selectGraph ((char*) USB_Rx_Buf);
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_SEL_FILTRO))
                {
                    //indiceComandoRecibido = 4;
                    rgbSetLedColor (&led1, RGB_C_MAGENTA);
                    //Obtener el valor de Gx
                    char *cadena = (char*) USB_Rx_Buf;

                    int8_t pos_indice = findStr (cadena,
                    CMD_SEL_FILTRO); //Posición de la F
                    int8_t pos_coma = findStr (cadena, ",");
                    uint8_t g_val = (uint8_t) strtoul ((char*) &cadena[pos_indice + 2], NULL, 10);
                    //Configurar gráfica 1
                    if (g_val == 1)
                        {
                            if ((char) cadena[pos_coma + 1] == 'N')
                                {
                                    derivacionesAGraficar.ecg_FiltroG1 = FILTRO_NULL;
                                }
                            else if ((char) cadena[pos_coma + 1] == 'D')
                                {
                                    derivacionesAGraficar.ecg_FiltroG1 = FILTRO_DIAGNOSTICO;
                                }
                            else if ((char) cadena[pos_coma + 1] == 'M')
                                {
                                    derivacionesAGraficar.ecg_FiltroG1 = FILTRO_MONITOR;
                                }
                            else if ((char) cadena[pos_coma + 1] == 'A')
                                {
                                    derivacionesAGraficar.ecg_FiltroG1 = FILTRO_MAXIMO;
                                }
                        }
                    //Configurar gráfica 2
                    else if (g_val == 2)
                        {
                            if ((char) cadena[pos_coma + 1] == 'N')
                                {
                                    derivacionesAGraficar.ecg_FiltroG2 = FILTRO_NULL;
                                }
                            else if ((char) cadena[pos_coma + 1] == 'D')
                                {
                                    derivacionesAGraficar.ecg_FiltroG2 = FILTRO_DIAGNOSTICO;
                                }
                            else if ((char) cadena[pos_coma + 1] == 'M')
                                {
                                    derivacionesAGraficar.ecg_FiltroG2 = FILTRO_MONITOR;
                                }
                            else if ((char) cadena[pos_coma + 1] == 'A')
                                {
                                    derivacionesAGraficar.ecg_FiltroG2 = FILTRO_MAXIMO;
                                }
                        }
                }
            // Comandos de apnea
            else if (strstr ((char*) USB_Rx_Buf, CMD_APNEA_ON))
                {
                    //indiceComandoRecibido = 5;
                    rgbSetLedColor (&led1, RGB_C_AMARILLO);
                    char *cadenaA = (char*) USB_Rx_Buf;
                    int8_t pos_ind = findStr (cadenaA, "C");
                    int8_t tiempo_alarm = (int8_t) strtoul ((char*) &cadenaA[pos_ind + 2], NULL, 10);
                    flag_activa_apnea = true;
                    valor_apnea = tiempo_alarm;
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_APNEA_OFF))
                {
                    //indiceComandoRecibido = 6;
                    rgbSetLedColor (&led1, RGB_C_AMARILLO);
                    flag_activa_apnea = false;
                }
            // Comando para tipo de paciente
            else if (strstr ((char*) USB_Rx_Buf, CMD_TIPO_PACIENTE))
            {
            	char *cadenaA = (char*) USB_Rx_Buf;
            	int8_t pos_ind = findStr (cadenaA, "T");
            	if ((char) cadenaA[pos_ind + 2] == 'A')		// Adulto
            		paciente_tipo = 0;
            	else if ((char) cadenaA[pos_ind + 2] == 'P')		// pediatrico
            		paciente_tipo = 1;
            	else if ((char) cadenaA[pos_ind + 2] == 'N')		// Neonato
            		paciente_tipo = 2;
            }
            ////	COMANDOS PARA ELEJIR ELECTRODO DE RESPIRACION
            else if (strstr ((char*) USB_Rx_Buf, CMD_REP_LL))
            {
            	HAL_GPIO_WritePin(IN_1_GPIO_Port,IN_1_Pin,GPIO_PIN_RESET);
            }
            else if (strstr ((char*) USB_Rx_Buf, CMD_REP_LA))
            {
            	HAL_GPIO_WritePin(IN_1_GPIO_Port,IN_1_Pin, GPIO_PIN_SET);
            }
            // Comandos para neopixel
            else if (strstr ((char*) USB_Rx_Buf, CMD_NEOPIXEL_PALTA_ROJO))
                {
                    rgbSetLedColor (&led1, RGB_C_BLANCO);
                    prioridadNeopixel = P_ALTA;
                    modoLeds = MODO_PARPADEO;
                    colorLeds = C_ROJO;
                    blink_ON = true;
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_NEOPIXEL_PMEDIA_AMARILLO))
                {
                    rgbSetLedColor (&led1, RGB_C_BLANCO);
                    prioridadNeopixel = P_MEDIA;
                    modoLeds = MODO_PARPADEO;
                    colorLeds = C_AMARILLO;
                    blink_ON = true;
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_NEOPIXEL_PBAJA_BLANCO))
                {
                    rgbSetLedColor (&led1, RGB_C_BLANCO);
                    prioridadNeopixel = P_BAJA;
                    modoLeds = MODO_PARPADEO;
                    colorLeds = C_BLANCO;
                    blink_ON = true;
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_NEOPIXEL_PMEDIA_AZUL))
                {
                    rgbSetLedColor (&led1, RGB_C_BLANCO);
                    prioridadNeopixel = P_MEDIA;
                    modoLeds = MODO_PARPADEO;
                    colorLeds = C_AZUL;
                    blink_ON = true;
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_NEOPIXEL_SOLIDO_ROJO))
                {
                    rgbSetLedColor (&led1, RGB_C_BLANCO);
                    prioridadNeopixel = P_MEDIA;
                    modoLeds = MODO_SOLIDO;
                    colorLeds = C_ROJO;
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_NEOPIXEL_SOLIDO_AMARILLO))
                {
                    rgbSetLedColor (&led1, RGB_C_BLANCO);
                    prioridadNeopixel = P_MEDIA;
                    modoLeds = MODO_SOLIDO;
                    colorLeds = C_AMARILLO;
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_NEOPIXEL_SOLIDO_VERDE))
                {
                    rgbSetLedColor (&led1, RGB_C_BLANCO);
                    prioridadNeopixel = P_MEDIA;
                    modoLeds = MODO_SOLIDO;
                    colorLeds = C_VERDE;
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_NEOPIXEL_SOLIDO_NEGRO))
                {
                    rgbSetLedColor (&led1, RGB_C_BLANCO);
                    prioridadNeopixel = P_MEDIA;
                    modoLeds = MODO_SOLIDO;
                    colorLeds = C_NEGRO;
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_NEOPIXEL_SOLIDO_BLANCO))
                {
                    rgbSetLedColor (&led1, RGB_C_BLANCO);
                    prioridadNeopixel = P_MEDIA;
                    modoLeds = MODO_SOLIDO;
                    colorLeds = C_BLANCO;
                }
            else if (strstr ((char*) USB_Rx_Buf, CMD_PING))
                {
                    flag_Ping = true;
                }
            memset (USB_Rx_Buf, 0x00, USB_Rx_Len); //Limpiar lo escrito en el buffer
            USB_RxFlag = false;
        }
    osDelay (TIME_IDLE);
}
