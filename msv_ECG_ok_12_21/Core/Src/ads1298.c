/*
 * ads1298.c
 *
 *  Created on: May 16, 2023
 *      Author: Ing. Alejandra Sakura Bautista Ambrocio
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <arm_math.h>
#include "usbd_cdc_if.h"

#include "ads1298.h"
/* Typedef -------------------------------------------------------------------*/
typedef union
{
    struct
    {
        uint8_t b2;
        uint8_t b1;
        uint8_t b0;
    } bytes;
    uint32_t data;
} ads_ChannelData_T;
/* Define --------------------------------------------------------------------*/
/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim13;


//Error debug
extern uint32_t error_cnt;

//Bubber de recepción y buffer de transmisión
static uint8_t ads_bufferRx[ADS_BUFF_RX] = { 0 };
uint8_t ads_bufferTx[ADS_BUFF_TX] = { 0 };


bool flag_ADS_DRDY = false;
bool flag_ADS_Write_Cplt = false; // Indica si se han transmitido los datos al ADS
bool flag_ADS_Reading = false; // Indica que ya se han recibido los datos del ADS

bool ads_is_rdatac = true; // En reset, el ADS comienza en RDATAC por default en el ADS



/* Private function prototypes -----------------------------------------------*/

/* Private application code --------------------------------------------------*/
//Callback de escritura SPI
void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
        {
            flag_ADS_Write_Cplt = true;
        }
}
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
        {
//            if (flag_ADS_Reading) // Si fue recepción por esperar lectura, proceso este callback
//                {
                    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); // Liberar el SPI del ADS
                    flag_ADS_Reading = false;
//                }
        }
}
void adsCmd_rdatac (void)
{
    ads_SendSimpleCommand (ADS_CMD_RDATAC);
    ads_is_rdatac = true;
}
void adsCmd_sdatac (void)
{
    ads_SendSimpleCommand (ADS_CMD_SDATAC);
    ads_is_rdatac = false;
}

void adsCmd_rdata (void) // Modo polling
{
    if (ads_is_rdatac) // Requiere SDATAC
        {
            adsCmd_sdatac ();
        }
    uint8_t comando = ADS_CMD_RDATA;
    //uint8_t disparo = 1;
    //while (disparo == 1)
    while (!flag_ADS_DRDY)
        { // Esperar que DRDY=L
            //disparo = HAL_GPIO_ReadPin (SPI1_DRDY_GPIO_Port, SPI1_DRDY_Pin);
        }
    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit (&hspi1, &comando, 1, 100);
    leerDatosAds ();

}

void adsCmd_rreg (ads_Registros_t regInit, uint8_t nRegs)
{ // 1er byte 001rrrrr, 2º byte 000nnnnn
    uint8_t comando[2];
    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    comando[0] = (regInit & 0b00011111) | ADS_RREG_MASK; // mantener los bytes r y convinarlos con la máscara de comando
    if (nRegs == 0)
        {
            nRegs = 1; // siempre leer mínimo 1 registro
        }
    comando[1] = (nRegs & 0b00011111) - 1; // mantener los bytes n
    HAL_SPI_Transmit (&hspi1, comando, 2, 100);
}
void adsCmd_wreg (ads_Registros_t regInit, uint8_t nRegs)
{ // 1er byte 001rrrrr, 2º byte 000nnnnn
    uint8_t comando[2];
    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    comando[0] = (regInit & 0b00011111) | ADS_WREG_MASK; // mantener los bytes r y convinarlos con la máscara de comando
    if (nRegs == 0)
        {
            nRegs = 1; // mínimo 1 registro a escribir
        }
    comando[1] = (nRegs & 0b00011111) - 1; // mantener los bytes n
    HAL_SPI_Transmit (&hspi1, comando, 2, 100);
}
void adsCmd_end (uint16_t dly)
{
    delay_us_tim13 (dly);
    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}
void ads_SendSimpleCommand (ads_Comandos_t cmd)
{
    uint8_t comando = cmd;
    uint16_t delay;
    if (comando == ADS_CMD_RESET)
        {
            delay = ADS_TCLK_18;
        }
    else
        {
            delay = ADS_TCLK_4;
        }
    adsCmd_pinStart_LOW();
    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    flag_ADS_Write_Cplt = false;
    HAL_SPI_Transmit_IT (&hspi1, &comando, 1);
    while (!flag_ADS_Write_Cplt)
        {
        }
    adsCmd_end (delay);
}
void ads_start_by_pin (void)
{
    HAL_GPIO_WritePin (SPI1_START_GPIO_Port, SPI1_START_Pin, GPIO_PIN_SET);
    delay_us_tim13 (ADS_TCLK_4);
    HAL_GPIO_WritePin (SPI1_START_GPIO_Port, SPI1_START_Pin, GPIO_PIN_RESET);
}

void leerDatosAds (void)
{
    memset (ads_bufferRx, 0x00, ADS_BUFF_RX);
    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    flag_ADS_Reading = true;
    HAL_SPI_Receive_IT (&hspi1, ads_bufferRx, ADS_BUFF_RX);
    while (flag_ADS_Reading)
        {
        }
}
void lecturaDatosAds (void)
{
    memset (ads_bufferRx, 0x00, ADS_BUFF_RX); // Borrar el buffer de lectura ADS (Evitar enviar datos de comandos)
    ads_start_by_pin (); // Inicar lectura
    while (!flag_ADS_DRDY) // Esperar la interrupción
        {
        }
    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); // Habilitar la lectura del ADS
    flag_ADS_Reading = true; // Indicar al callback del SPI, que si espero una lectura de datos
    HAL_SPI_Receive_IT (&hspi1, ads_bufferRx, ADS_BUFF_RX); // Recibir los datos
    uint32_t time = HAL_GetTick (); // Anotar el tiempo actual para evitar cuelgue en esta parte (en el debug vi que esta parte tendía a bloquearse)
    while (flag_ADS_Reading)
        {
            if ((HAL_GetTick () - time) > 10)
                { //SOLO ESPERO 10 ms máximo
                    flag_ADS_Reading = false;
                }
        }
}
void lecturaContinuaDatosAds (void)
{
    // Identificar si hubo error en la lectura del ADS
    bool errorLectura = false;

    //Respaldo del dato anterior en el buffer de recepción.
    //En caso de error, reenviar el último dato válido
    uint8_t ads_bufferRx_Resp[ADS_BUFF_RX];
    for (uint8_t i = 0; i < ADS_BUFF_RX; i++)
        {
            ads_bufferRx_Resp[i] = ads_bufferRx[i];
        }

    // Borrar el buffer de lectura ADS (Evitar enviar datos de comandos)
    memset (ads_bufferRx, 0x00, ADS_BUFF_RX);
    // Habilitar la lectura del ADS
    HAL_GPIO_WritePin (SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    // Indicar al callback del SPI, que espero una lectura de datos
    flag_ADS_Reading = true;
    // Recibir los datos
    HAL_SPI_Receive_IT (&hspi1, ads_bufferRx, ADS_BUFF_RX);
    // En caso de que la lectura no se logre, no bloquearme en esta tarea.
    // Además la tarea de Lectura ADS requiere del valor leído del ADS
    uint32_t time = HAL_GetTick ();
    while (flag_ADS_Reading)
        {
            if ((HAL_GetTick () - time) > 1)
                { //SOLO ESPERO 2 ms máximo y regreso el dato salvado previamente
                    errorLectura = true;

//                    for (uint8_t i = 0; i < ADS_BUFF_RX; i++)
//                        {
//                            ads_bufferRx[i] = ads_bufferRx_Resp[i];
//                        }
                    break;
                }
        }
    // Acá ya se leyó el ADS, ahora hay que validar lo leído
    // Los 27 bytes leídos deben comenzar con 1100 y los canales no usados deben contener 0's
    // |           STATUS WORD              | RESP       | DERIVADA I | DERIVADA II|    NULL    |    NULL    |    NULL    |    NULL    |     V1     |
    // |             3 bytes                | 3Bytes CH1 | 3Bytes CH2 | 3Bytes CH3 | 3Bytes CH4 | 3Bytes CH5 | 3Bytes CH6 | 3Bytes CH7 | 3Bytes CH8 |
    // |1100 <statP[8]> <statN[8]> <GPIO[4]>| 0x......   | 0x......   | 0x......   | 0x000000   | 0x000000   | 0x000000   | 0x000000   | 0x......   |
    uint8_t bi0 = ads_bufferRx[0] >> 4;
    uint8_t bytes12_23 = 0;

    bytes12_23 = ads_bufferRx[12] | ads_bufferRx[13] | ads_bufferRx[14] // Algo en CH4
               | ads_bufferRx[15] | ads_bufferRx[16] | ads_bufferRx[17] // Algo en CH5
               | ads_bufferRx[18] | ads_bufferRx[19] | ads_bufferRx[20] // Algo en CH6
               | ads_bufferRx[21] | ads_bufferRx[22] | ads_bufferRx[23]; // Algo en CH7

    // Encabezado mal || Algo en los bytes 12:23
    if((bi0 != 0x0c) || (bytes12_23) // Algo en CH4
       )
        {
            errorLectura = true;

        }

    if(errorLectura) //En caso de error devolver el dato anterior
        {
            error_cnt++;
            for (uint8_t i = 0; i < ADS_BUFF_RX; i++)
                {
                    ads_bufferRx[i] = ads_bufferRx_Resp[i];
                }
        }

}
int32_t leerValorCanal (uint32_t IndiceCanal) // Devuelve un valor de 24 bits. Canales desde 0 hasta 8. 0 -> palabra de estado
{
    // Comenzamos con valores limpios para asegurar que no se corrompan con ruido
    ads_ChannelData_T channelData;    // Dónde se procesará el dato leído
    uint32_t indice = 0;              // Dónde comienza el dato del canal
    //uint8_t *pADSMsg = ads_bufferRx;  // Puntero buffer RX del ADS
    int32_t valor_final = 0;          // Valor calculado

    // Variable de valor leído inicializada en 0
    channelData.data = 0;
    //Posición del byte a leer
    indice = IndiceCanal * 3;
    //Copiar los bytes del canal a la variable temporal
    channelData.bytes.b0 = ads_bufferRx[indice];
    channelData.bytes.b1 = ads_bufferRx[indice + 1];
    channelData.bytes.b2 = ads_bufferRx[indice + 2];

    //Canal 2 Derivación I
//    if((IndiceCanal == 2) &&  (channelData.data > 0x1E848)) //mayor a 125000
//            {
//                bool flag;
//                flag = true;
//            }

    if (IndiceCanal) // al canal cero no se le aplica la conversión de números
        { // Canales del 1 al 8 Máximo 0x7FFFFF, Negativos800000
            if (channelData.data & 0x00800000) // 0x800000 es la máscara de negativos el último bit es el indicador de negativo
                {
                    channelData.data |= 0xFF800000; //Rellenar el 4bit con '1'
                }
        }
    valor_final |= channelData.data;


    //Acá estaba todo junto por simplicidad
//    int32_t valor_final = ((ads_bufferRx[indice] << 16) | (ads_bufferRx[indice + 1] << 8) | (ads_bufferRx[indice + 2])) & 0x00ffffff; // Solo conservar los 3bytes útiles
//    if (IndiceCanal) // al canal cero no se le aplica la conversión de números
//        { // Canales del 1 al 8 Máximo 0x7FFFFF, Negativos800000
//            if (valor_final & 0x800000) // 0x800000 es la máscara de negativos el último bit es el indicador de negativo
//                {
//                    valor_final = valor_final | 0xFF800000; //Rellenar el 4bit con '1' para poder invertir el valor adecuadamente
//                    valor_final ^= 0xFFFFFFFF; // Volvemos el valor a positivo de 32bits
//                    valor_final = (valor_final) * (-1); // Lo volvemos negativo de 32 bits (valor era de 32bits con signo)
//                }
//        }



    return valor_final;
}

/* Retardos para este Init:
 * POR 150ms. El ADS1298R tiene un POR de 2^18 tclk -> 128 ms (150ms typ).
 * Wakeup 9ms. Si ya están todas las alimentaciones y se usa el pin PWDN.
 * Treset 977ns. Si se ejecuta esta función por un comando reset o pin reset
 * no requiere retardo ya que las funciones internas ya tienen los tiempos requeridos
 */
void ads_Init(void)
{
    ads_reset_byPin();
    //delay_us_tim13 (1);            // Retardo mínimo para el reset por pin
    // Preparar la configuración del ADS
    //ads_bufferTx[0] = 0b10000110;  // CONFIG1 V1.3.0+ //HR 500SPS
    ads_bufferTx[0] = 0b00000101;  // CONFIG1 V1.3.0+ //LP 500SPS
    ads_bufferTx[1] = 0b00010000;  // CONFIG2
    ads_bufferTx[2] = 0b11001110;  // CONFIG3 |1->en_int_ref_buf|1|0->vref=2.4v|0->rld_meas=Open|1->rld_ref=internal|1->rld_buff=on|1->rld_sense=enabled|x->soloLectura=rld_stat|
    ads_bufferTx[3] = 0b10101111;  // LOFF
    // Canales inicialmente apagados
    ads_bufferTx[4] = 0b10000001;  // CH1SET Gain [6:4] 000-6 001-1 010-2 011-3 100-4 101-8 110-12 Ganancia 4 (3 o 4 recomendada en datasheet)  0b11000001;
    ads_bufferTx[5] = 0b10000001;  // CH2SET Ganancia en 6
    ads_bufferTx[6] = 0b10000001;  // CH3SET
    ads_bufferTx[7] = 0b10000001;  // CH4SET Canal no usado
    ads_bufferTx[8] = 0b10000001;  // CH5SET Canal no usado
    ads_bufferTx[9] = 0b10000001;  // CH6SET Canal no usado
    ads_bufferTx[10] = 0b10000001; // CH7SET Canal no usado
    ads_bufferTx[11] = 0b10000001; // CH8SET
    // Revisión de electrodos conectados
    //ads_bufferTx[12] = 0b00000100; // RLD_SENSP
    ads_bufferTx[12] = 0b00000110; // RLD_SENSP
    ads_bufferTx[13] = 0b00000110; // RLD_SENSN
    ads_bufferTx[14] = 0b10000110; // LOFF_SENSP V1-CH8 LL-CH3 LA-CH2
    ads_bufferTx[15] = 0b00000010; // LOFF_SENSN RA Detectado en CH2
    ads_bufferTx[16] = 0b00000000; // LOFF_FLIP
    // Modo de lectura, respiración, marcapasos y extras
    ads_bufferTx[17] = 0b00000000; // GPIO
    ads_bufferTx[18] = 0b00000011; // PACE
    ads_bufferTx[19] = 0b11110010; // RESP
    ads_bufferTx[20] = 0b00100010; // CONFIG4 //V1.3.0+ NO SINGLE SHOT
    ads_bufferTx[21] = 0b00001011; // WCT1
    ads_bufferTx[22] = 0b11010100; // WCT2

    // Configurar ADS - Envío de datos (paquete 1)
    adsCmd_sdatac (); // SDATAC para poder configurar el ADS
    adsCmd_wreg (ADS_REG_CONFIG1, 17);
    HAL_SPI_Transmit (&hspi1, ads_bufferTx, 17, 100);
    adsCmd_end (ADS_TCLK_18);
    // Configurar ADS - Envío de datos (paquete 2)
    adsCmd_wreg (ADS_REG_GPIO, 6);
    HAL_SPI_Transmit (&hspi1, &ads_bufferTx[17], 6, 100);
    adsCmd_end (ADS_TCLK_18);
    adsCmd_sdatac (); // Detener lecturas continuas
}
void ads_Start (void)
{
    adsCmd_pinStart_LOW();
    flag_ADS_DRDY = false;
    //Activar ADS para lecturas
    adsCmd_sdatac (); // SDATAC para poder configurar el ADS
    ads_bufferTx[4] = 0b00000000; // CH1SET Gain [6:4] 000-6 001-1 010-2 011-3 100-4 101-8 110-12  0b01010000;
    ads_bufferTx[5] = 0b00000000; // CH2SET Ganancia 6
    ads_bufferTx[6] = 0b00000000; // CH3SET
    ads_bufferTx[7] = 0b10000001; // CH4SET Canal no usado
    ads_bufferTx[8] = 0b10000001; // CH5SET Canal no usado
    ads_bufferTx[9] = 0b10000001; // CH6SET Canal no usado
    ads_bufferTx[10] = 0b10000001; // CH7SET Canal no usado
    ads_bufferTx[11] = 0b00000000; // CH8SET
    adsCmd_wreg (ADS_REG_CH1SET, 8);
    HAL_SPI_Transmit (&hspi1, &ads_bufferTx[4], 8, 100);
    adsCmd_end (ADS_TCLK_18);
    adsCmd_sdatac (); // Detener lecturas continuas
    adsCmd_rdatac (); // Modo de lecturas contínuas (Solicitar lectura con el pin start)
    adsCmd_pinStart_HIGH();
}
void ads_Stop(void){
    adsCmd_pinStart_LOW();
    flag_ADS_DRDY = false;
    //Desactivar el ADS
    adsCmd_sdatac (); // SDATAC para poder configurar el ADS
    ads_bufferTx[4] = 0b10000001; // CH1SET Gain [6:4] 000-6 001-1 010-2 011-3 100-4 101-8 110-12   0b11010001;
    ads_bufferTx[5] = 0b10000001; // CH2SET
    ads_bufferTx[6] = 0b10000001; // CH3SET
    ads_bufferTx[7] = 0b10000001; // CH4SET Canal no usado
    ads_bufferTx[8] = 0b10000001; // CH5SET Canal no usado
    ads_bufferTx[9] = 0b10000001; // CH6SET Canal no usado
    ads_bufferTx[10] = 0b10000001; // CH7SET Canal no usado
    ads_bufferTx[11] = 0b10000001; // CH8SET
    adsCmd_wreg (ADS_REG_CH1SET, 8);
    HAL_SPI_Transmit (&hspi1, &ads_bufferTx[4], 8, 100);
    adsCmd_end (ADS_TCLK_18);
}
void ads_reset_byPin(void)
{
    HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_RESET);
    delay_us_tim13(6);
    HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_SET);
    delay_us_tim13(ADS_TCLK_18);
}

void ads_IntTrue(void){
    flag_ADS_DRDY = true;
}
void ads_IntFalse(void){
    flag_ADS_DRDY = false;
}
bool ads_IntStat(void)
{
    return flag_ADS_DRDY;
}
void delay_us_tim13 (uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim13, 0); // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim13) < us)
        {
        }
}
