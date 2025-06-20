/*
 * ads1298.h
 *
 *  Created on: May 16, 2023
 *      Author: Ing. Alejandra Sakura Bautista Ambrocio
 * Description: Librería con las funciones básicas del ADS1298R. Se desarrolla
 *              para liberar a la tarea principal y mejorar la comprensión del
 *              software.
 *
 *              Estamos usando el timer 13 para generar delay de mucrosegundos
 *              (delay_uSec()).
 *              Requiere activar el timer con HAL_TIM_Base_Start (&htim13)
 *              antes de poder usarse.
 *
 * Lista de cambios:
 *   1 - Primer versión estable
 */

#ifndef INC_ADS1298_H_
#define INC_ADS1298_H_

/* Includes ------------------------------------------------------------------*/

/* Typedef -------------------------------------------------------------------*/

typedef enum
{
    ADS_CMD_WAKEUP = 0x02,
    ADS_CMD_STANDBY = 0x04,
    ADS_CMD_RESET = 0x06,
    ADS_CMD_START = 0x08,
    ADS_CMD_STOP = 0x0A,
    ADS_CMD_RDATAC = 0x10,
    ADS_CMD_SDATAC = 0x11,
    ADS_CMD_RDATA = 0x12
} ads_Comandos_t;
typedef enum
{
    ADS_REG_ID,
    ADS_REG_CONFIG1,
    ADS_REG_CONFIG2,
    ADS_REG_CONFIG3,
    ADS_REG_LOFF,
    ADS_REG_CH1SET,
    ADS_REG_CH2SET,
    ADS_REG_CH3SET,
    ADS_REG_CH4SET,
    ADS_REG_CH5SET,
    ADS_REG_CH6SET,
    ADS_REG_CH7SET,
    ADS_REG_CH8SET,
    ADS_REG_RLD_SENSP,
    ADS_REG_RLD_SENSN,
    ADS_REG_LOFF_SENSP,
    ADS_REG_LOFF_SENSN,
    ADS_REG_LOFF_FLIP,
    ADS_REG_LOFF_STATP,
    ADS_REG_LOFF_STATN,
    ADS_REG_GPIO,
    ADS_REG_PACE,
    ADS_REG_RESP,
    ADS_REG_CONFIG4,
    ADS_REG_WCT1,
    ADS_REG_WCT2,
    N_CONFIG_REGS
} ads_Registros_t;

/* Define --------------------------------------------------------------------*/

#define LIB_VERSION_ADS  1

#define ADS_RREG_MASK  0b00100000
#define ADS_WREG_MASK  0b01000000
#define ADS_TCLK_4     6  // Tclk a 2.048 MHz (488.28 ns) -> 2us (1.95313 us)
#define ADS_TCLK_18    40 // Tclk a 2.048 MHz (488.28 ns) -> 9us (8.78906 us)
#define ADS_MULT_OUT   1000.0 //Multiplicador de voltaje calculado 1000 mV, 1000000 nV, etc.
#define ADS_RESOLUTION 8388607.0 //Resolución del ADS1298R
#define ADS_BUFF_RX    27 //La lectura es de 27 bytes, voy a realizar una doble lectura para verificar el dato leído
#define ADS_BUFF_TX    30

/* Macro ---------------------------------------------------------------------*/

#define adsCmd_pinStart_HIGH() HAL_GPIO_WritePin (SPI1_START_GPIO_Port, SPI1_START_Pin, GPIO_PIN_SET)
#define adsCmd_pinStart_LOW() HAL_GPIO_WritePin (SPI1_START_GPIO_Port, SPI1_START_Pin, GPIO_PIN_RESET)
#define adsCmd_wakeup() ads_SendSimpleCommand(ADS_CMD_WAKEUP)
#define adsCmd_standby() ads_SendSimpleCommand(ADS_CMD_STANDBY)
#define adsCmd_reset() ads_SendSimpleCommand(ADS_CMD_RESET)
#define adsCmd_start() ads_SendSimpleCommand(ADS_CMD_START)
#define adsCmd_stop() ads_SendSimpleCommand(ADS_CMD_STOP)

/* Variables -----------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void adsCmd_rdatac (void);
void adsCmd_sdatac (void);
void adsCmd_rdata (void);
void adsCmd_rreg (ads_Registros_t regInit, uint8_t nRegs);
void adsCmd_wreg (ads_Registros_t regInit, uint8_t nRegs);
void adsCmd_end (uint16_t dly);
void ads_SendSimpleCommand (ads_Comandos_t cmd);
void ads_start_by_pin (void);
void leerDatosAds (void);
void lecturaDatosAds (void);
void lecturaContinuaDatosAds (void);
int32_t leerValorCanal (uint32_t IndiceCanal);
void ads_Init(void);
void ads_Start(void);
void ads_Stop(void);
void ads_reset_byPin(void);
/*
 * Establece la bandera de interrupción, colocar en el callback
 * HAL_GPIO_EXTI_Callback()
 * */
void ads_IntTrue(void);
void ads_IntFalse(void);
bool ads_IntStat(void);

void delay_us_tim13 (uint16_t us);

/* Private application code --------------------------------------------------*/

#endif /* INC_ADS1298_H_ */
