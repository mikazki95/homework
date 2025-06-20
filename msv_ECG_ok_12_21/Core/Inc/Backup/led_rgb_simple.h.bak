/*
 * led_rgb_simple.h

 *  Created on: May 16, 2023
 *      Author: Ing. Alejandra Sakura Bautista Ambrocio
 * Description: Librería con las funciones básicas del led rgb simple. Se
 *              desarrolla para liberar a la tarea principal y mejorar la
 *              comprensión del software.
 *
 * Lista de cambios:
 *   1 - Primer versión estable
 */

#ifndef INC_LED_RGB_SIMPLE_H_
#define INC_LED_RGB_SIMPLE_H_
/* Includes ------------------------------------------------------------------*/

/* Typedef -------------------------------------------------------------------*/

/* Estructura representativa de un ledRGB:
 *   Puerto y pin de cada led individual y su estado.
 */
typedef struct
{
    GPIO_TypeDef *PuertoR;
    uint16_t PinR;
    GPIO_TypeDef *PuertoG;
    uint16_t PinG;
    GPIO_TypeDef *PuertoB;
    uint16_t PinB;
    bool R;
    bool G;
    bool B;
} ledRGB_Instancia_t;

/* Combinaciones posibles de los 3 leds individuales del led RGB
 */
typedef enum // ComLeds RGB que tiene la tarjeta
{
    RGB_C_NULL,     // 000 Todos apagados
    RGB_C_AZUL,     // 001
    RGB_C_VERDE,    // 010
    RGB_C_CYAN,     // 011
    RGB_C_ROJO,     // 100
    RGB_C_MAGENTA,  // 101
    RGB_C_AMARILLO, // 110
    RGB_C_BLANCO,   // 111 Todos encendidos
} ledRGB_Colors_t;



/* Define --------------------------------------------------------------------*/
#define LIB_VERSION_RGBSIMPLE 1
/* Macro ---------------------------------------------------------------------*/
/* Variables -----------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void rgbSetLedColor (ledRGB_Instancia_t *led, ledRGB_Colors_t color);
/* Private application code --------------------------------------------------*/


#endif /* INC_LED_RGB_SIMPLE_H_ */
