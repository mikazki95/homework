/*
 * led_rgb_simple.c
 *
 *  Created on: May 16, 2023
 *      Author: sakura
 */

#include "main.h"
#include "led_rgb_simple.h"


void rgbSetLedColor (ledRGB_Instancia_t *led, ledRGB_Colors_t color)
{
    led->R = (0b100 & color) >> 2;
    led->G = (0b010 & color) >> 1;
    led->B = 0b001 & color;
    HAL_GPIO_WritePin (led->PuertoR, led->PinR, !led->R);
    HAL_GPIO_WritePin (led->PuertoG, led->PinG, !led->G);
    HAL_GPIO_WritePin (led->PuertoB, led->PinB, !led->B);
}
