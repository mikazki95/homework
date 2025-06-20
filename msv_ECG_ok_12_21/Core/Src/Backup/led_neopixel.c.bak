/*
 * led_neopixel.c
 *
 *  Created on: 22 may 2023
 *      Author: sakura
 */
#include "FreeRTOS.h"
#include "main.h"
#include "led_neopixel.h"

void neopixel_Init (neoPixel_Instancia_t * pInstanciaNeopixel, // Apuntador a la instancia de la tira de leds a inicializar
                    TIM_HandleTypeDef    * pTimerNeopixel,
                    uint32_t               canalTimer,
                    uint32_t               uCantidadLeds,
                    neoPixel_ColorData_t * pColorDataBuff,
                    uint32_t             * pDmaBuffer)
{
    pInstanciaNeopixel -> htim = pTimerNeopixel;
    pInstanciaNeopixel -> timerChannel = canalTimer;
    pInstanciaNeopixel -> ledCount = uCantidadLeds;
    pInstanciaNeopixel -> led = pColorDataBuff;
    pInstanciaNeopixel -> dmaBuffSize = (uCantidadLeds * 24) + 1;
    pInstanciaNeopixel -> dmaBuffer = pDmaBuffer;
}
void neopixel_LoadColor (neoPixel_ColorData_t *colorDataArray, uint32_t colorPixel, uint32_t numPixels)
{
    for (uint16_t i = 0; i < numPixels; i++)
        {
            colorDataArray[i].data = colorPixel;
        }
}
void neopixel_ShowLeds (neoPixel_Instancia_t *pInstanciaNeopixel)
{
    // Variable para salvar la dirección del puntero, mas seguro que usar el puntero por sí mismo
    uint32_t *pBuff;
    pBuff = pInstanciaNeopixel -> dmaBuffer;

    for (uint32_t i = 0; i < pInstanciaNeopixel -> ledCount; i++)
        {
            for (int j = 23; j >= 0; j--) //24 bits de información de color
                {
                    if ((pInstanciaNeopixel -> led[i].data >> j) & 0x01)
                        {
                            *pBuff = NEOPIXEL_ONE;
                        }
                    else
                        {
                            *pBuff = NEOPIXEL_ZERO;
                        }
                    pBuff++;
                }
        }
    // El último valor del DMA debe ser 0
    pInstanciaNeopixel -> dmaBuffer [pInstanciaNeopixel -> dmaBuffSize -1] = 0;

    // Ejecutar el timer con la información del buffer DMA
    HAL_TIM_PWM_Start_DMA (pInstanciaNeopixel -> htim,        // Timer
                           pInstanciaNeopixel -> timerChannel, // Canal
                           pInstanciaNeopixel -> dmaBuffer,    // Buffer DMA
                           pInstanciaNeopixel -> dmaBuffSize); // Tamaño del buffer
    //delay_us_tim13 (60); //Debe esperar al menos 60 us entre datos en la tira. Como de por sí no lo hacemos tan seguido, elimino esta pausa
}
