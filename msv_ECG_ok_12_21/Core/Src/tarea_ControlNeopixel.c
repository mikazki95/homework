/*
 * tarea_ControlNeopixel.c
 *
 *  Created on: May 23, 2023
 *      Author: sakura
 */

#include "main.h"
#include "cmsis_os.h"

#include "led_neopixel.h"
#include "tarea_ControlNeopixel.h"

const char versionControlNeopixel[]= LIB_VERSION_CONTROLNEOPIXEL;
// Características de la rutina
extern neoPixel_RutinaPrioridad_t prioridadNeopixel;
extern neoPixel_RutinaModo_t modoLeds;
extern neoPixel_Colors_t colorLeds;
extern bool blink_ON;

// Instancia de la tira led
neoPixel_Instancia_t tiraDeLedsNeopixel;
extern TIM_HandleTypeDef htim2;
neoPixel_ColorData_t neopixelColorData[NUM_PIXELS] = { 0 };
uint32_t neopixelDmaBuffer[DMA_BUFF_SIZE] = { 0 };

// Marca de tiempo
uint32_t timeActLeds;
bool flag_ActualizarLeds = false;

// Callback ajustado como indica la librería led_neopixel.h
void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
{
    if (htim->Instance == tiraDeLedsNeopixel.htim->Instance)
        {
            HAL_TIM_PWM_Stop_DMA (htim, tiraDeLedsNeopixel.timerChannel);
        }

}

// Función de la tarea5

void tarea_ControlNeopixel_Setup(void)
{
    neopixel_Init (&tiraDeLedsNeopixel, &htim2, TIM_CHANNEL_1, NUM_PIXELS, neopixelColorData, neopixelDmaBuffer);
        // Cargar leds al inicio arrancar el equipo, sirve como un test de la tira
        tiraDeLedsNeopixel.led[0].data = C_ROJO;
        tiraDeLedsNeopixel.led[1].data = C_VERDE;
        tiraDeLedsNeopixel.led[2].data = C_AZUL;
        tiraDeLedsNeopixel.led[3].data = C_CYAN;
        tiraDeLedsNeopixel.led[4].data = C_MAGENTA;
        tiraDeLedsNeopixel.led[5].data = C_AMARILLO;
        tiraDeLedsNeopixel.led[6].data = C_MAGENTA;
        tiraDeLedsNeopixel.led[7].data = C_CYAN;
        tiraDeLedsNeopixel.led[8].data = C_AZUL;
        tiraDeLedsNeopixel.led[9].data = C_VERDE;
        tiraDeLedsNeopixel.led[10].data = C_ROJO;
        neopixel_ShowLeds (&tiraDeLedsNeopixel);
        timeActLeds = HAL_GetTick();
}

void tarea_ControlNeopixel_Loop(void)
{
    if ((HAL_GetTick () - timeActLeds) > (prioridadNeopixel)) // Activar la bandera dependiendo de la prioridad de la alarma
        {
            timeActLeds = HAL_GetTick ();
            flag_ActualizarLeds = true;
        }
    if (flag_ActualizarLeds && modoLeds) //modoLeds Verdadero para MODO_SOLIDO y MODO_PARPADEO
        {
            flag_ActualizarLeds = false; // Limpiar bandera
            if (modoLeds == MODO_SOLIDO)
                {
                    neopixel_LoadColor (tiraDeLedsNeopixel.led, colorLeds, tiraDeLedsNeopixel.ledCount);
                    modoLeds = MODO_NULO;
                }
            else if (modoLeds == MODO_PARPADEO)
                {
                    if (blink_ON)
                        {
                            blink_ON = false; //Apagar los leds en la siguiente interrupción
                            neopixel_LoadColor (tiraDeLedsNeopixel.led, colorLeds, tiraDeLedsNeopixel.ledCount);
                        }
                    else
                        {
                            blink_ON = true; //Encender los leds en la siguiente interrupción
                            neopixel_LoadColor (tiraDeLedsNeopixel.led, C_NEGRO, tiraDeLedsNeopixel.ledCount);
                        }
                }
            neopixel_ShowLeds (&tiraDeLedsNeopixel);
        }
    else
        {
            osDelay (10);
        }
}
