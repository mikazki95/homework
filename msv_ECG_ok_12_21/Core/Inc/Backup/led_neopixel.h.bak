/*
 * led_neopixel.h
 *
 *  Created on: 22 may 2023
 *      Author: Ing. Alejandra Sakura Bautista Ambrocio.
 *
 *  Esta librería se encarga del manejo de la tira de leds RGB neopixel por uso de un Timer y
 *  su acceso mediante DMA para cambiar los valores del contador a los valores de tiempo requeridos
 *  por el chip neopixel.
 *
 *  Requisitos previos:
 *      * Habilitar un timer
 *      * Clock source: Internal clock
 *      * Elegir un canal del timer que tenga PWM Generation
 *      * Habilitar en NVIC DMA global interrupt
 *      * En DMA
 *          Agregar el canal del timer a un DMA stream
 *          Dirección Memory to peripheral
 *          Prioridad Alta
 *      * DMA request setting:
 *          Modo normal
 *          Increment Address: Memory
 *          Data width: Word Word
 *      * Ajustar el callback del timer y el canal:
              void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
               {
                 HAL_TIM_PWM_Stop_DMA (htim, TIM_CHANNEL_1);
               }
 *
 * Requisitos de funcionamiento
 *  * Declarar una instancia neoPixel_Instancia_t Que tendrá toda la información para el manejo de la tira de leds
 *  * Declarar un array neoPixel_ColorData_t de tamaño igual a la cantidad de leds de la tira.
 *  * Declarar un array para el buffer DMA, éste debe tener un tamaño [(#leds * 24)+1]
 *  * Inicializar la tira con la información pertinente.
 *  * Cargar el color a mostrar en la tira.
 *  * Mostrar el color almacenado en la memoria
 *
 * Ej:
 *   //////////////////////////////////////////////////////////////////////////////////////////////////////
     // Ir encendiendo la tira poco a poco hasta endencer todos los leds
     // desde 0x00000000 hasta 0x00ffffff

     #define NLEDS 11
     #define NDMA  (NLEDS*24)+1
     extern TIM_HandleTypeDef htim2;                  // Timer previamente configurado
     neoPixel_Instancia_t tiraDeLedsNeopixel;         // Instancia de la tira
     neoPixel_ColorData_t npColorData[NLEDS] = { 0 }; // Buffer de los datos de la tira led
     uint32_t dmaBuffer[NDMA] = { 0 };                // Cálculo de los valores requeridos para el DMA para la tira led

     main()
     {
      neopixel_Init(&tiraDeLedsNeopixel, &htim2, TIM_CHANNEL_1, NLEDS, npColorData, dmaBuffer);
      uint32_t colorTira = 0;
      for(;;)
      {
       if(colorTira > 0x00ffffff)
       {
        colorTira = 0x0;
       }
       neopixel_LoadColor (tiraDeLedsNeopixel.colorData, colorTira, tiraDeLedsNeopixel.ledQuantity); //Mostrar el x00ffffff (blanco al máximo)
       neopixel_ShowLeds(&tiraDeLedsNeopixel);
       colorTira++;
       osDelay(1);
      }
     }
   //////////////////////////////////////////////////////////////////////////////////////////////////////
 *
 * Lista de cambios:
 *   1 - Primer versión estable
    *
 */

#ifndef INC_LED_NEOPIXEL_H_
#define INC_LED_NEOPIXEL_H_

#define LIB_VERSION_NEOPIXEL 1

#define NEOPIXEL_ZERO 33
#define NEOPIXEL_ONE 67

// Dato de color (1 pixel)
typedef union
{
    struct
    {
        uint8_t b;
        uint8_t r;
        uint8_t g;
    } color;
    uint32_t data;
} neoPixel_ColorData_t;
// Instancia. Representa una tira de leds
typedef struct{
    TIM_HandleTypeDef    * htim;         // Apuntador al timer que utiliza la tira de leds
    uint32_t               timerChannel; // Canal del timer a utilizar
    uint32_t               ledCount;  // Cantidad de leds de la tira (tamaño de colorData[])
    neoPixel_ColorData_t * led;    // Apuntador al buffer con los datos de color de la tira
    uint32_t               dmaBuffSize;  // Tamaño del bufffer DMA
    uint32_t             * dmaBuffer;    // Apuntador al buffer DMA
} neoPixel_Instancia_t;
// Colores básicos de pixel (brillo al 100%)
typedef enum
{
    C_NEGRO = 0x00000000u,
    C_AZUL = 0x000000ffu,
    C_ROJO = 0x0000ff00u,
    C_MAGENTA = 0x0000ffffu,
    C_VERDE = 0x00ff0000u,
    C_CYAN = 0x00ff00ffu,
    C_AMARILLO = 0x00ffff00u,
    C_BLANCO = 0X00ffffffu
} neoPixel_Colors_t;
// Modos de encendido de la tira
typedef enum
{
    MODO_NULO = 0u, // Apagada
    MODO_SOLIDO,    // Color sólido
    MODO_PARPADEO   // Encendido intermitente
} neoPixel_RutinaModo_t;
// Tiempos de prioridad para las alarmas intermitentes
typedef enum
{
    P_ALTA =  250 - 1,  //1 parpadeo cada 500ms
    P_MEDIA = 650 - 1, //1 Parpadeo cada 1 segundo
    P_BAJA = 1300 - 1  //1 Parpadeo cada 2 segundos
} neoPixel_RutinaPrioridad_t;

/* @ Descripción: Inicializar los valores de una tira de leds. Se requiere de definir un array
 *                "neoPixel_ColorData_t" del tamaño de la tira, y un buffer de datos para el DMA,
 *                el tamaño de este segundo array debe ser (número de pixeles * 24) + 1
 * @ Retorna:     Nada
 * @ Parámetros:
 *                pInstanciaNeopixel: Puntero a la estructura con toda la información necesaria
 *                                    de la tira de leds.
 *                pTimerNeopixel:     Timer que se utilizará para enviar los datos a la tira.
 *                uCantidadLeds:      Cantidad de leds de la tira.
 *                pColorDataBuff:     Puntero al array con la indormación de color de la tira
 *                pDmaBuffer:         Puntero al array de datos para el DMA.
 * */
void neopixel_Init (neoPixel_Instancia_t *pInstanciaNeopixel,
                    TIM_HandleTypeDef *pTimerNeopixel,
                    uint32_t canalTimer,
                    uint32_t uCantidadLeds,
                    neoPixel_ColorData_t *pColorDataBuff,
                    uint32_t *pDmaBuffer);

/* @ Descripción: Carga un valor de color a uno o varios leds contiguos.
 * @ Retorna:     Nada
 * @ Parámetros:
 *                colorDataArray: Apuntador al buffer con la información de color de la tira.
 *                colorPixel:     Valor de color RGB (orden BGR) de 32 bits
 *                numPixels:      Cantidad de leds a los cuales asignarles el color
 * */
void neopixel_LoadColor (neoPixel_ColorData_t *colorDataArray, uint32_t colorPixel, uint32_t numPixels);

/* @ Descripción: (Des)activa los leds de la tira conforme a la programación realizada
 *                por neopixel_LoadColor().
 * @ Retorna:     Nada
 * @ Parámetros:
 *                pInstanciaNeopixel: Puntero a la estructura con toda la información necesaria
 *                                    de la tira de leds.
 *                */
void neopixel_ShowLeds (neoPixel_Instancia_t *pInstanciaNeopixel);


#endif /* INC_LED_NEOPIXEL_H_ */
