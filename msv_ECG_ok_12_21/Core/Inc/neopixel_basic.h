///**
// ******************************************************************************
// * File Name          : neopixel_basic.h
// * Description        : manejo básico de neopixel
// ******************************************************************************
// *
// * Se requiere de 1 timer en modo PWM con salida en un canal X y acceso al
// * timer por llamadas de DMA
// * DMA request Memory To Peripheral Very High, increment address ->Memory
// *
// * BUS   TIMERS
// * APB1  TIM2, TIM3, TIM4, TIM5,  TIM12, TIM13, TIM14
// * APB2  TIM1, TIM8, TIM9, TIM10, TIM11
// *
// * TIM6 Y TIM7 No se de donde sale su reloj
// * 2 tipos de leds WS2812
// * Periodo del pulso PWM = 1.25 us
// * Valor cuenta 0 = 32%
// * Valor cuenta 1 = 64%
// *
// * LED    | Periodo             | Cuenta 0    | Cuenta 1    | Reset
// * WS2812 | 1.25 us (800KHz)    | 32%  0.4 us | 64%  0.8 us | L 50 us
// * WS6812 | 1.2 us  (833.33KHz) | 25%  0.3 us | 50%  0.6 us | L 80 us
// *
// ******************************************************************************
// */
//
//
//#include "FreeRTOS.h"
//#include "task.h"
//#include "main.h"
//#include "cmsis_os.h"
//#include "tim.h"
//
//// Instancia Led
//typedef union
//{
//  struct
//  {
//    uint8_t b;
//    uint8_t r;
//    uint8_t g;
//  } color;
//  uint32_t data;
//} PixelRGB_t;
//
//
//typedef struct{
//	PixelRGB_t * arrayPixeles; // Puntero del array de datos
//	uint32_t numPixeles;
//	uint32_t *arrayDma;
//	uint32_t sizeDma;
//	TIM_HandleTypeDef *htim;   // Puntero del timer a manejar
//	uint32_t channel;          // Canal de salida del timer
//	uint32_t cntCero;          // Valor de conteo 0 para el PWM
//	uint32_t cntUno;           // Valor de conteo 1 para el PWM
//}neoPixelRGB_t;
//
//
//
//#define WS2812_PERIODO 0.00000125 //1.25 us
//
//#define WS2812_RESET_T 50
//
//#define WS2812_CERO  0.32
//#define WS2812_UNO   0.64
//
//#define wsCero(x) ((x+1) * WS2812_CERO)
//#define wsUno(x) ((x+1) * WS2812_UNO)
//
//
//
//uint32_t calcularCuentaPeriodoNeopixel(uint32_t freqTimer,float periodoNeopixel);
//
//
//void initNeopixels(neoPixelRGB_t *instanciaNeopixel,
//		           PixelRGB_t * arrNPix,
//				   uint32_t numPix,
//				   uint32_t *Dmabuff,
//				   uint32_t Dmasize,
//				   TIM_HandleTypeDef *htim,
//				   uint32_t canal,
//				   uint32_t frecuenciaTimer);
//
//
//void neopixelShow(neoPixelRGB_t *leds);
//
//
//
//
//
//
//
//
