//#include "neopixel_basic.h"
//
//uint32_t calcularCuentaPeriodoNeopixel(uint32_t freqTimer,float periodoNeopixel)
//{
//	float periodoTimer = 1.0/freqTimer;
//	uint32_t cuenta = periodoNeopixel /periodoTimer -1;
//	return cuenta;
//}
//
//
//void initNeopixels(neoPixelRGB_t *instanciaNeopixel,
//		           PixelRGB_t * arrNPix,
//				   uint32_t numPix,
//				   uint32_t *Dmabuff,
//				   uint32_t Dmasize,
//				   TIM_HandleTypeDef *htim,
//				   uint32_t canal,
//				   uint32_t frecuenciaTimer)
//{
//	uint32_t cuenta_tmp = calcularCuentaPeriodoNeopixel(frecuenciaTimer,WS2812_PERIODO);
//
//	instanciaNeopixel->arrayPixeles =arrNPix;
//	instanciaNeopixel->numPixeles = numPix;
//	instanciaNeopixel->arrayDma = Dmabuff;
//	instanciaNeopixel->sizeDma = Dmasize;
//	instanciaNeopixel->htim = htim;
//	instanciaNeopixel->channel = canal;
//	instanciaNeopixel->cntCero = wsCero(cuenta_tmp);
//	instanciaNeopixel->cntUno  = wsUno(cuenta_tmp);
//
//}
//void neopixelShow(neoPixelRGB_t *leds)
//{
//	uint32_t *pBuff;
//	pBuff = leds->arrayDma;
//	for (uint32_t i = 0; i < leds->numPixeles; i++)
//	    {
//	       for (uint8_t j = 23; j >= 0; j--)
//	       {
//	         if ((leds->arrayPixeles[i].data >> j) & 0x01)
//	         {
//	           *pBuff = leds->cntUno;
//	         }
//	         else
//	         {
//	           *pBuff = leds->cntCero;
//	         }
//	         pBuff++;
//	     }
//	    }
//	    leds->arrayDma[leds->sizeDma - 1] = 0; // last element must be 0!
//
//	    HAL_TIM_PWM_Start_DMA(leds->htim, leds->channel,leds->arrayDma,leds->sizeDma);
//
//}
