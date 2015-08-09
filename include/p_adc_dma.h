#ifndef __P_ADC_DMA_H__
#define __P_ADC_DMA_H__

#include <lpc43xx.h>


void setup_pll0audio(uint32_t, uint32_t, uint32_t);
void VADC_Init(void);
void VADC_SetupDMA(void);
void VADC_Start(void);
void VADC_Stop(void);


#endif
