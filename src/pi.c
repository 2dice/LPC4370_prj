/*
 * pi.c
 *
 *  Created on: 2015/08/09
 *      Author: 2dice
 */

#include "p_adc_dma.h"


// PLL0AUDIO: 37.5kHz = (12MHz / 256) * (4 * 2) / (5 * 2)
#define PLL0_MSEL	4
#define PLL0_NSEL	256
#define PLL0_PSEL	5
void ADC_DMA_Init()
{
	setup_pll0audio(PLL0_MSEL, PLL0_NSEL, PLL0_PSEL);
	VADC_Init();
	VADC_SetupDMA();
	VADC_Start();
}

void ADC_DMA_Exit(void)
{
	VADC_Stop();
}
