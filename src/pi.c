/*
 * pi.c
 *
 *  Created on: 2015/08/09
 *      Author: 2dice
 */

#include "pi.h"
#include "p_adc_dma.h"


// PLL0AUDIO: 150kHz = (12MHz / 64) * (4 * 2) / (5 * 2)
//mselは1~131071の範囲であること
//nselは1~256の範囲であること
//pselは1~32の範囲であること
#define PLL0_MSEL	4
#define PLL0_NSEL	64
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
