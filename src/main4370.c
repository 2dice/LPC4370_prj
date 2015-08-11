/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

// ペリフェラルを定義したライブラリをインクルード(CMSIS_LPC43xx_DriverLib)
#include <lpc43xx.h>
#include <lpc43xx_gpio.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_emc.h>

#include <cr_section_macros.h>

#include <stdio.h>

// 独自定義したヘッダをインクルード
#include "pi.h"
#include "wave_gen.h"

/*! Frequency of external xtal */
#define XTAL_FREQ  (12000000UL)
extern uint32_t CGU_ClockSourceFrequency[CGU_CLKSRC_NUM];
void setup_systemclock()
{
	/* enable the crystal oscillator */
	CGU_SetXTALOSC(XTAL_FREQ);
	CGU_EnableEntity(CGU_CLKSRC_XTAL_OSC, ENABLE);

	/* connect the cpu to the xtal */
	CGU_EntityConnect(CGU_CLKSRC_XTAL_OSC, CGU_BASE_M4);

	/* connect the PLL to the xtal */
	CGU_EntityConnect(CGU_CLKSRC_XTAL_OSC, CGU_CLKSRC_PLL1);

	/* configure the PLL to 120 MHz */
	CGU_SetPLL1(10);
	while((LPC_CGU->PLL1_STAT&1) == 0x0);

	/* enable the PLL */
	CGU_EnableEntity(CGU_CLKSRC_PLL1, ENABLE);

	/* connect to the CPU core */
	CGU_EntityConnect(CGU_CLKSRC_PLL1, CGU_BASE_M4);

	SystemCoreClock = 120000000;

	/* wait one msec */
	emc_WaitUS(1000);

	/* Change the clock to 204 MHz */
	CGU_SetPLL1(17);
	while((LPC_CGU->PLL1_STAT&1) == 0x0);

	SystemCoreClock = 204000000;

    CGU_ClockSourceFrequency[CGU_CLKSRC_PLL1] = SystemCoreClock;
}

volatile uint32_t msTicks; // counter for 10ms SysTicks

// ****************TODO:割り込みに切り出し
//  SysTick_Handler - just increment SysTick counter
void SysTick_Handler(void) {
	msTicks++;
}

// ****************
// systick_delay - creates a delay of the appropriate number of Systicks
void systick_delay(uint32_t delayTicks) {
	uint32_t currentTicks;

	currentTicks = msTicks;	// read current tick counter
	// Now loop until required number of ticks passes.
	while ((msTicks - currentTicks) < delayTicks);
}

volatile uint32_t *ADC;
int main(void) {
    setup_systemclock();

    gen_dac_cfg_t cfg;
    cfg.amplitude=2500;
    cfg.dcOffset=0;
    cfg.frequency=1500;
    cfg.waveform=GEN_DAC_CFG_WAVE_SINUS;
    dac_buffer_t buf;
    wave_gen(&cfg, &buf);

//    ADC_DMA_Init();
//    NVIC_SetPriority(DMA_IRQn,   ((0x01<<3)|0x01));

    //TODO:TimerInitに切り出し
	// Setup SysTick Timer to interrupt at 10 msec intervals
	SysTick_Config(CGU_GetPCLKFrequency(CGU_PERIPHERAL_M4CORE)/100);

	GPIO_SetDir(0,1<<8, 1);	// GPIO0[8]を出力に設定
	GPIO_ClearValue(0,1<<8);// GPIO0[8]出力L

	int i;
    // Enter an infinite loop
    while(1) {
    	for (i=0; i < buf.numLUTEntries; i++)
    	{
    		printf("%d\n",buf.LUT_BUFFER[i]);//printf表示テスト用.処理が遅すぎて割り込みが不安定になる
        	systick_delay(100);
    	}
    }
	ADC_DMA_Exit();
    return 0 ;
}

//TODO:割り込みに切り出し,capture countが必要(buffer0/1から吸い出すとき)
extern uint32_t capture_count;
__RAMFUNC(RAM)
void DMA_IRQHandler (void)
{
  if (LPC_GPDMA->INTERRSTAT & 1)
  {
    LPC_GPDMA->INTERRCLR = 1;
  }
  if (LPC_GPDMA->INTTCSTAT & 1)
  {
	LPC_GPDMA->INTTCCLEAR = 1;
  }

  if (capture_count == 0)
  {
	  GPIO_SetValue(0,1<<8);// GPIO0[8]出力H
	  capture_count ++;
  }else{
	  GPIO_ClearValue(0,1<<8);// GPIO0[8]出力L
	  capture_count = 0;
  }
  ADC = (uint32_t*)CAPTUREBUFFER0;
}
