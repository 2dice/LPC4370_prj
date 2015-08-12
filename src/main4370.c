/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC43xx.h"
#endif
// ペリフェラルを定義したライブラリをインクルード(CMSIS_LPC43xx_DriverLib)
#include <lpc43xx.h>
#include <lpc43xx_gpio.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_emc.h>

#include <cr_section_macros.h>

#include <stdio.h>
////////////////FFT用////////////////////
#include <arm_math.h>
////////////////////////////////////////

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
//    ADC_DMA_Init();
//    NVIC_SetPriority(DMA_IRQn,   ((0x01<<3)|0x01));

    //TODO:TimerInitに切り出し
	// Setup SysTick Timer to interrupt at 10 msec intervals
	SysTick_Config(CGU_GetPCLKFrequency(CGU_PERIPHERAL_M4CORE)/100);

	GPIO_SetDir(0,1<<8, 1);	// GPIO0[8]を出力に設定
	GPIO_ClearValue(0,1<<8);// GPIO0[8]出力L

////////////////////wave_gen///////////////////////////////
	int i;
	gen_dac_cfg_t cfg;
    cfg.amplitude=5000;
    cfg.dcOffset=0;
    cfg.frequency=1500;
    cfg.waveform=GEN_DAC_CFG_WAVE_SQUARE;
    dac_buffer_t buf;
    wave_gen(&cfg, &buf);

////////////////////fft///////////////////////////////
	#define FFT_SAMPLES 512 /* 2048 real party and 2048 imaginary parts */
	#define FFT_SIZE (FFT_SAMPLES / 2) /* FFT size is always the same size as we have samples, so 2048 in our case */
	arm_cfft_radix4_instance_f32 S;	/* ARM CFFT module */
	float32_t Input[FFT_SAMPLES];
	float32_t Output[FFT_SIZE];
	float32_t maxValue;	/* Max FFT value is stored here */
	uint32_t maxIndex;	/* Index in Output array where max value is */

	//ジェネレータのバッファを埋める
	int j;
	j = 0;
	for (i=buf.numLUTEntries; i < FFT_SIZE; i++)
	{
		buf.LUT_BUFFER[i] = buf.LUT_BUFFER[j];
		if(j==buf.numLUTEntries-1)
		{
			j=0;
		}else{
			j++;
		}
	}

	/* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
	arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
	for (i = 0; i < FFT_SAMPLES; i += 2) {
		Input[(uint16_t)i] = (float32_t)((float32_t)buf.LUT_BUFFER[i/2] - (float32_t)2048.0) / (float32_t)2048.0;
		Input[(uint16_t)(i + 1)] = 0;
	}
	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_radix4_f32(&S, Input);
	/* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
	arm_cmplx_mag_f32(Input, Output, FFT_SIZE);
	/* Calculates maxValue and returns corresponding value */
	arm_max_f32(Output, FFT_SIZE, &maxValue, &maxIndex);
	printf("max%f\n",maxValue);
	printf("index%d\n",maxIndex);
	printf("FFT\n");
	for (i=0; i < FFT_SIZE/2; i++)
	{
		printf("%f\n",Output[i]);//printf表示テスト用.処理が遅すぎて割り込みが不安定になる
	}
///////////////////////////////////////////////////

    // Enter an infinite loop
    while(1) {
    	systick_delay(100);
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
