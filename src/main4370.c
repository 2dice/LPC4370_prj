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

#define FFT_SAMPLES 8192 /* 4096 real party and 4096 imaginary parts */
#define FFT_SIZE (FFT_SAMPLES / 2) /* FFT size is always the same size as we have samples, so 2048 in our case */
static float32_t Output[FFT_SIZE];
static float32_t maxValue;	/* Max FFT value is stored here */
static dac_buffer_t buf;
void fft_gen(void){
    //TODO:バッファをstaticにして関数切り出し．割り込みから呼ぶ，切り出し先はfft.c
	arm_cfft_radix4_instance_f32 S;	/* ARM CFFT module */
	float32_t Input[FFT_SAMPLES];
	uint32_t maxIndex;	/* Index in Output array where max value is */
	//ジェネレータのバッファを埋める(ジェネレータ入力時の不足データを繰り返しで埋める)(TODO:ADとgenを振り分け)
	uint16_t i;
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
	//DA=30000sps/4096bit=7.32421875Hzステップでデータが格納されている
}
static uint16_t adc_buf[FFT_SIZE];
void fft_adc(void){
	uint16_t i;
    //TODO:バッファをstaticにして関数切り出し．割り込みから呼ぶ，切り出し先はfft.c
	arm_cfft_radix4_instance_f32 S;	/* ARM CFFT module */
	float32_t Input[FFT_SAMPLES];
	uint32_t maxIndex;	/* Index in Output array where max value is */

	/* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
	arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
	for (i = 0; i < FFT_SAMPLES; i += 2) {
		Input[(uint16_t)i] = (float32_t)((float32_t)adc_buf[i/2] - (float32_t)2048.0) / (float32_t)2048.0;
		Input[(uint16_t)(i + 1)] = 0;
	}
	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_radix4_f32(&S, Input);
	/* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
	arm_cmplx_mag_f32(Input, Output, FFT_SIZE);
	/* Calculates maxValue and returns corresponding value */
	arm_max_f32(Output, FFT_SIZE, &maxValue, &maxIndex);
	//AD=79872sps/4096bit=19.5Hzステップでデータが格納される
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
static uint16_t lcd_data[240][25];
int main(void) {
    setup_systemclock();

    //TODO:TimerInitに切り出し
	// Setup SysTick Timer to interrupt at 10 msec intervals
	SysTick_Config(CGU_GetPCLKFrequency(CGU_PERIPHERAL_M4CORE)/100);

	GPIO_SetDir(0,1<<8, 1);	// GPIO0[8](LED)を出力に設定
	GPIO_ClearValue(0,1<<8);// GPIO0[8](LED)出力L

////////////////////wave_gen///////////////////////////////
	gen_dac_cfg_t cfg;
    cfg.amplitude=5000;
    cfg.dcOffset=0;
    cfg.frequency=150;
    cfg.waveform=GEN_DAC_CFG_WAVE_TRIANGLE;
    wave_gen(&cfg, &buf);

////////////////////fft///////////////////////////////
    fft_gen();

////////////////spi///////////////////////////////////
	lcd_init();
	lcd_clear();
////////////////lcd///////////////////////////////////
//		uint16_t lcd_x;
//		uint16_t lcd_y;
//		uint8_t scale_time = 2;//横軸データ数を2倍表示
//		uint8_t scale_fft = 2;//横軸データ数を2倍表示

//////////////////////////////////////////////////////
//	uint16_t i;
//	int j;

	ADC_DMA_Init();
    NVIC_SetPriority(DMA_IRQn,   ((0x01<<3)|0x01));

    // Enter an infinite loop
    while(1) {
//wav_genタイムドメイン表示
//    	for (i = 0; i < 240; i++) {
//    		for (j = 0; j < 25; j++) {
//    			lcd_data[i][j]=0x0000;
//    		}
//    	}
//    	for (i = 0; i < 400*scale_time; i++ ){
//    	    lcd_x = ((int16_t)(i/scale_time) -200) * -1 + 200;//左右逆転(LCD都合
//    		lcd_y = (buf.LUT_BUFFER[i]-2048)/17 + 120;//0~240に正規化
//    		lcd_data[lcd_y][lcd_x/16] = lcd_data[lcd_y][lcd_x/16] | 0x01<<(lcd_x%16);
//    	}
//    	lcd_write(lcd_data);
    	systick_delay(50);
    }
	ADC_DMA_Exit();
    return 0 ;
}

//TODO:割り込みに切り出し,capture countが必要(buffer0/1から吸い出すとき)
extern uint32_t capture_count;
__RAMFUNC(RAM)
void DMA_IRQHandler (void)
{
	uint16_t i;
	uint16_t j;
////////////////lcd///////////////////////////////////
	uint16_t lcd_x;
	uint16_t lcd_y;
	uint8_t scale_fft = 1;//横軸データ数を2倍表示
	uint8_t scale_time = 2;//横軸データ数を2倍表示

//////////////////////////////////////////////////////
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
	  ADC = (uint32_t*)CAPTUREBUFFER0;
	  capture_count ++;
  }else{
	  GPIO_ClearValue(0,1<<8);// GPIO0[8]出力L
	  ADC = (uint32_t*)CAPTUREBUFFER1;
	  capture_count = 0;
  }
//AD値を配列に格納
  for (i = 0; i < 4096/2-1; i++){
	  adc_buf[i*2] = (uint16_t)(ADC[i] & 0x00000FFF);
	  adc_buf[i*2+1] = (uint16_t)((ADC[i] & 0x0FFF0000)>>16);
  }
//LCDデータクリア
  for (i = 0; i < 240; i++) {
	  for (j = 0; j < 25; j++) {
		  lcd_data[i][j]=0x0000;
	  }
  }
//wav_adcFFT
  fft_adc();
  for (i = 0; i < 400*scale_fft; i++ ){
	  lcd_x = ((int16_t)(i/scale_fft) -200) * -1 + 200;//左右逆転(LCD都合
	  lcd_y = (uint16_t)(Output[i]/maxValue*240);//FFT結果のMAX値を240に正規化
      for (j = 0; j < lcd_y; j++){
    	  lcd_data[j][lcd_x/16] = lcd_data[j][lcd_x/16] | 0x01<<(lcd_x%16);//結果を白抜きにする
      }
  }
//wav_adcタイムドメイン表示
  for (i = 0; i < 400*scale_time; i++ ){
	  lcd_x = ((int16_t)(i/scale_time) -200) * -1 + 200;//左右逆転(LCD都合
	  lcd_y = (adc_buf[i]-2048)/17 + 120;//0~240に正規化
	  lcd_data[lcd_y][lcd_x/16] = lcd_data[lcd_y][lcd_x/16] | 0x01<<(lcd_x%16);
  }
  lcd_write(lcd_data);
}
