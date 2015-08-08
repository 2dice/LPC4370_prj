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

#include <cr_section_macros.h>

#include <stdio.h>

// 独自定義したヘッダをインクルード
#include "vadc.h"

volatile uint32_t msTicks; // counter for 10ms SysTicks

// ****************
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

int main(void) {
	printf("test\n");//printf表示テスト用
	// Setup SysTick Timer to interrupt at 10 msec intervals
	SysTick_Config(CGU_GetPCLKFrequency(CGU_PERIPHERAL_M4CORE)/100);

	GPIO_SetDir(0,1<<8, 1);	// GPIO0[8]を出力に設定
	GPIO_ClearValue(0,1<<8);// GPIO0[8]出力L

    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
		systick_delay(50);
		GPIO_SetValue(0,1<<8);// GPIO0[8]出力H
		systick_delay(100);
		GPIO_ClearValue(0,1<<8);// GPIO0[8]出力L

        i++ ;
    }
    return 0 ;
}
