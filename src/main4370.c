/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

// ここにペリフェラルを定義したヘッダをインクルード(CMSIS_LPC43xx_DriverLib)
#include <lpc43xx.h>

#include <cr_section_macros.h>

#include <stdio.h>

#include "lpc43xx_gpio.h"

#include "lpc43xx_cgu.h"

volatile uint32_t msTicks; // counter for 1ms SysTicks

// ****************
//  SysTick_Handler - just increment SysTick counter
void SysTick_Handler(void) {
	msTicks++;
}

// ****************
// systick_delay - creates a delay of the appropriate number of Systicks (happens every 1 ms)
void systick_delay(uint32_t delayTicks) {
	uint32_t currentTicks;

	currentTicks = msTicks;	// read current tick counter
	// Now loop until required number of ticks passes.
	while ((msTicks - currentTicks) < delayTicks);
}



int main(void) {
	printf("test\n");
	// Setup SysTick Timer to interrupt at 1 msec intervals
	SysTick_Config(CGU_GetPCLKFrequency(CGU_PERIPHERAL_M4CORE)/1000);

	GPIO_SetDir(0,1<<8, 1);
	GPIO_ClearValue(0,1<<8);

    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
		systick_delay(500);
		GPIO_SetValue(0,1<<8);
		systick_delay(1000);
		GPIO_ClearValue(0,1<<8);

        i++ ;
    }
    return 0 ;
}
