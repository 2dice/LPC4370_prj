#ifndef __PERIPHERAL_INTERFACE_H__
#define __PERIPHERAL_INTERFACE_H__

#include <lpc43xx.h>

#define CAPTUREBUFFER0		((uint8_t*)0x20000000)

void ADC_DMA_Init(void);
void ADC_DMA_Exit(void);


void lcd_init(void);
void lcd_clear(void);
void lcd_write(uint16_t data[240][25]);

#endif
