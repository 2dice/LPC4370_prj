#ifndef __PERIPHERAL_INTERFACE_H__
#define __PERIPHERAL_INTERFACE_H__

#define CAPTUREBUFFER0		((uint8_t*)0x20000000)

void ADC_DMA_Init(void);
void ADC_DMA_Exit(void);


#endif
