#ifndef __P_SPI_H__
#define __P_SPI_H__

#include <lpc43xx.h>

void spi_lcd_init(void);
void spi_lcd_write(uint8_t data[50][240]);

#endif
