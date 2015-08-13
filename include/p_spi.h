#ifndef __P_SPI_H__
#define __P_SPI_H__

#include <lpc43xx.h>

void spi_lcd_init(void);
void spi_lcd_clear(void);
void spi_lcd_write(uint16_t data[240][25]);

#endif
