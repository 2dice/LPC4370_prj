/*
 * p_spi.c
 *
 *  Created on: 2015/08/13
 *      Author: 2dice
 */

#include <lpc43xx_ssp.h>
#include "p_spi.h"


/*! Chip Select On. GPIO is used instead of the SSP_SSEL. */
#define CS_OFF (LPC_GPIO_PORT->CLR[5] |= (1UL << 9)) // p3.2 (gpio5[9]) -> low

/*! Chip Select Off. GPIO is used instead of the SSP_SSEL. */
#define CS_ON (LPC_GPIO_PORT->SET[5] |= (1UL << 9)) // p3.2 (gpio5[9]) -> high

/*! SPI bus to use */
#define SSP_PORT  (LPC_SSP1)

/*! Clock rate to use */
#define SSP_CLOCK 2000000

static SSP_CFG_Type SSP_ConfigStruct;


void spi_lcd_init(void)
{
  // Initialize SSP configuration structure to default
  SSP_ConfigStructInit(&SSP_ConfigStruct);

  // Set clock rate and number of address bits
  SSP_ConfigStruct.ClockRate = SSP_CLOCK;
  SSP_ConfigStruct.Databit = SSP_DATABIT_8;

  // Initialize SSP peripheral with parameter given in structure above
  SSP_Init(SSP_PORT, &SSP_ConfigStruct);

  // Enable SSP peripheral
  SSP_Cmd(SSP_PORT, ENABLE);
}
#define RBIT8(x) (uint8_t)(__RBIT((uint32_t)x) >> 24)
void spi_lcd_write(uint8_t data[50][240])
{
	SSP_DATA_SETUP_Type sspCfg;
	uint8_t tmp[2];
	int i;
	int j;

	CS_ON;
//モード選択8bit
	tmp[0] = RBIT8(0x01);//データ更新モード,COM反転なし
	sspCfg.tx_data = tmp;
	sspCfg.rx_data = NULL;
	sspCfg.length  = 1;
	SSP_ReadWrite(SSP_PORT, &sspCfg, SSP_TRANSFER_POLLING);
//アドレス8bit
	tmp[0] = RBIT8(0x01);
	sspCfg.tx_data = tmp;
	sspCfg.rx_data = NULL;
	sspCfg.length  = 1;
	SSP_ReadWrite(SSP_PORT, &sspCfg, SSP_TRANSFER_POLLING);
//データ400bit(50byte)
	for (i=0; i < 50; i++){
		tmp[0] = RBIT8(data[i][0]);
		sspCfg.tx_data = tmp;
		sspCfg.rx_data = NULL;
		sspCfg.length  = 1;
		SSP_ReadWrite(SSP_PORT, &sspCfg, SSP_TRANSFER_POLLING);
	}
	for (j = 1; j < 240 ; j++){	//line-loop
		//ダミー8bit+アドレス8bit
		tmp[0] = RBIT8(0x00);//ダミー
		tmp[1] = RBIT8(j+1);
		sspCfg.tx_data = tmp;
		sspCfg.rx_data = NULL;
		sspCfg.length  = 2;
		SSP_ReadWrite(SSP_PORT, &sspCfg, SSP_TRANSFER_POLLING);
		//データ
		for (i=0; i < 50; i++){	//row-loop
			tmp[0] = RBIT8(data[i][j]);
			sspCfg.tx_data = tmp;
			sspCfg.rx_data = NULL;
			sspCfg.length  = 1;
			SSP_ReadWrite(SSP_PORT, &sspCfg, SSP_TRANSFER_POLLING);
		}
	}

	//ダミー16bit
	tmp[0] = RBIT8(0x00);
	tmp[1] = RBIT8(0x00);
	sspCfg.tx_data = tmp;
	sspCfg.rx_data = NULL;
	sspCfg.length  = 2;
	SSP_ReadWrite(SSP_PORT, &sspCfg, SSP_TRANSFER_POLLING);

	CS_OFF;
}

