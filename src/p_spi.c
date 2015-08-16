/*
 * p_spi.c
 *
 *  Created on: 2015/08/13
 *      Author: 2dice
 */

#include <lpc43xx_ssp.h>
#include <lpc43xx_scu.h>
#include "p_spi.h"


/*! Chip Select On. GPIO is used instead of the SSP_SSEL. */
#define CS_OFF (LPC_GPIO_PORT->CLR[5] |= (1UL << 9)) // p3.2 (gpio5[9]) -> low

/*! Chip Select Off. GPIO is used instead of the SSP_SSEL. */
#define CS_ON (LPC_GPIO_PORT->SET[5] |= (1UL << 9)) // p3.2 (gpio5[9]) -> high

/*! SPI bus to use */
#define SSP_PORT  (LPC_SSP1)

/*! Clock rate to use */
#define SSP_CLOCK 10000000

#define SETTINGS_SSP (PUP_DISABLE | PDN_DISABLE | SLEWRATE_SLOW | INBUF_ENABLE  | FILTER_ENABLE)
#define SETTINGS_GPIO_OUT (PUP_DISABLE | PDN_DISABLE | SLEWRATE_SLOW |          FILTER_ENABLE)

static SSP_CFG_Type SSP_ConfigStruct;


void spi_lcd_init(void)
{
	scu_pinmux(0x1,  3, SETTINGS_SSP, FUNC5); //SSP1_MISO
	scu_pinmux(0x1,  4, SETTINGS_SSP, FUNC5); //SSP1_MOSI
	scu_pinmux(0xF,  4, SETTINGS_SSP, FUNC0); //SSP1_SCK
	scu_pinmux(0x3,  2, SETTINGS_GPIO_OUT, FUNC4); //GPIO5[9], available on J7-12

	LPC_GPIO_PORT->DIR[5] |= (1UL << 9);
	LPC_GPIO_PORT->SET[5] |= (1UL << 9);

  // Initialize SSP configuration structure to default
  SSP_ConfigStructInit(&SSP_ConfigStruct);

  // Set clock rate and number of address bits
  SSP_ConfigStruct.ClockRate = SSP_CLOCK;
  SSP_ConfigStruct.Databit = SSP_DATABIT_16;

  // Initialize SSP peripheral with parameter given in structure above
  SSP_Init(SSP_PORT, &SSP_ConfigStruct);

  // Enable SSP peripheral
  SSP_Cmd(SSP_PORT, ENABLE);
}

#define RBIT8(x) (uint8_t)(__RBIT((uint32_t)(x)) >> 24)
void spi_lcd_clear(void)
{
	SSP_DATA_SETUP_Type sspCfg;
	uint8_t tmp[2];
	//モード選択8bit+アドレス8bit
	tmp[0] = RBIT8(0x04);//オールクリア,COM反転なし
	tmp[1] = RBIT8(0x00);//dummy
	sspCfg.tx_data = tmp;
	sspCfg.rx_data = NULL;
	sspCfg.length  = 2;
	SSP_ReadWrite(SSP_PORT, &sspCfg, SSP_TRANSFER_POLLING);
}

#define RBIT16(x) (uint16_t)(__RBIT((uint32_t)(x)) >> 16)
void spi_lcd_write(uint16_t data[240][25])
{
	SSP_DATA_SETUP_Type sspCfg;
	uint16_t tmp[240][26];
	int i;
	int j;

	CS_ON;
//モード選択8bit+アドレス8bit
	tmp[0][0] = RBIT16(0x01 | 0x01<<8);//データ更新モード,COM反転なし|アドレスL1
//データ400bit(50byte)
	for (i=0; i < 25; i++){
		tmp[0][i+1] = RBIT16(data[0][i]);
	}

	for (j = 1; j < 240 ; j++){	//line-loop
		//ダミー8bit+アドレス8bit
		tmp[j][0] = RBIT16(0x00 | (j+1)<<8);
		//データ400bit(50byte)
		for (i = 0; i < 25; i++){	//row-loop
			tmp[j][i+1] = RBIT16(data[j][i]);
		}
	}
	sspCfg.tx_data = tmp;
	sspCfg.rx_data = NULL;
	sspCfg.length  = 52*240;
	SSP_ReadWrite(SSP_PORT, &sspCfg, SSP_TRANSFER_POLLING);

	//ダミー16bit
	tmp[0][0] = RBIT16(0x0000);
	sspCfg.tx_data = tmp;
	sspCfg.rx_data = NULL;
	sspCfg.length  = 2;
	SSP_ReadWrite(SSP_PORT, &sspCfg, SSP_TRANSFER_POLLING);

	CS_OFF;
	//tmp[0] = RBIT16(0x55<<8 | 0xAA);
}

void spi_lcd_write2(uint8_t data[50][240])
{
	SSP_DATA_SETUP_Type sspCfg;
	uint8_t tmp[240][52];
	int i;
	int j;

	CS_ON;
//モード選択8bit+アドレス8bit
	tmp[0][0] = RBIT8(0x01);//データ更新モード,COM反転なし
	tmp[0][1] = RBIT8(0x01);//アドレスL1
//データ400bit(50byte)
	for (i=0; i < 50; i++){
		tmp[0][i+2] = RBIT8(data[i][0]);
	}

	for (j = 1; j < 240 ; j++){	//line-loop
		//ダミー8bit+アドレス8bit
		tmp[j][0] = RBIT8(0x00);//ダミー
		tmp[j][1] = RBIT8(j+1);
		//データ400bit(50byte)
		for (i=0; i < 50; i++){	//row-loop
			tmp[j][i+2] = RBIT8(data[i][j]);
		}
	}
	sspCfg.tx_data = tmp;
	sspCfg.rx_data = NULL;
	sspCfg.length  = 52*240;
	SSP_ReadWrite(SSP_PORT, &sspCfg, SSP_TRANSFER_POLLING);

	//ダミー16bit
	tmp[0][0] = RBIT8(0x00);
	tmp[0][1] = RBIT8(0x00);
	sspCfg.tx_data = tmp;
	sspCfg.rx_data = NULL;
	sspCfg.length  = 2;
	SSP_ReadWrite(SSP_PORT, &sspCfg, SSP_TRANSFER_POLLING);

	CS_OFF;
}

