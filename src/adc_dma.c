#include <lpc43xx.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_rgu.h>
#include <lpc43xx_gpdma.h>

#include "adc_dma.h"

////////////////////////////////DMA-CLK設定////////////////////////////
// PLL0AUDIO: 37.5kHz = (12MHz / 256) * (4 * 2) / (5 * 2)
#define PLL0_MSEL	4
#define PLL0_NSEL	256
#define PLL0_PSEL	5
#define PLL0_MSEL_MAX (1<<15)
#define PLL0_NSEL_MAX (1<<8)
#define PLL0_PSEL_MAX (1<<5)

static uint32_t FindMDEC(uint32_t msel)
{
	//mselは1~131071の範囲であること
  /* multiplier: compute mdec from msel */
  uint32_t x = 0x4000;
  uint32_t im;

  switch (msel)
  {
    case 0:
      return 0xffffffff;
    case 1:
      return 0x18003;
    case 2:
      return 0x10003;
    default:
      for (im = msel; im <= PLL0_MSEL_MAX; im++)
      {
        x = (((x ^ x>>1) & 1) << 14) | (x>>1 & 0xFFFF);
      }
      return x;
  }
}

static uint32_t FindNDEC(uint32_t nsel)
{
	//nselは1~256の範囲であること
  /* pre-divider: compute ndec from nsel */
  uint32_t x = 0x80;
  uint32_t in;

  switch (nsel)
  {
    case 0:
      return 0xffffffff;
    case 1:
      return 0x302;
    case 2:
      return 0x202;
    default:
      for (in = nsel; in <= PLL0_NSEL_MAX; in++)
      {
        x = (((x ^ x>>2 ^ x>>3 ^ x>>4) & 1) << 7) | (x>>1 & 0xFF);
      }
      return x;
  }
}

static uint32_t FindPDEC(uint32_t psel)
{
	//pselは1~32の範囲であること
  /* post-divider: compute pdec from psel */
  uint32_t x = 0x10;
  uint32_t ip;

  switch (psel)
  {
    case 0:
      return 0xffffffff;
    case 1:
      return 0x62;
    case 2:
      return 0x42;
    default:
      for (ip = psel; ip <= PLL0_PSEL_MAX; ip++)
      {
        x = (((x ^ x>>2) & 1) << 4) | (x>>1 & 0x3F);
      }
      return x;
  }
}

extern uint32_t CGU_ClockSourceFrequency[CGU_CLKSRC_NUM];
static void setup_pll0audio(uint32_t msel, uint32_t nsel, uint32_t psel)
{
  uint32_t ClkSrc;

  //CGU_EnableEntity(CGU_BASE_PERIPH, DISABLE);
  CGU_EnableEntity(CGU_BASE_VADC, DISABLE);

  /* source = XTAL OSC 12 MHz */
  ClkSrc = CGU_CLKSRC_XTAL_OSC;

  /* disable clock, disable skew enable, power down pll,
  * (dis/en)able post divider, (dis/en)able pre-divider,
  * disable free running mode, disable bandsel,
  * enable up limmiter, disable bypass
  */
  LPC_CGU->PLL0AUDIO_CTRL = (ClkSrc << 24) | _BIT(0);  /* power down */

  /* set NDEC, PDEC and MDEC register */
  LPC_CGU->PLL0AUDIO_NP_DIV = (FindNDEC(nsel)<<12) | (FindPDEC(psel) << 0);
  LPC_CGU->PLL0AUDIO_MDIV = FindMDEC(msel);

  LPC_CGU->PLL0AUDIO_CTRL = (ClkSrc << 24) | (6<< 12);     // fractional divider off and bypassed

  /* wait for lock */
  //while (!(LPC_CGU->PLL0AUDIO_STAT & 1));

  /* enable clock output */
  LPC_CGU->PLL0AUDIO_CTRL |= (1<<4); /* CLKEN */

  CGU_ClockSourceFrequency[CGU_CLKSRC_PLL0_AUDIO] =
    msel * (CGU_ClockSourceFrequency[ClkSrc] / (psel * nsel));

  //CGU_UpdateClock();

  // Re-enable the clocks that uses PLL0AUDIO
  //CGU_EnableEntity(CGU_BASE_PERIPH, ENABLE);
  CGU_EnableEntity(CGU_BASE_VADC, ENABLE);
}

void set_ADC_DMA_clk()
{
	setup_pll0audio(PLL0_MSEL, PLL0_NSEL, PLL0_PSEL);
}

////////////////////////////////ADCコントロール////////////////////////////
#define CAPTUREBUFFER_SIZE	0x10000
#define CAPTUREBUFFER1		((uint8_t*)0x20008000)
#define CAPTUREBUFFER_SIZEHALF	0x8000
#define ADCCLK_MATCHVALUE	(4 - 1)  // 39.936MHz / 4 = 9.984MHz
#define ADCCLK_DGECI 0
#define FIFO_SIZE       8
#define DMA_LLI_NUM    16
static GPDMA_LLI_Type DMA_LTable[DMA_LLI_NUM];
volatile int32_t capture_count;
void VADC_SetupDMA(void)
{
  int i;
  uint32_t transfersize;
  uint32_t blocksize;
  uint8_t *buffer;

  NVIC_DisableIRQ(DMA_IRQn);
  LPC_GPDMA->C0CONFIG = 0;

  /* clear all interrupts on channel 0 */
  LPC_GPDMA->INTTCCLEAR = 0x01;
  LPC_GPDMA->INTERRCLR = 0x01;

  /* Setup the DMAMUX */
  LPC_CREG->DMAMUX &= ~(0x3<<(VADC_DMA_WRITE*2));
  LPC_CREG->DMAMUX |= 0x3<<(VADC_DMA_WRITE*2);  /* peripheral 7 vADC Write(0x3) */
  LPC_CREG->DMAMUX &= ~(0x3<<(VADC_DMA_READ*2));
  LPC_CREG->DMAMUX |= 0x3<<(VADC_DMA_READ*2);  /* peripheral 8 vADC read(0x3) */

  LPC_GPDMA->CONFIG = 0x01;  /* Enable DMA channels, little endian */
  while ( !(LPC_GPDMA->CONFIG & 0x01) );

  // The size of the transfer is in multiples of 32bit copies (hence the /4)
  // and must be even multiples of FIFO_SIZE.
  buffer = CAPTUREBUFFER0;
  blocksize = CAPTUREBUFFER_SIZE / DMA_LLI_NUM;
  transfersize = blocksize / 4;

  for (i = 0; i < DMA_LLI_NUM; i++)
  {
	if (i == DMA_LLI_NUM / 2)
		buffer = CAPTUREBUFFER1;
	DMA_LTable[i].SrcAddr = VADC_DMA_READ_SRC;
	DMA_LTable[i].DstAddr = (uint32_t)buffer;
	DMA_LTable[i].NextLLI = (uint32_t)(&DMA_LTable[(i+1) % DMA_LLI_NUM]);
	DMA_LTable[i].Control = (transfersize << 0) |      // Transfersize (does not matter when flow control is handled by peripheral)
                           (0x2 << 12)  |          // Source Burst Size
                           (0x2 << 15)  |          // Destination Burst Size
                           //(0x0 << 15)  |          // Destination Burst Size
                           (0x2 << 18)  |          // Source width // 32 bit width
                           (0x2 << 21)  |          // Destination width   // 32 bits
                           (0x1 << 24)  |          // Source AHB master 0 / 1
                           (0x0 << 25)  |          // Dest AHB master 0 / 1
                           (0x0 << 26)  |          // Source increment(LAST Sample)
                           (0x1 << 27)  |          // Destination increment
                           (0x0UL << 31);          // Terminal count interrupt disabled
    buffer += blocksize;
  }

  // Let the last LLI in the chain cause a terminal count interrupt to
  // notify when the capture buffer is completely filled
  DMA_LTable[DMA_LLI_NUM/2 - 1].Control |= (0x1UL << 31); // Terminal count interrupt enabled
  DMA_LTable[DMA_LLI_NUM - 1].Control |= (0x1UL << 31); // Terminal count interrupt enabled

  LPC_GPDMA->C0SRCADDR = DMA_LTable[0].SrcAddr;
  LPC_GPDMA->C0DESTADDR = DMA_LTable[0].DstAddr;
  LPC_GPDMA->C0CONTROL = DMA_LTable[0].Control;
  LPC_GPDMA->C0LLI     = (uint32_t)(&DMA_LTable[1]); // must be pointing to the second LLI as the first is used when initializing
  LPC_GPDMA->C0CONFIG  =  (0x1)        |          // Enable bit
                          (VADC_DMA_READ << 1) |  // SRCPERIPHERAL - set to 8 - VADC
                          (0x0 << 6)   |          // Destination peripheral - memory - no setting
                          (0x2 << 11)  |          // Flow control - peripheral to memory - DMA control
//                          (0x6 << 11)  |          // Flow control - peripheral to memory - peripheral control
                          (0x1 << 14)  |          // Int error mask
                          (0x1 << 15);            // ITC - term count error mask

  NVIC_EnableIRQ(DMA_IRQn);
}

void VADC_Init(void)
{
  CGU_EntityConnect(CGU_CLKSRC_PLL0_AUDIO, CGU_BASE_VADC);
  CGU_EnableEntity(CGU_BASE_VADC, ENABLE);

//  RGU_SoftReset(RGU_SIG_DMA);
//  while(RGU_GetSignalStatus(RGU_SIG_DMA));

  // Reset the VADC block
  RGU_SoftReset(RGU_SIG_VADC);
  while(RGU_GetSignalStatus(RGU_SIG_VADC));

  // Disable the VADC interrupt
  NVIC_DisableIRQ(VADC_IRQn);
  LPC_VADC->CLR_EN0 = STATUS0_CLEAR_MASK;         // disable interrupt0
  LPC_VADC->CLR_STAT0 = STATUS0_CLEAR_MASK;       // clear interrupt status
  while(LPC_VADC->STATUS0 & 0x7d);  // wait for status to clear, have to exclude FIFO_EMPTY (bit 1)
  LPC_VADC->CLR_EN1 = STATUS1_CLEAR_MASK;          // disable interrupt1
  LPC_VADC->CLR_STAT1 = STATUS1_CLEAR_MASK;  // clear interrupt status
  while(LPC_VADC->STATUS1);         // wait for status to clear

  // Make sure the VADC is not powered down
  LPC_VADC->POWER_DOWN =
    (0<<0);        /* PD_CTRL:      0=disable power down, 1=enable power down */

  // Clear FIFO
  LPC_VADC->FLUSH = 1;

  // FIFO Settings
  LPC_VADC->FIFO_CFG =
    (1<<0) |         /* PACKED_READ:      0= 1 sample packed into 32 bit, 1= 2 samples packed into 32 bit */
    (FIFO_SIZE<<1);  /* FIFO_LEVEL:       When FIFO contains this or more samples raise FIFO_FULL irq and DMA_Read_Req, default is 8 */

  // Descriptors:
  LPC_VADC->DSCR_STS =
      (0<<0) |       /* ACT_TABLE:        0=table 0 is active, 1=table 1 is active */
      (0<<1);        /* ACT_DESCRIPTOR:   ID of the descriptor that is active */

  LPC_VADC->CONFIG = /* configuration register */
    (1<<0) |        /* TRIGGER_MASK:     0=triggers off, 1=SW trigger, 2=EXT trigger, 3=both triggers */
    (0<<2) |        /* TRIGGER_MODE:     0=rising, 1=falling, 2=low, 3=high external trigger */
    (0<<4) |        /* TRIGGER_SYNC:     0=no sync, 1=sync external trigger input */
    (0<<5) |        /* CHANNEL_ID_EN:    0=don't add, 1=add channel id to FIFO output data */
    (0x90<<6);      /* RECOVERY_TIME:    ADC recovery time from power down, default is 0x90 */

  LPC_VADC->DESCRIPTOR_0[0] =
	  (0<<0) |       /* CHANNEL_NR:    0=convert input 0, 1=convert input 1, ..., 5=convert input 5 */
	  (0<<3) |       /* HALT:          0=continue with next descriptor after this one, 1=halt after this and restart at a new trigger */
	  (0<<4) |       /* INTERRUPT:     1=raise interrupt when ADC result is available */
	  (0<<5) |       /* POWER_DOWN:    1=power down after this conversion */
	  (1<<6) |       /* BRANCH:        0=continue with next descriptor (wraps around after top) */
					 /*                1=branch to the first descriptor in this table */
					 /*                2=swap tables and branch to the first descriptor of the new table */
					 /*                3=reserved (do not store sample). continue with next descriptor (wraps around the top) */
	  (ADCCLK_MATCHVALUE<<8)  |    /* MATCH_VALUE:   Evaluate this desciptor when descriptor timer value is equal to match value */
	  (0<<22) |      /* THRESHOLD_SEL: 0=no comparison, 1=THR_A, 2=THR_B */
	  (1<<24) |      /* RESET_TIME:    1=reset descriptor timer */
	  (1UL<<31);       /* UPDATE_TABLE:  1=update table with all 8 descriptors of this table */

  LPC_VADC->ADC_SPEED =
    ADCCLK_DGECI;   /* DGECx:      For CRS=3 all should be 0xF, for CRS=4 all should be 0xE, */
                       /*             for all other cases it should be 0 */

  LPC_VADC->POWER_CONTROL =
    (0 /*crs*/ << 0) |    /* CRS:          current setting for power versus speed programming */
    (1 << 4) |      /* DCINNEG:      0=no dc bias, 1=dc bias on vin_neg slide */
    (0 << 10) |     /* DCINPOS:      0=no dc bias, 1=dc bias on vin_pos slide */
    (0 << 16) |     /* TWOS:         0=offset binary, 1=two's complement */
    (1 << 17) |     /* POWER_SWITCH: 0=ADC is power gated, 1=ADC is active */
    (1 << 18);      /* BGAP_SWITCH:  0=ADC bandgap reg is power gated, 1=ADC bandgap is active */
}

void VADC_Start(void)
{
	capture_count = 0;
	LPC_VADC->TRIGGER = 1;
}

void VADC_Stop(void)
{
  // disable DMA
  LPC_GPDMA->C0CONFIG |= (1 << 18); //halt further requests

  NVIC_DisableIRQ(I2S0_IRQn);
  NVIC_DisableIRQ(DMA_IRQn);
  //NVIC_DisableIRQ(VADC_IRQn);

  LPC_VADC->TRIGGER = 0;
  // Clear FIFO
  LPC_VADC->FLUSH = 1;
  // power down VADC
  LPC_VADC->POWER_CONTROL = 0;

  // Reset the VADC block
  RGU_SoftReset(RGU_SIG_VADC);
  while(RGU_GetSignalStatus(RGU_SIG_VADC));
}
