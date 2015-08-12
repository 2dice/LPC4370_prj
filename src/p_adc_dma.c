#include <lpc43xx.h>
#include <lpc43xx_cgu.h>
#include <lpc43xx_rgu.h>
#include <lpc43xx_gpdma.h>

#include "p_adc_dma.h"

/**
  * @brief Product name title=UM????? Chapter title=?????? Modification date=12/11/2012 Major revision=? Minor revision=?  (VADC)
    0x400F0000
  */
typedef struct {                            /*!< (@ 0x400F0000) VADC Structure         */
  __O  uint32_t FLUSH;                      /*!< (@ 0x400F0000) Flushes FIFO */
  __IO uint32_t DMA_REQ;                    /*!< (@ 0x400F0004) Set or clear DMA write request */
  __I  uint32_t FIFO_STS;                   /*!< (@ 0x400F0008) Indicates FIFO fullness status */
  __IO uint32_t FIFO_CFG;                   /*!< (@ 0x400F000C) Configures FIFO fullness level that triggers interrupt and packing 1 or 2 samples per word. */
  __O  uint32_t TRIGGER;                    /*!< (@ 0x400F0010) Enable software trigger to start descriptor processing */
  __IO uint32_t DSCR_STS;                   /*!< (@ 0x400F0014) Indicates active descriptor table and descriptor entry */
  __IO uint32_t POWER_DOWN;                 /*!< (@ 0x400F0018) Set or clear power down mode */
  __IO uint32_t CONFIG;                     /*!< (@ 0x400F001C) Configures external trigger mode, store channel ID in FIFO and wakeup recovery time from power down. */
  __IO uint32_t THR_A;                      /*!< (@ 0x400F0020) Configures window comparator A levels. */
  __IO uint32_t THR_B;                      /*!< (@ 0x400F0024) Configures window comparator B levels. */
  __I  uint32_t LAST_SAMPLE[6];             /*!< (@ 0x400F0028)	Contains last converted sample of input M [M=0..5) and result of window comparator. */
  __I  uint32_t RESERVED0[48];
  __IO uint32_t ADC_DEBUG;                  /*!< (@ 0x400F0100) Reserved  (ADC Debug pin inputs) */
  __IO uint32_t ADC_SPEED;                  /*!< (@ 0x400F0104) ADC speed control */
  __IO uint32_t POWER_CONTROL;              /*!< (@ 0x400F0108) Configures ADC power vs. speed, DC-in biasing, output format and power gating. */
  __I  uint32_t RESERVED1[61];
  __I  uint32_t FIFO_OUTPUT[16];            /*!< (@ 0x400F0200 - 0x400F023C) FIFO output mapped to 16 consecutive address locations. An output contains the value and input channel ID of one or two converted samples  */
  __I  uint32_t RESERVED2[48];
  __IO uint32_t DESCRIPTOR_0[8];            /*!< (@ 0x400F0300) Table0  descriptor n, n= 0 to 7  */
  __IO uint32_t DESCRIPTOR_1[8];            /*!< (@ 0x400F0320) Table1  descriptor n, n= 0 to 7  */
  __I  uint32_t RESERVED3[752];
  __O  uint32_t CLR_EN0;                    /*!< (@ 0x400F0F00) Interrupt0 clear mask */
  __O  uint32_t SET_EN0;                    /*!< (@ 0x400F0F04) Interrupt0 set mask */
  __I  uint32_t MASK0;                      /*!< (@ 0x400F0F08) Interrupt0 mask */
  __I  uint32_t STATUS0;                    /*!< (@ 0x400F0F0C) Interrupt0 status. Interrupt0 contains FIFO fullness, descriptor status and ADC range under/overflow */
  __O  uint32_t CLR_STAT0;                  /*!< (@ 0x400F0F10) Interrupt0 clear status  */
  __O  uint32_t SET_STAT0;                  /*!< (@ 0x400F0F14) Interrupt0 set status  */
  __I  uint32_t RESERVED4[2];
  __O  uint32_t CLR_EN1;                    /*!< (@ 0x400F0F20) Interrupt1 mask clear enable.  */
  __O  uint32_t SET_EN1;                    /*!< (@ 0x400F0F24) Interrupt1 mask set enable  */
  __I  uint32_t MASK1;                      /*!< (@ 0x400F0F28) Interrupt1 mask */
  __I  uint32_t STATUS1;                    /*!< (@ 0x400F0F2C) Interrupt1 status. Interrupt1 contains window comparator results and register last LAST_SAMPLE[M] overrun. */
  __O  uint32_t CLR_STAT1;                  /*!< (@ 0x400F0F30) Interrupt1 clear status  */
  __O  uint32_t SET_STAT1;                  /*!< (@ 0x400F0F34) Interrupt1 set status  */
} LPC_VADC_Type;

#define LPC_VADC_BASE             0x400F0000
#define LPC_VADC                  ((LPC_VADC_Type           *) LPC_VADC_BASE)

#define CGU_BASE_VADC CGU_BASE_ENET_CSR
#define VADC_IRQn RESERVED7_IRQn

#define VADC_DMA_WRITE  7
#define VADC_DMA_READ   8
#define VADC_DMA_READ_SRC  (LPC_VADC_BASE + 512)  /* VADC FIFO_OUTPUT0 0x400F 0200 */

#define RGU_SIG_VADC 60


#define STATUS0_FIFO_FULL_MASK      (1<<0)
#define STATUS0_FIFO_EMPTY_MASK     (1<<1)
#define STATUS0_FIFO_OVERFLOW_MASK  (1<<2)
#define STATUS0_DESCR_DONE_MASK     (1<<3)
#define STATUS0_DESCR_ERROR_MASK    (1<<4)
#define STATUS0_ADC_OVF_MASK        (1<<5)
#define STATUS0_ADC_UNF_MASK        (1<<6)

#define STATUS0_CLEAR_MASK          0x7f

#define STATUS1_THCMP_BRANGE(__ch)  ((1<<0) << (5 * (__ch)))
#define STATUS1_THCMP_ARANGE(__ch)  ((1<<1) << (5 * (__ch)))
#define STATUS1_THCMP_DCROSS(__ch)  ((1<<2) << (5 * (__ch)))
#define STATUS1_THCMP_UCROSS(__ch)  ((1<<3) << (5 * (__ch)))
#define STATUS1_THCMP_OVERRUN(__ch) ((1<<4) << (5 * (__ch)))

#define STATUS1_CLEAR_MASK          0x1fffffff

////////////////////////////////DMA-CLK設定////////////////////////////
#define PLL0_MSEL_MAX (1<<15)
#define PLL0_NSEL_MAX (1<<8)
#define PLL0_PSEL_MAX (1<<5)
static uint32_t FindMDEC(uint32_t msel)
{
	//データシートのサンプルコードそのまま
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
void setup_pll0audio(uint32_t msel, uint32_t nsel, uint32_t psel)
{
  uint32_t ClkSrc;

  /* source = XTAL OSC 12 MHz, enumで6が入ってる*/
  ClkSrc = CGU_CLKSRC_XTAL_OSC;
  /* crystal oscillator power down */
  LPC_CGU->PLL0AUDIO_CTRL = (ClkSrc << 24) | _BIT(0);
  /* set NDEC, PDEC register */
  LPC_CGU->PLL0AUDIO_NP_DIV = (FindNDEC(nsel)<<12) | (FindPDEC(psel) << 0);
  /* set MDEC register */
  LPC_CGU->PLL0AUDIO_MDIV = FindMDEC(msel);
  //MDECを有効にし，シグマデルタモジュールをOFFにして水晶をPLLに接続
  LPC_CGU->PLL0AUDIO_CTRL = (ClkSrc << 24) | (6<< 12);     // fractional divider off and bypassed
  /* enable PLL0 clock output */
  LPC_CGU->PLL0AUDIO_CTRL |= (1<<4); /* CLKEN */

  CGU_ClockSourceFrequency[CGU_CLKSRC_PLL0_AUDIO] =
    msel * (CGU_ClockSourceFrequency[ClkSrc] / (psel * nsel));
}

////////////////////////////////ADCコントロール////////////////////////////
#define CAPTUREBUFFER_SIZE	0x4000 //0x10000(64kB)から0x4000(16kB=8kBx2)に変更
#define CAPTUREBUFFER0		((uint8_t*)0x20000000)//16kB AHB SRAMx2
#define CAPTUREBUFFER1		((uint8_t*)0x20008000)//16kB AHB SRAMx2
#define CAPTUREBUFFER_SIZEHALF	0x8000
#define ADCCLK_MATCHVALUE	(500 - 1)  // サンプリング周期 = PLL0AUDIO / 500 = 79.872ksps
#define ADCCLK_DGECI 0
#define FIFO_SIZE       8
#define DMA_LLI_NUM    16//リンクリストの個数．最低４個くらい必要
static GPDMA_LLI_Type DMA_LTable[DMA_LLI_NUM];//リンクリストの個数分，設定用構造体を定義
volatile int32_t capture_count;
void VADC_SetupDMA(void)
{
  int i;
  uint32_t transfersize;
  uint32_t blocksize;
  uint8_t *buffer;

  //DMA割り込みディスエーブル
  NVIC_DisableIRQ(DMA_IRQn);
  //DMAch0ディスエーブル
  LPC_GPDMA->C0CONFIG = 0;

  /* clear all interrupts on channel 0 */
  LPC_GPDMA->INTTCCLEAR = 0x01;//ターミナルカウントリクエストクリア
  LPC_GPDMA->INTERRCLR = 0x01;//割り込みエラークリア

  /* Setup the DMAMUX */
  LPC_CREG->DMAMUX &= ~(0x3<<(VADC_DMA_WRITE*2));//DMAペリフェラル7をADCHS書き込みに接続(一回クリアして書き直し)
  LPC_CREG->DMAMUX |= 0x3<<(VADC_DMA_WRITE*2);  /* peripheral 7 vADC Write(0x3) */
  LPC_CREG->DMAMUX &= ~(0x3<<(VADC_DMA_READ*2));//DMAペリフェラル8をADCHS読み込みに接続(一回クリアして書き直し)
  LPC_CREG->DMAMUX |= 0x3<<(VADC_DMA_READ*2);  /* peripheral 8 vADC read(0x3) */

  LPC_GPDMA->CONFIG = 0x01;  /* Enable DMA channels, little endian */

  // The size of the transfer is in multiples of 32bit copies (hence the /4)
  // and must be even multiples of FIFO_SIZE.
  buffer = CAPTUREBUFFER0;
  blocksize = CAPTUREBUFFER_SIZE / DMA_LLI_NUM;
  transfersize = blocksize / 4;

  //分割されたバッファごとに転送．
  for (i = 0; i < DMA_LLI_NUM; i++)
  {
	if (i == DMA_LLI_NUM / 2)//リンクリストの半分まで来たらBUFFER0からBUFFER1にSRAMアドレスを切り替える
		buffer = CAPTUREBUFFER1;
	//0x4000 210C(DMA Channel Control register)参照
	DMA_LTable[i].SrcAddr = VADC_DMA_READ_SRC;
	DMA_LTable[i].DstAddr = (uint32_t)buffer;
	DMA_LTable[i].NextLLI = (uint32_t)(&DMA_LTable[(i+1) % DMA_LLI_NUM]);//次ブロックのポインタを指定(環状リンクリスト)
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
  //リンクリストの真ん中(BUFFER0満杯)と最後(BUFFFER1満杯)で計２回割り込み(要データ退避)するよう設定．
  DMA_LTable[DMA_LLI_NUM/2 - 1].Control |= (0x1UL << 31); // Terminal count interrupt enabled
  DMA_LTable[DMA_LLI_NUM - 1].Control |= (0x1UL << 31); // Terminal count interrupt enabled

  //上で構造体で定義した最初のリンクリストの設定をレジスタに入れ込み
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
  //DMA割り込みイネーブル
  NVIC_EnableIRQ(DMA_IRQn);
}

void VADC_Init(void)
{
  CGU_EntityConnect(CGU_CLKSRC_PLL0_AUDIO, CGU_BASE_VADC);//VADCをPLL0AUDIOに接続
  CGU_EnableEntity(CGU_BASE_VADC, ENABLE);//VADCに電源供給

  RGU_SoftReset(RGU_SIG_VADC);  // Reset the VADC block
  while(RGU_GetSignalStatus(RGU_SIG_VADC));//リセット解除待ち

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
  //32bitx8回(16回分のデータ)のサンプリングが終わったらDMA転送
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

  //デバイス内のDCバイアス設定(データシート通りの動作にならない．よくわからん．)
  LPC_VADC->POWER_CONTROL =
    (0 /*crs*/ << 0) |    /* CRS:          current setting for power versus speed programming */
    (1 << 4) |      /* DCINNEG:      0=no dc bias, 1=dc bias(0.5V) on vin_neg slide */
    (1 << 10) |     /* DCINPOS:      0=no dc bias, 1=dc bias(0.5V) on vin_pos slide */
    (0 << 16) |     /* TWOS:         0=offset binary0x000~0xFFF, 1=two's complement */
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

  NVIC_DisableIRQ(DMA_IRQn);

  LPC_VADC->TRIGGER = 0;
  // Clear FIFO
  LPC_VADC->FLUSH = 1;
  // power down VADC
  LPC_VADC->POWER_CONTROL = 0;

  // Reset the VADC block
  RGU_SoftReset(RGU_SIG_VADC);
  while(RGU_GetSignalStatus(RGU_SIG_VADC));
}
