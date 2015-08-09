#include <lpc43xx.h>
#include <lpc43xx_cgu.h>

#include "adc_dma.h"


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
