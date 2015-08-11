/*
 * wave_gen.c
 *
 *  Created on: 2015/08/11
 *      Author: 2dice
 */

#include "wave_gen.h"
#include "math.h"

//DAC sample/s
#define DAC_SPS 30000
//DAC voltage(V)
#define DAC_V 5.0
//DAC ダイナミックレンジ
#define DAC_FULSCALE 4096

/**************************************************************************//**
 *
 * @brief  Fills the lookup table with data for the requested waveform.
 *
 * Generates the data for the lookup table based on waveform and amplitude.
 *
 * @param [in]  cfg      Configuration for the waveform (from client)
 * @param [in]  lutSize  Number of entries to use in the lookup table
 * @param [in]  ch       The configuration to update (local)
 *
 *****************************************************************************/
void wave_gen(const gen_dac_cfg_t * const cfg, dac_buffer_t *dacBuffer)
{
  int i;
  int val;
  float dcOffset = cfg->dcOffset / 1000.0;
  float amplitude = cfg->amplitude / 1000.0 * 2;
  uint32_t lutSize;
  lutSize = DAC_SPS/cfg->frequency;

  if (cfg->waveform == GEN_DAC_CFG_WAVE_SINUS)
  {
    for (i = 0; i < lutSize; i++)
    {
      float deg = (i * 360.0)/lutSize;
      float rad = (deg * 3.141519) / 180.0;
      float sin = sinf(rad);

      // change amplitude
      sin = sin * amplitude;

      // apply calibration
      // DACは5V電源(4.096V)を想定
      val = (sin + dcOffset) * (DAC_FULSCALE / 2) / DAC_V + DAC_FULSCALE / 2;
      dacBuffer->LUT_BUFFER[i] = val;
    }
  }
  else if (cfg->waveform == GEN_DAC_CFG_WAVE_TRIANGLE)
  {
    /*
     * Based on http://en.wikipedia.org/wiki/Triangle_wave
     * and the fact that the triangle wave can be the absolute
     * value of the sawtooth wave.
     *
     * x(t) = ABS( 2 * (t/a - FLOOR( t/a - 1/2 )) )
     *
     * with
     *   t = 0..1,
     *   a = num periods (always 1)
     *   x = 0..1..0
     */
    for (i = 0; i < lutSize; i++)
    {
      float t = i/((float)lutSize);
      float a = 1;
      float x = fabs(2 * (t/a - floorf(t/a + 0.5)));

      // move from 0..1 to -1..1 and change amplitude
      x = (x - 0.5) * 2 * amplitude;

      // apply calibration
      val = (x + dcOffset) * (DAC_FULSCALE / 2) / DAC_V + DAC_FULSCALE / 2;

      dacBuffer->LUT_BUFFER[i] = val;
    }
  }
  else if (cfg->waveform == GEN_DAC_CFG_WAVE_SQUARE)
  {
    // Calculate the square wave's high point
    float tmp = (dcOffset + amplitude);

    // apply calibration
    val = tmp * (DAC_FULSCALE / 2) / DAC_V + DAC_FULSCALE / 2;

    for (i = 0; i < lutSize/2; i++)
    {
      dacBuffer->LUT_BUFFER[i] = val;
    }

    // Calculate the square wave's low point
    tmp = (dcOffset - amplitude);

    // apply calibration
    val = tmp * (DAC_FULSCALE / 2) / DAC_V + DAC_FULSCALE / 2;

    for (; i < lutSize; i++)
    {
      dacBuffer->LUT_BUFFER[i] = val;
    }
  }
  else if ((cfg->waveform == GEN_DAC_CFG_WAVE_SAWTOOTH) ||
           (cfg->waveform == GEN_DAC_CFG_WAVE_INV_SAWTOOTH))
  {
    int mul = 1;
    if (cfg->waveform == GEN_DAC_CFG_WAVE_INV_SAWTOOTH)
    {
      mul = -1;
    }

    /*
     * Based on http://en.wikipedia.org/wiki/Sawtooth_wave
     *
     * x(t) = 2 * (t/a - FLOOR( t/a + 1/2 ))
     *
     * with
     *   t = 0..1,
     *   a = num periods (always 1)
     *   x = 0..1..-1..0
     */
    for (i = 0; i < lutSize; i++)
    {
      float t = i/((float)lutSize);
      float a = 1;
      float x = 2 * (t/a - floorf(t/a + 0.5));

      // change amplitude, and correct if it is an inverse sawtooth
      x = x * mul * amplitude;

      // apply calibration
      val = x * (DAC_FULSCALE / 2) / DAC_V + DAC_FULSCALE / 2;

      dacBuffer->LUT_BUFFER[i] = val;
    }
  }
  else if (cfg->waveform == GEN_DAC_CFG_WAVE_LEVEL)
  {
    // Find actual amplitude, which for a level type is based on offset

    // apply calibration
    val = dcOffset * (DAC_FULSCALE / 2) / DAC_V + DAC_FULSCALE / 2;

    dacBuffer->LUT_BUFFER[0] = val;
  }
  dacBuffer->numLUTEntries = lutSize;
}
