#ifndef __WAVE_GEN_H__
#define __WAVE_GEN_H__

#include <lpc43xx.h>

/*! \name Types of waveforms, used in \ref gen_dac_cfg_t
 * \{
 */
#define GEN_DAC_CFG_WAVE_SINUS         0
#define GEN_DAC_CFG_WAVE_SQUARE        1
#define GEN_DAC_CFG_WAVE_TRIANGLE      2
#define GEN_DAC_CFG_WAVE_SAWTOOTH      3
#define GEN_DAC_CFG_WAVE_INV_SAWTOOTH  4
#define GEN_DAC_CFG_WAVE_LEVEL         5
/* \} */

/*! @brief Configuration of one analog signal to generate.
 */
typedef struct
{
  /*! @brief Type of waveform to generate.
   *
   * Value | Waveform type
   * :---: | -------------
   *   0   | Sine
   *   1   | Square
   *   2   | Triangle
   *   3   | Sawtooth
   *   4   | Reverse (or inverse) Sawtooth
   *   5   | Level (outputs DC offset, ignores amplitude)
   */
  uint32_t waveform;
  uint32_t frequency; /*!< Frequency in Hz */
  uint32_t amplitude; /*!< Amplitude in mV, 0..5000 */
  int32_t  dcOffset;  /*!< DC offset in mV, -2500..2500 */
} gen_dac_cfg_t;

/*! Size of lookup table for waveform data */
#define MAX_LUT_SIZE  4096
/*! @brief Configuration of one analog output. */
typedef struct
{
  /*! Lookup table for waveform data */
  uint16_t LUT_BUFFER[MAX_LUT_SIZE];

  /*! Current number of entries in the lookup table */
  uint16_t numLUTEntries;
} dac_buffer_t;

void wave_gen(const gen_dac_cfg_t *, dac_buffer_t *);

#endif
