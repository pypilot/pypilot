/*
 * Author: Timo Birnschein (timo.birnschein@googlemail.com)
 * Date: 2019/11/30
 * Notes: This code implements a exponential moving average filter as described at 
 * https://helpful.knobs-dials.com/index.php/Low-pass_filter
 * All channels needed for the PyPolit are being read periodically, filtered and then provided
 * to PyPilot uupon request.
 * The APLHA values per channel can be configured to what is needed by the user.
 * Current needs to be read quicker than anything else here. Temperature can be filtered a lot.
 * Rudder may be reasonably slow.
 */

#ifndef __ADC_FILTERING__
#define __ADC_FILTERING__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ALPHA_RUDDER 0.15f
#define ALPHA_VOLTAGE 0.15f
#define ALPHA_CURRENT 0.3f
#define ALPHA_TEMPCNTRL 0.015f
#define ALPHA_TEMPMOTOR 0.015f

enum {
  RUDDER_ANGLE = 0,
  SUPPLY_VOLTAGE = 1,
  MOTOR_CURRENT = 2,
  CONTROLLER_TEMPERATURE = 3,
  MOTOR_TEMPERATURE = 4
};

extern uint16_t adcChannels[5];
extern uint16_t *adcChannelFilterHistory[5];

void ADC_initFilters(void);
void ADC_updateAndFilter(void);
uint16_t getADCFilteredValue(uint8_t channel);

/*
 * Used to rescale an incorrectly scaled value and set its offset
 */
uint16_t rescaleValue16(uint16_t input, uint32_t minRange, uint32_t maxRange, uint32_t desiredRange);
#ifdef __cplusplus
}
#endif

#endif 
