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
int16_t getADCFilteredValue(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif 
