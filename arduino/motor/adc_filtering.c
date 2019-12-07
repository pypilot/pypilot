#include "adc_filtering.h"
#include "pins.h"

uint16_t adcChannels[5] = {0,0,0,0,0};
uint16_t adcChannelHistory[5] = {0,0,0,0,0};

void ADC_initFilters(void)
{
#ifndef DISABLE_RUDDER_SENSE
    adcChannels[0]        = analogRead(RUDDER_SENSE_PIN) << 6;
    adcChannelHistory[0]  = adcChannels[0];
#endif
#ifndef DISABLE_VOLTAGE_SENSE
    adcChannels[1]        = analogRead(VOLTAGE_SENSE_PIN);
    adcChannelHistory[1]  = adcChannels[1];
#endif
#ifndef DISABLE_CURRENT_SENSE
    adcChannels[2]        = analogRead(CURRENT_SENSE_PIN_1);
    adcChannelHistory[2]  = adcChannels[2];
#endif
#ifndef DISABLE_TEMP_SENSE
    adcChannels[3]        = analogRead(TEMPERATURE_CONTROLLER_SENSE_PIN);
    adcChannelHistory[3]  = adcChannels[3];
#endif
#ifndef DISABLE_TEMP_SENSE
    adcChannels[4]        = analogRead(TEMPERATURE_MOTOR_SENSE_PIN);
    adcChannelHistory[4]  = adcChannels[4];
#endif
}

/*
 * This function collects a number of samples per ADC channel and filters by averaging the values.
 * Per channel, a different filtering method can be implemented.
 * Current and voltage spikes are desireable to see when setting up PyPilot.
 * Noise on the rudder sensor not so much.
 */
void ADC_updateAndFilter(void)
{
  uint8_t filterNumber = 0;
  uint16_t adcInputValue = 0;
#ifdef BOARD_IBT2_H_BRIDGE
  uint16_t adcInputValue2 = 0; // only used for current sensing on the second channel
#endif
  
  switch (filterNumber)
  {
    case RUDDER_ANGLE:
#ifndef DISABLE_RUDDER_SENSE
      adcInputValue = analogRead(RUDDER_SENSE_PIN) << 6;
      adcChannels[0] = (uint16_t)(ALPHA_RUDDER * (float)adcInputValue + (1 - ALPHA_RUDDER) * (float)adcChannelHistory[0]);
      adcChannelHistory[0] = adcChannels[0];
#else
      adcInputValue = 0;
#endif
      filterNumber++;
      
    case SUPPLY_VOLTAGE:
#ifndef DISABLE_VOLTAGE_SENSE
      adcInputValue = analogRead(VOLTAGE_SENSE_PIN);
      adcChannels[1] = (uint16_t)(ALPHA_VOLTAGE * (float)adcInputValue + (1 - ALPHA_VOLTAGE) * (float)adcChannelHistory[1]);
      adcChannelHistory[1] = adcChannels[1];
#else
      adcInputValue = 0;
#endif
      filterNumber++;
      
    case MOTOR_CURRENT:
#ifndef DISABLE_CURRENT_SENSE
      adcInputValue = analogRead(CURRENT_SENSE_PIN_1); // read in one channel for current sensing
      
#ifdef BOARD_IBT2_H_BRIDGE
  uint16_t adcInputValue2 = analogRead(CURRENT_SENSE_PIN_2); // read second channel for current sensing the IBT_2 h_bridge driver
/*  
 *   Since we only store one current sensor value and filter on that, we need to decide which one we want to follow.
 *   In order to do that, check whether one is larger than the other and select the larger one.
 */
  adcInputValue = adcInputValue > adcInputValue2 ? adcInputValue : adcInputValue2;
#endif

      adcChannels[2] = (uint16_t)(ALPHA_CURRENT * (float)adcInputValue + (1 - ALPHA_CURRENT) * (float)adcChannelHistory[2]);
      adcChannelHistory[2] = adcChannels[2];
#else
      adcInputValue = 0;
#endif
      filterNumber++;
      
    case CONTROLLER_TEMPERATURE:
#ifndef DISABLE_TEMP_SENSE
      adcInputValue = analogRead(TEMPERATURE_CONTROLLER_SENSE_PIN);
      adcChannels[3] = (uint16_t)(ALPHA_TEMPCNTRL * (float)adcInputValue + (1 - ALPHA_TEMPCNTRL) * (float)adcChannelHistory[3]);
      adcChannelHistory[3] = adcChannels[3];
#else
      adcInputValue = 0;
#endif
      filterNumber++;
      
    case MOTOR_TEMPERATURE:
#ifndef DISABLE_TEMP_SENSE
      adcInputValue = analogRead(TEMPERATURE_MOTOR_SENSE_PIN);
      adcChannels[4] = (uint16_t)(ALPHA_TEMPMOTOR * (float)adcInputValue + (1 - ALPHA_TEMPMOTOR) * (float)adcChannelHistory[4]);
      adcChannelHistory[4] = adcChannels[4];
#else
      adcInputValue = 0;
#endif
      break;
    default:
      break;
  }
}

/*
 * Returns a signed 16 bit value as a filtered ADC value. Averaged over a number of samples to reduce noise.
 */
uint16_t getADCFilteredValue(uint8_t channel)
{
  switch (channel)
  {
    case RUDDER_ANGLE:
      return adcChannels[0];
      break;
    case SUPPLY_VOLTAGE:
      return adcChannels[1];
      break;
    case MOTOR_CURRENT:
      return adcChannels[2];
      break;
    case CONTROLLER_TEMPERATURE:
      return adcChannels[3];
      break;
    case MOTOR_TEMPERATURE:
      return adcChannels[4];
      break;
    default:
      return 0;
      break;
  }
}

uint16_t rescaleValue16(uint16_t input, uint32_t minRange, uint32_t maxRange, uint32_t desiredRange)
{
  if (input < minRange)
    return 0;
  else if (input > maxRange)
    return desiredRange;
  
  uint32_t fullRange = maxRange - minRange;
  uint16_t factor = (uint16_t)((desiredRange << 10) / fullRange);
  
  return (uint16_t)(((uint32_t)(input - minRange) * factor) >> 10);
}
