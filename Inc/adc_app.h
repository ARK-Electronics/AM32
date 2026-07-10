/*
 * adc_app.h - application-level ADC service (temp / bus / current)
 *
 * Conversion kick and temperature math are MCU-specific (see implementation).
 * Call adcAppServiceConversion() once when PROCESS_ADC_FLAG is set.
 */
#ifndef ADC_APP_H_
#define ADC_APP_H_

#include <stdint.h>

/*
 * Finish the pending DMA sample, restart conversion, update:
 *   converted_degrees, degrees_celsius, battery_voltage, actual_current
 * (and smoothed_raw_current). Does not clear PROCESS_ADC_FLAG or run LVC.
 */
void adcAppServiceConversion(void);

#endif /* ADC_APP_H_ */
