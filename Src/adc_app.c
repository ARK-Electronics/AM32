/*
 * adc_app.c - finish ADC sample + scale electrical quantities
 *
 * Vendor branches were previously inlined in runtime_loop.c. Policy (LVC)
 * stays in runtime_loop; this file only samples and scales.
 */

#include "adc_app.h"

#include "ADC.h"
#include "motor_runtime.h"
#include "control_loop.h"
#include "targets.h"
#include "common.h"

extern int16_t converted_degrees;
extern int32_t smoothed_raw_current;
extern uint16_t VOLTAGE_DIVIDER;

void adcAppServiceConversion(void)
{
#if defined(STMICRO)
	ADC_DMA_Callback();
	LL_ADC_REG_StartConversion(ADC1);
#	ifdef USE_ADC_1_2
	LL_ADC_REG_StartConversion(ADC2);
#	endif
	converted_degrees = (int16_t)__LL_ADC_CALC_TEMPERATURE(3300, ADC_raw_temp, LL_ADC_RESOLUTION_12B);
#endif

#ifdef MCU_GDE23
	ADC_DMA_Callback();
	converted_degrees = (int16_t)(((int32_t)(357.5581395348837f * (1 << 16)) -
				       (int32_t)ADC_raw_temp * (int32_t)(0.18736373546511628f * (1 << 16))) >>
				      16);
	adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
#endif

#ifdef ARTERY
	ADC_DMA_Callback();
	adc_ordinary_software_trigger_enable(ADC1, TRUE);
#	ifdef USE_NTC
	converted_degrees = getNTCDegrees(ADC_raw_ntc);
#	else
	converted_degrees = getConvertedDegrees(ADC_raw_temp);
#	endif
#endif

#ifdef NXP
	ADC_DMA_Callback();
	converted_degrees = computeTemperature(ADC_raw_temp[0], ADC_raw_temp[1]);
	startADCConversion();
#endif

#ifdef WCH
	startADCConversion();
	converted_degrees = getConvertedDegrees(ADC_raw_temp);
#endif

	degrees_celsius = converted_degrees;

#ifdef NXP
	battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 65535 * VOLTAGE_DIVIDER) / 100)) / 8;
	smoothed_raw_current = getSmoothedCurrent();
	actual_current = (((smoothed_raw_current * 3300 / 65535) - CURRENT_OFFSET) * 100) / (MILLIVOLT_PER_AMP);
#else
	battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER) / 100)) >> 3;
	smoothed_raw_current = getSmoothedCurrent();
	actual_current = ((smoothed_raw_current * 3300 / 41) - (CURRENT_OFFSET * 100)) / (MILLIVOLT_PER_AMP);
#endif
	if (actual_current < 0) {
		actual_current = 0;
	}
}
