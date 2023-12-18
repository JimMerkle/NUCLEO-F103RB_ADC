// Copyright Jim Merkle, 12/15/2023
// File: adc.c
//
// Interact with the ADC on the STM32F103RB
//


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h> // uint8_t
#include "command_line.h"
#include "main.h"   // HAL functions and defines

extern ADC_HandleTypeDef hadc1; // main.c

// The ADC should already be started after calling HAL_ADCEx_Calibration_Start() in main.c
int cl_adc(void) {

	// Measure and report the reference voltage
	for(unsigned i = 0; i<10; i++) {
		HAL_Delay(5);
		HAL_ADC_Start(&hadc1);
		HAL_Delay(5);
		HAL_StatusTypeDef hal_rc = HAL_ADC_PollForConversion(&hadc1,10); // this should take less than 2us
		if(HAL_OK != hal_rc) printf("HAL_ADC_PollForConversion() error: %s\n",PrintHalStatus(hal_rc));
		else {
			uint32_t adc_raw = HAL_ADC_GetValue(&hadc1);
			printf("V: %u\n",(uint16_t)adc_raw);
		}
	} // for-loop
	return 0;
}
