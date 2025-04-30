#if !defined(SYSTEM_DATA_H)
#define SYSTEM_DATA_H

#include <stdint.h>
#include <freertos/FreeRTOS.h>

typedef struct ADC_Data
{
	uint16_t raw;
	uint16_t filtered;
} ADC_Data;

typedef struct Control_Unit
{
	ADC_Data adc;
	float ratio;
	SemaphoreHandle_t mutex;
	SemaphoreHandle_t sync;
} Control_Unit;

typedef struct System_Data
{
	Control_Unit speed_command_input;
	Control_Unit heat_input;
	Control_Unit speed_command_output;
	SemaphoreHandle_t adc_mutex;
} System_Data;

extern volatile System_Data system_data;

void SystemDataInitSemaphores(void);

void SystemDataDeleteSemaphores(void);

#endif //SYSTEM_DATA_H