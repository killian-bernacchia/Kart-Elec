#include "heat_control.h"

#include "stdint.h"

#include "filter.h"

#define SAMPLES_SIZE 20
#define NO_SIGNAL_VOLTAGE 0.0067f // -20°C

Filter _heat_filter;
uint16_t _heat_samples[SAMPLES_SIZE];

QueueHandle_t heat_ctrl_2_motor_ctrl_queue;
QueueHandle_t heat_ctrl_2_communication_queue;

float _temperature;
bool manual_mode;

static void ErrorNoSignal(Heat_Data heat_data);

void ErrorOverheat(Heat_Data heat_data);

void heat_control_init()
{
    manual_mode = false;
    FilterInit(&_heat_filter, _heat_samples, SAMPLES_SIZE);

    for (int i = 0; i < SAMPLES_SIZE; i++)
    {
        uint16_t VIN;
        xSemaphoreTake(ADC_mutex, portMAX_DELAY);
        VIN = analogRead(ADC_HEAT_PIN);
        xSemaphoreGive(ADC_mutex);
        FilterPush(&_heat_filter, VIN);
    }

    heat_ctrl_2_motor_ctrl_queue = xQueueCreate(1, sizeof(Heat_Data));
    heat_ctrl_2_communication_queue = xQueueCreate(1, sizeof(Heat_Data));
}

void heat_control_reset();

//for testing purposes - soon will be set remotely
void heat_control_manual_mode()
{
    manual_mode = true;
    _temperature = START_SAFE_TEMPERATURE;
}

void heat_control_set_temperature(float temperature)
{
    _temperature = temperature;
}

void heat_control_automatic_mode()
{
    manual_mode = false;
}

//get the temperature, scale it to a ratio and send it to motor_control_task and communication_task
void heat_control_task(void *pvParameters)
{
    Heat_Data heat_data;
    uint16_t VIN;
    float temperatureK;
    float R2;
    percent_t ratio;

    while(1)
    {
        xSemaphoreTake(ADC_mutex, portMAX_DELAY);
        VIN = analogRead(ADC_HEAT_PIN);
        xSemaphoreGive(ADC_mutex);

        heat_data.adc.raw = VIN;
        heat_data.adc.voltage = (VIN*ADC_VOLTAGE)/ADC_MAX_VALUE;
        FilterPush(&_heat_filter, VIN);
        heat_data.adc.filtered = FilterDoubledMeanFloat(&_heat_filter, 1.2f);

        if ( heat_data.adc.voltage < NO_SIGNAL_VOLTAGE )
        {
            heat_data.adc.no_signal_count++;
        }
        else
        {
            if ( heat_data.adc.no_signal_count > 0)
              heat_data.adc.no_signal_count--;
        }

        if( heat_data.adc.no_signal_count > SAMPLES_SIZE )
        {
            ErrorNoSignal(heat_data);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        else
        {
            R2 = 4700.0 * ((ADC_VOLTAGE / heat_data.adc.voltage) - 1);
            temperatureK = 1 / ((1 / 298.15) + (1 / 3950.0) * log(R2 / 10000.0));
            heat_data.temperature = temperatureK - 273.15;

            if ( heat_data.temperature > OVERHEAT_TEMPERATURE)
            {
                ErrorOverheat(heat_data);
                vTaskDelay(1 / portTICK_PERIOD_MS);
            }
            else if ( heat_data.temperature <= START_SAFE_TEMPERATURE)
            {
                ratio = 1.0f;
            }
            else if ( heat_data.temperature >= MAX_SAFE_TEMPERATURE)
            {
                ratio = MINIMAL_SAFE_RATIO;
            }
            else
            {
                ratio = (heat_data.temperature - START_SAFE_TEMPERATURE)/(MAX_SAFE_TEMPERATURE - START_SAFE_TEMPERATURE);
                ratio = MINIMAL_SAFE_RATIO + ratio * (1.0f - MINIMAL_SAFE_RATIO);
            }
                if(ratio > 1.0f) ratio = 1.0f;
                heat_data.ratio = ratio;
                xQueueOverwrite(heat_ctrl_2_motor_ctrl_queue, &heat_data);
                xQueueOverwrite(heat_ctrl_2_communication_queue, &heat_data);
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

static void ErrorNoSignal(Heat_Data heat_data)
{
    motor_is_unsafe = true;
    //if ( heat_data.adc.no_signal_count < 2*SAMPLES_SIZE )
        //dacWrite(ADC_MOTOR_PIN, SPEED_SLEEP_SIGNAL);
    //else
        //dacWrite(ADC_MOTOR_PIN, 0);
}

void ErrorOverheat(Heat_Data heat_data)
{
    motor_is_unsafe = true;
    //dacWrite(ADC_MOTOR_PIN, SPEED_SLEEP_SIGNAL);
}