#include "motor_control.h"

#include "filter.h"

#define SAMPLES_SIZE 10
#define PEDAL_CORRECTION_FACTOR 1.4f

static void ErrorNoSignal(Speed_Data speed_data);

Filter _speed_filter;
uint16_t _speed_samples[SAMPLES_SIZE]; 
percent_t _speed_limite_ratio;
bool motor_is_unsafe;

QueueHandle_t user_speed_cmd_2_motor_ctrl_queue; // speed_command_tsk => motor_control_tsk
QueueHandle_t motor_ctrl_queue_cmd_2_communication_queue; // speed_command_tsk => communication_tsk

void motor_control_init(percent_t speed_limite_ratio)
{
    _speed_limite_ratio = speed_limite_ratio;
    FilterInit(&_speed_filter, _speed_samples, SAMPLES_SIZE);

    for (int i = 0; i < SAMPLES_SIZE; i++)
    {
        uint16_t VIN;
        xSemaphoreTake(ADC_mutex, portMAX_DELAY);
        VIN = analogRead(ADC_PEDAL_PIN);
        xSemaphoreGive(ADC_mutex);
        FilterPush(&_speed_filter, VIN);
    }

    motor_is_unsafe = false;
    user_speed_cmd_2_motor_ctrl_queue = xQueueCreate(1, sizeof(Speed_Data));
    motor_ctrl_queue_cmd_2_communication_queue = xQueueCreate(1, sizeof(Speed_Data));
}

void motor_control_set_speed_limite_ratio(percent_t speed_limite_ratio)
{
    _speed_limite_ratio = speed_limite_ratio;
}

void motor_control_reset();

//send the speed to the motor
void motor_control_task(void *pvParameters)
{
    Speed_Data speed_data;
    Heat_Data heat_data;

    while(1)
    {
        if ( motor_is_unsafe )
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        while( xQueueReceive(user_speed_cmd_2_motor_ctrl_queue, &speed_data, 0) == pdFALSE && xQueueReceive(heat_ctrl_2_motor_ctrl_queue, &heat_data, 0) == pdFALSE )
        {
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }

        speed_data.final_ratio = speed_data.ratio * heat_data.ratio * _speed_limite_ratio;
        speed_data.speed = speed_data.final_ratio * DAC_MAX_VALUE;
        if ( speed_data.speed > DAC_MAX_VALUE )
            speed_data.speed = DAC_MAX_VALUE;
        if ( speed_data.speed < 0 )
            speed_data.speed = 0;
        dacWrite(ADC_MOTOR_PIN, speed_data.speed);
        
        xQueueOverwrite(motor_ctrl_queue_cmd_2_communication_queue, &speed_data);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

//get the speed from the user, scale it to a ratio and send it to motor_control_task
void user_speed_command_task(void *pvParameters)
{    
    Speed_Data speed_data;
    uint16_t VIN;
    float speed;
    percent_t ratio;

    while(1)
    {
        xSemaphoreTake(ADC_mutex, portMAX_DELAY);
        VIN = analogRead(ADC_PEDAL_PIN);
        xSemaphoreGive(ADC_mutex);

        speed_data.adc.raw = VIN;
        speed_data.adc.voltage = (VIN*ADC_VOLTAGE)/ADC_MAX_VALUE;
        FilterPush(&_speed_filter, VIN);
        speed_data.adc.filtered = FilterDoubledMeanFloat(&_speed_filter, 1.2f);

        if ( VIN < SPEED_NO_SIGNAL )
        {
            speed_data.adc.no_signal_count++;
        }
        else
        {
            if ( speed_data.adc.no_signal_count > 0)
              speed_data.adc.no_signal_count--;
        }

        if( speed_data.adc.no_signal_count > SAMPLES_SIZE )
        {
            ErrorNoSignal(speed_data);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        else
        {
            speed = PEDAL_CORRECTION_FACTOR * speed_data.adc.filtered;
            if ( speed > SPEED_SLEEP_SIGNAL)
            {
                ratio = (speed - SPEED_SLEEP_SIGNAL)/(SPEED_MAX_SIGNAL - SPEED_SLEEP_SIGNAL);
                if(ratio > 1.0f) ratio = 1.0f;
                speed_data.ratio = ratio;
            }
            else
            {
                speed_data.ratio = 0.0f;
            }
            xQueueOverwrite(user_speed_cmd_2_motor_ctrl_queue, &speed_data);
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

static void ErrorNoSignal(Speed_Data speed_data)
{
    motor_is_unsafe = true;
    if ( speed_data.adc.no_signal_count < 2*SAMPLES_SIZE )
        dacWrite(ADC_MOTOR_PIN, SPEED_SLEEP_SIGNAL);
    else
        dacWrite(ADC_MOTOR_PIN, 0);
}
