#include "motor_control.h"

#include "filter.h"

#define SAMPLES_SIZE 10

Filter _speed_filter;
uint16_t _speed_samples[SAMPLES_SIZE];
percent_t _speed_limite_ratio;

QueueHandle_t user_speed_cmd_2_motor_ctrl_queue; // speed_command_tsk => motor_control_tsk
QueueHandle_t user_speed_cmd_2_communication_queue; // speed_command_tsk => communication_tsk

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

    user_speed_cmd_2_motor_ctrl_queue = xQueueCreate(1, sizeof(Speed_Data));
    user_speed_cmd_2_communication_queue = xQueueCreate(1, sizeof(Speed_Data));
}

void motor_control_set_speed_limite_ratio(percent_t speed_limite_ratio)
{
    _speed_limite_ratio = speed_limite_ratio;
}

void motor_control_reset();

//send the speed to the motor
void motor_control_task(void *pvParameters);

//get the speed from the user, scale it to a ratio and send it to motor_control_task
void user_speed_command_task(void *pvParameters);
