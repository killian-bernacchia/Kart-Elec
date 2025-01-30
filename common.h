#if !defined(COMMON_H)
#define COMMON_H

/********************************
 * -
 */

/********************************
 * Includes
 */

#include <Arduino.h>
//#include <Wire.h>
//#include <LiquidCrystal_I2C.h>

/********************************
 * Defines
 */

#define ADC_RESOLUTION 12
#define ADC_MAX_VALUE ((1 << ADC_RESOLUTION) - 1)

#define ADC_MOTOR_PIN 25
#define ADC_PEDAL_PIN 34
#define ADC_HEAT_PIN 15

#define __SAFE_WHILE(CONDITION, MAX, UNIQUE_NAME) \
    for (int UNIQUE_NAME = 0; (CONDITION) && UNIQUE_NAME < (MAX); UNIQUE_NAME++)

#define safe_while(CONDITION, MAX) \
    __SAFE_WHILE((CONDITION), (MAX), _i_##__COUNTER__)
/********************************
 * typedefs
 */

typedef float percent_t;

typedef struct ADC_Data{
    uint16_t raw;
    float voltage;
} ADC_Data;

typedef struct Heat_Data{
    ADC_Data raw;
    percent_t ratio;
    float temperature;
} Heat_Data;

typedef struct Speed_Data{
    ADC_Data raw;
    percent_t ratio;
    percent_t final_ratio;
} Speed_Data;

/********************************
 * Task Handles
 */

extern TaskHandle_t heat_control_tsk_hndl; // get the temperature
extern TaskHandle_t user_speed_command_tsk_hndl; // get the speed
extern TaskHandle_t motor_control_tsk_hndl; // send the speed
extern TaskHandle_t communication_tsk_hndl; // send the data
extern TaskHandle_t motor_speed_mode_tsk_hndl; // change motor mode

/********************************
 * Queues
 */
extern QueueHandle_t speed_cmd_2_motor_ctrl_queue; // speed_command_tsk => motor_control_tsk
extern QueueHandle_t heat_ctrl_2_motor_ctrl_queue; // heat_control_tsk => motor_control_tsk
extern QueueHandle_t speed_cmd_2_communication_queue; // speed_command_tsk => communication_tsk
extern QueueHandle_t heat_ctrl_2_communication_queue; // heat_control_tsk => communication_tsk

/********************************
 * Semaphores
 */
extern SemaphoreHandle_t ADC_mutex;

#endif // COMMON_H