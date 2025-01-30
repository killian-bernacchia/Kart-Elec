#if !defined(MOTOR_CONTROL_H)
#define MOTOR_CONTROL_H

/********************************
 * Includes
 */
#include "common.h"

/********************************
 * Defines
 */

#define MAX_SPEED 255
#define SLEEP_SPEED 80 // minimum speed before the motor starts (real value is 100 but we keep a margin)
#define MIN_SPEED 35 // lesser speeds are seen as 'no signals'

/********************************
 * Functions prototype
 */

void motor_control_init(percent_t max_speed_rate);
void motor_control_set_max_speed_rate(percent_t max_speed_rate);
void motor_control_reset();

//send the speed to the motor
void motor_control_task(void *pvParameters);

//get the speed from the user, scale it to a ratio and send it to motor_control_task
void speed_command_task(void *pvParameters);


#endif // MOTOR_CONTROL_H
