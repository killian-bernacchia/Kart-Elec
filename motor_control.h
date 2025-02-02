#if !defined(MOTOR_CONTROL_H)
#define MOTOR_CONTROL_H

/********************************
 * Includes
 */
#include "common.h"

/********************************
 * Defines
 */

#define SPEED_MAX_SIGNAL (255<<4)
#define SPEED_SLEEP_SIGNAL (80<<4) // minimum speed before the motor starts (real value is 100 but we keep a margin)
#define SPEED_NO_SIGNAL (35<<4) // lesser speeds are seen as 'no signals'

/********************************
 * Functions prototype
 */

void motor_control_init(percent_t speed_limite_ratio);
void motor_control_set_speed_limite_ratio(percent_t speed_limite_ratio);
void motor_control_reset();

//send the speed to the motor
void motor_control_task(void *pvParameters);

//get the speed from the user, scale it to a ratio and send it to motor_control_task
void user_speed_command_task(void *pvParameters);


#endif // MOTOR_CONTROL_H
