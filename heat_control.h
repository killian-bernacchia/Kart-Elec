#if !defined(HEAT_CONTROL_H)
#define HEAT_CONTROL_H

/********************************
 * Includes
 */
#include "common.h"

/********************************
 * Defines
 */

#define OVERHEAT_TEMPERATURE 90.0f   //
#define MAX_SAFE_TEMPERATURE 85.0f   // °C
#define START_SAFE_TEMPERATURE 70.0f //

#define MINIMUM_SPEED_RATIO 0.1f

/********************************
 * Functions prototype
 */

void heat_control_init();
void heat_control_reset();

//for testing purposes
void heat_control_manual_mode();
void heat_control_set_temperature(float temperature);
void heat_control_automatic_mode();

//get the temperature, scale it to a ratio and send it to motor_control_task and communication_task
void heat_control_task(void *pvParameters);




#endif // HEAT_CONTROL_H
