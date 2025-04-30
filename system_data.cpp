#include "system_data.h"

void SystemDataInitSemaphore(volatile Control_Unit *unit);
void SystemDataDeleteSemaphore(volatile Control_Unit *unit);

void SystemDataInitSemaphores(void) {
  system_data.adc_mutex = xSemaphoreCreateMutex();
  SystemDataInitSemaphore(&system_data.speed_command_input);
  SystemDataInitSemaphore(&system_data.heat_input);
  SystemDataInitSemaphore(&system_data.speed_command_output);
}

void SystemDataDeleteSemaphores(void) {
  SystemDataDeleteSemaphore(&system_data.speed_command_input);
  SystemDataDeleteSemaphore(&system_data.heat_input);
  SystemDataDeleteSemaphore(&system_data.speed_command_output);
}

void SystemDataInitSemaphore(volatile Control_Unit *unit) {
  unit->mutex = xSemaphoreCreateMutex(); 
  unit->sync = xSemaphoreCreateBinary();
}
void SystemDataDeleteSemaphore(volatile Control_Unit *unit) {
  vSemaphoreDelete(unit->mutex);  
  vSemaphoreDelete(unit->sync);  
}
