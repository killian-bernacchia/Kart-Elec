#if !defined(HEAT_CONTROL_H)
#define HEAT_CONTROL_H

#include <math.h>
#include <stdint.h>

float rawToKelvin(uint16_t VIN);
float rawToCelsius(uint16_t VIN);

#endif //HEAT_CONTROL_H