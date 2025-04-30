#include "heat_control.h"

float rawToKelvin(uint16_t voltage) {
    float voltage = voltage * 3.3 / 4095.0;
    float R2 = 4700.0 * ((3.3 / voltage) - 1);
    float kelvin = 1 / ((1 / 298.15) + (1 / 3950.0) * log(R2 / 10000.0));
    return kelvin;
}

float rawToCelsius(uint16_t voltage) {
    return rawToKelvin(voltage) - 273.15;
}
