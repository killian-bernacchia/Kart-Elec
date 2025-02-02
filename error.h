#if !defined(ERROR_H)
#define ERROR_H

#include "common.h"

typedef enum ErrorLocation{
    HEAT_CONTROL,
    COMMUNICATION,
    MOTOR_CONTROL,
    USER_SPEED_COMMAND,
    MOTOR_SPEED_MODE
} ErrorLocation;

void ErrorNoSignal(ErrorLocation location);


#endif // ERROR_H


