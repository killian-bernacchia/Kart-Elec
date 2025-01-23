#ifndef GLOBALS_H
#define GLOBALS_H

#include <Wire.h>

#include <LiquidCrystal_I2C.h>


#define MOTOR_PIN 25
#define PEDAL_PIN 34
#define TEMP_PIN 15

#define NO_SIGNAL_TEMP_VOLTAGE 0.0067f // ~-20°C

#define MAX_SPEED_COUNT 10 //number of time the speed can be unchanged before the motor is stopped
#define MAX_TEMP_COUNT 250 //number of time the temperature can be unchanged before the motor is stopped
#define NO_SIGNAL_SPEED 35 // inferior speeds are seen as 'no signals'
#define SLEEP_SPEED 62 // speed when the motor is stopped
#define SPEED_FILTER_SIZE 9

#define START_SAFE_TEMP 80
#define END_SAFE_TEMP 118
#define OVERHEAT_TEMP 120
#define TEMP_FILTER_SIZE 13

#if END_SAFE_TEMP <= START_SAFE_TEMP
#error "END_SAFE_TEMP doit être STRICTEMENT SUPERIEUR à START_SAFE_TEMP ! (on a eu chaud)"
#endif


static QueueHandle_t speedQueue;
static QueueHandle_t tempQueue;

volatile uint8_t feedBack_speed;    // writed by getTemp_Task, readed by feedBack_Task
volatile int feedBack_temperature;  // writed by getTemp_Task, readed by feedBack_Task

SemaphoreHandle_t ADC_mutex;
TaskHandle_t getTemp_TaskHandle;
TaskHandle_t getUserSpeed_TaskHandle;
TaskHandle_t commandSpeed_TaskHandle;
TaskHandle_t feedBack_TaskHandle;

LiquidCrystal_I2C lcd(0x27, 20, 4); // Adresse I2C, colonnes, lignes


#endif // GLOBALS_H