#include "common.h"
#include "motor_control.h"
#include "heat_control.h"
#include "communication.h"

SemaphoreHandle_t ADC_mutex;

TaskHandle_t heat_control_tsk_hndl; // get the temperature
TaskHandle_t user_speed_command_tsk_hndl; // get the speed
TaskHandle_t motor_control_tsk_hndl; // send the speed
//TaskHandle_t communication_tsk_hndl; // send the data
//TaskHandle_t motor_speed_mode_tsk_hndl; // change motor mode

void setup() {

    Serial.begin(115200);
    pinMode(ADC_MOTOR_PIN, OUTPUT);
    pinMode(ADC_PEDAL_PIN, INPUT);
    pinMode(ADC_HEAT_PIN, INPUT);
    dacWrite(ADC_MOTOR_PIN, SPEED_SLEEP_SIGNAL);     //repos = 0? ou 800mv? // 62 = environ 800mV

    //------------Init Lcd-----------/
    //lcd.init();
    //lcd.init();
    //lcd.backlight();
    //lcd.setCursor(6, 0);
    //lcd.print("eKart");

    ADC_mutex = xSemaphoreCreateMutex();
    motor_control_init(1.0f);
    heat_control_init();

    xTaskCreatePinnedToCore(heat_control_task, "Heat control task", 2048, NULL, 1, &heat_control_tsk_hndl, 1);
    xTaskCreatePinnedToCore(user_speed_command_task, "User speed command task", 2048, NULL, 1, &user_speed_command_tsk_hndl, 1);
    xTaskCreatePinnedToCore(motor_control_task, "Motor control task", 2048, NULL, 2, &motor_control_tsk_hndl, 1);
    //xTaskCreate(communication_task, "Communication task", 4096, NULL, 1, communication_tsk_hndl, 0);
    return;
}

void loop(){;}