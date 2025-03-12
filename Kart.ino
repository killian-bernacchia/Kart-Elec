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

//LiquidCrystal_I2C lcd(0x27, 20, 4); // Adresse I2C, colonnes, lignes

void setup() {

    Serial.begin(115200);
    pinMode(ADC_MOTOR_PIN, OUTPUT);
    pinMode(ADC_PEDAL_PIN, INPUT);
    pinMode(ADC_HEAT_PIN, INPUT);
    //dacWrite(ADC_MOTOR_PIN, SPEED_SLEEP_SIGNAL);     //repos = 0? ou 800mv? // 62 = environ 800mV

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
    //xTaskCreatePinnedToCore(motor_control_task, "Motor control task", 2048, NULL, 2, &motor_control_tsk_hndl, 1);
    //xTaskCreate(communication_task, "Communication task", 4096, NULL, 1, communication_tsk_hndl, 0);
    return;
}

void loop(){
  static int i = 0;
  Speed_Data speed_data;
  //xQueueReceive(user_speed_cmd_2_motor_ctrl_queue, &speed_data, portMAX_DELAY);
  Serial.printf("raw %i\n",speed_data.adc.raw);
  Serial.printf("vlt %f\n",speed_data.adc.voltage);
  Serial.printf("fil %f\n",speed_data.adc.filtered);
  Serial.printf("nsc %i\n",speed_data.adc.no_signal_count);
  Serial.printf("spd %i\n",speed_data.speed);
  Serial.printf("rat %f\n",speed_data.ratio);
  int ratio = speed_data.ratio*100;
  //lcd.setCursor(2, 0);
  //lcd.print("SPEED : ");
  //lcd.print(ratio);
  //lcd.print("%");
  //lcd.setCursor(2, 1);
  //lcd.print("loop n°");
  //lcd.print(i++);
  delay(1000);
}