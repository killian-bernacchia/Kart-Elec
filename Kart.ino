#include "globals.h"

void setup() {

  speedQueue = xQueueCreate(1, sizeof(uint8_t));
  tempQueue = xQueueCreate(1, sizeof(int));
  ADC_mutex = xSemaphoreCreateMutex();

  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(PEDAL_PIN, INPUT);
  pinMode(TEMP_PIN, INPUT);
  dacWrite(MOTOR_PIN, SLEEP_SPEED);     //repos = 0? ou 800mv? // 62 = environ 800mV

  //------------Init Lcd-----------/
  lcd.init();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(6, 0);
  lcd.print("eKart");

  //------------creation process-----------/
  //xTaskCreatePinnedToCore(taskFunction, taskName, stackSize, taskParameters, taskPriority, taskHandle, coreID)
  xTaskCreatePinnedToCore(getUserSpeed_Task, "getUserSpeed", 2048, NULL, 2, &getUserSpeed_TaskHandle, 1);
  xTaskCreatePinnedToCore(getTemp_Task, "getTemp", 2048, NULL, 2, &getTemp_TaskHandle, 1);
  xTaskCreatePinnedToCore(commandSpeed_Task, "commandSpeed", 2048, NULL, 1, &commandSpeed_TaskHandle, 1);
  xTaskCreatePinnedToCore(feedBack_Task, "feedBack", 4096, NULL, 2, &feedBack_TaskHandle, 0);  
}

uint16_t filteredSpeed(uint16_t VIN_brut) {
    static uint16_t sum = 0;
    static uint16_t samples[SPEED_FILTER_SIZE];
    static bool initialised = false;
    static uint16_t index = 0;
    
    if(!initialised) {
      for (int i = 0; i < SPEED_FILTER_SIZE; i++) {
        samples[i] = SLEEP_SPEED;
      }
      sum = SLEEP_SPEED*SPEED_FILTER_SIZE; // Initialisation de la somme
      initialised = true;
    }

    sum -= samples[index];
    samples[index] = VIN_brut;
    index = (index + 1)%SPEED_FILTER_SIZE;
    sum += VIN_brut;
    
    return sum/SPEED_FILTER_SIZE;
}

uint16_t filteredTemp(uint16_t VIN_brut){
    static uint16_t sum = 0;
    static uint16_t samples[TEMP_FILTER_SIZE];
    static bool initialised = false;
    static uint16_t index = 0;
    
    if(!initialised) {
      for (int i = 0; i < TEMP_FILTER_SIZE; i++) {
        samples[i] = SLEEP_SPEED;
      }
      sum = SLEEP_SPEED*TEMP_FILTER_SIZE; // Initialisation de la somme
      initialised = true;
    }

    sum -= samples[index];
    samples[index] = VIN_brut;
    index = (index + 1)%TEMP_FILTER_SIZE;
    sum += VIN_brut;
    
    return sum/TEMP_FILTER_SIZE;
}

void getUserSpeed_Task(void *pvParameters) {
  int VIN;
  uint8_t speed;
  for (;;) {
    xSemaphoreTake(ADC_mutex, portMAX_DELAY);
    VIN = analogRead(PEDAL_PIN);
    xSemaphoreGive(ADC_mutex);
    VIN = filteredSpeed(VIN);

    //overflow safety
    if( ((7*VIN)/5)>>4 <= 255) {
      speed = ((7*VIN)/5)>>4; //1.4*(VIN>>4); facteur correcteur + 12bits -> bits
    }
    else {
      speed = 255;
    }

    //underflow safety
    if(speed > NO_SIGNAL_SPEED && speed < SLEEP_SPEED) {
      speed = SLEEP_SPEED;
    }
    
    //no signal safety
    if(speed > NO_SIGNAL_SPEED) {
      feedBack_speed = speed;
      xQueueOverwrite(speedQueue, &speed);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void getTemp_Task(void *pvParameters) {
  uint16_t VIN;
  float voltage;
  float R2;
  float temperatureK;
  int temperatureC;

  for (;;) {
    xSemaphoreTake(ADC_mutex, portMAX_DELAY);
    VIN = analogRead(TEMP_PIN);
    xSemaphoreGive(ADC_mutex);
    voltage = filteredTemp(VIN);
    voltage = voltage * 3.3 / 4095.0;

    //no signal and division by zero safety
    if(voltage > NO_SIGNAL_TEMP_VOLTAGE)
    {
      R2 = 4700.0 * ((3.3 / voltage) - 1);
      temperatureK = 1 / ((1 / 298.15) + (1 / 3950.0) * log(R2 / 10000.0));
      temperatureC = temperatureK - 273.15;
      feedBack_temperature = temperatureC;
      xQueueOverwrite(tempQueue, &temperatureC);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void commandSpeed_Task(void *pvParameters) {
  uint8_t speed;
  uint8_t last_speed;
  int speed_count = 0;

  int temperature;
  int last_temperature;
  int temp_count = 0;

  //wait for the first speed and temperature value
  while(xQueuePeek(speedQueue, &speed, portMAX_DELAY) == pdFALSE) {
    vTaskDelay(1);
  }
  while(xQueuePeek(tempQueue, &temperature, portMAX_DELAY) == pdFALSE) {
    vTaskDelay(1);
  }

  for (;;) {
    //wait a few ticks for new speed, else use last one
    if(xQueueReceive(speedQueue, &speed, 0) == pdTRUE) {
      last_speed = speed;
      speed_count = 0;
    }
    else {
      speed = last_speed;
      speed_count++;
    }

    //wait a few ticks for new temperature, else use last one
    if(xQueueReceive(tempQueue, &temperature, 0) == pdTRUE) {
      last_temperature = temperature;
      temp_count = 0;
    }
    else {
      temperature = last_temperature;
      temp_count++;
    }
    
    //if speed or temperature too old, stop the motor and temporary block the task
    if(speed_count > MAX_SPEED_COUNT) {
      speed = SLEEP_SPEED;
      dacWrite(MOTOR_PIN, speed);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      continue;
    } 
    else if(temp_count > MAX_TEMP_COUNT) {
      speed = SLEEP_SPEED;
      dacWrite(MOTOR_PIN, speed);
      vTaskDelay(2500 / portTICK_PERIOD_MS);
      continue;
    }
    
    //temperature safety
    if(temperature >= START_SAFE_TEMP) {
      if(temperature < END_SAFE_TEMP) {
        speed = (speed - SLEEP_SPEED)*(END_SAFE_TEMP-temperature)/(END_SAFE_TEMP-START_SAFE_TEMP) + SLEEP_SPEED;
      }
      else if(temperature < OVERHEAT_TEMP) {
        speed = SLEEP_SPEED;
      }
      else {
        speed = 0; //block the motor until reboot
      }
    }
      
    //send speed to the motor
    dacWrite(MOTOR_PIN, speed);

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void feedBack_Task(void *pvParameters) {
  for (;;) {
    Serial.print("Temperature = ");
    Serial.print(feedBack_temperature);
    Serial.println(" °C");
    Serial.print("Vitesse : ");
    Serial.println(feedBack_speed);
    // Synchronisation de l'accès au LCD
    if (feedBack_speed < 85){
      lcd.setCursor(2, 0);
      lcd.print("TEMP : ");
      lcd.print(feedBack_temperature);
      lcd.print(" 'C");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}