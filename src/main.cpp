#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

//define TIMERS
#define TIMER_LED 500
#define TIMER_KEEPALIVE 10000
#define TIMER_BLINK_SHORT 100
#define TIMER_BLINK_MEDIUM 500
#define TIMER_BLINK_HIGH 1000 
#define TIMER_TEMPMEASUREMENT 1000
#define TASK_PWM_DELAY 100 

//secuence Blinks
#define BLINK_1 1
#define BLINK_2 2
#define BLINK_3 3
#define BLINK_4 4

//Address definition
#define KY053_ADDRESS 0x49

// PT100 constants
#define PT100_R0 100.0        // Resistance of PT100 at 0°C
#define ALPHA 0.00385         // Temperature coefficient for PT100
#define REF_RESISTANCE_LEFT 9750.0 // Reference resistance in the divider
#define REF_RESISTANCE_RIGHT 9870.0 // Reference resistance in the divider
#define VDD_PT100 3.270 
#define GAIN_SELECTED GAIN_SIXTEEN  
#define VREF 0.256
//GAIN_ONE  4.096 
//GAIN TWO  2.048 
//GAIN FOUR  1.024
//GAIN EIGHT 0.512
//GAIN SIXTEEN 0.256 


//Heating mat definitions
#define LED_PIN 25         // Pin del LED conectado al ESP32
#define PWM_CHANNEL 0      // Canal de PWM
#define PWM_RESOLUTION 8   // Resolución del PWM (0-255)
#define PWM_FREQUENCY 5000 // Frecuencia en Hz


// Rango de temperatura y parámetros de control
#define TEMP_TARGET 28.0    // Temperatura objetivo en grados Celsius
#define TEMP_MAX 35.0       // Temperatura máxima para el ajuste PWM
#define PWM_MAX 255         // PWM máximo (100%)
#define PWM_MIN 0           // PWM mínimo (0%)
#define PWM_RAMP_UP_MAX 95  // PWM a 34°C (95%)


// Definir las constantes PID
#define KP 10.0  // Ganancia proporcional
#define KI 0.5   // Ganancia integral
#define KD 1.0   // Ganancia derivativa



//Task
void keepalive(void *pvParameters);
void readKY053(void *pvParameters);
void I2C_check(void *pvParameters);
void pwmControlTask(void *pvParameters);

//task handlers
TaskHandle_t tsk1 = NULL;
TaskHandle_t tsk2 = NULL;
TaskHandle_t tsk3 = NULL;
TaskHandle_t tsk4 = NULL;

//Semaphore handlers
SemaphoreHandle_t i2cSemaphore;
SemaphoreHandle_t serialSemaphore;

//Mutex handlers
SemaphoreHandle_t pwmMutex;

// ADS1115 instance
Adafruit_ADS1115 ads;


// Definición de la variable de temperatura
float currentTemp = 0.0; // Temperatura actual

// Variables para el control PID
float previousError = 0.0;
float integral = 0.0;
float pwmDutyCycle = 0.0;

void setup() {
  Serial.begin(115200);
  Serial.println("SETUP: first print ");

  const char *pcTaskError = "Error creating task! ";

  Wire.begin(); /* join i2c bus with SDA=D1 and SCL=D2 of NodeMCU */
  Serial.println("SETUP: I2C created");

  /*ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  vTaskDelay(pdMS_TO_TICKS(10)); 
  ledcAttachPin(LED_PIN, PWM_CHANNEL);
  vTaskDelay(pdMS_TO_TICKS(10)); 
  ledcWrite(PWM_CHANNEL, 0);*/

// Sempahore I2C
  i2cSemaphore = xSemaphoreCreateBinary();
  if (i2cSemaphore != NULL) {
    xSemaphoreGive(i2cSemaphore);  
  }
  if (i2cSemaphore == NULL) {
    Serial.println("SETUP: I2C Semaphore couldn't being created");
    while (true);  // Bloquea el programa si no se pudo crear el semáforo
  } else {
     Serial.println("SETUP: I2C Semaphore created & released");
  }
  xSemaphoreGive(i2cSemaphore);  // Release the semaphore initially

//Semaphore Serial port
   serialSemaphore = xSemaphoreCreateBinary();
  if (serialSemaphore != NULL) {
    xSemaphoreGive(serialSemaphore);  
  }
  if (serialSemaphore == NULL) {
    Serial.println("SETUP: Serial port Semaphore couldn't being created");
    while (true);  // Bloquea el programa si no se pudo crear el semáforo
  } else {
     Serial.println("SETUP: Serial portSemaphore created & released");
  }
  xSemaphoreGive(serialSemaphore);  // Release the semaphore initially

// Crear el mutex
  pwmMutex = xSemaphoreCreateMutex();


// Initialize ADS1115
  if (!ads.begin(KY053_ADDRESS)) {
    Serial.println("SETUP: ADS1115 not found. Halting...");
    while (true);
  }
  ads.setGain(GAIN_SELECTED); // 
  Serial.println("SETUP: ADS1115 configured");


// TASKs 
// TASKs 
  if (xTaskCreate(I2C_check, "I2C Check", 2048, NULL, 1, &tsk1) != pdPASS) {
    Serial.print(F(pcTaskError));
    if (tsk1 != NULL) {
        Serial.println(pcTaskGetName(tsk1));
    } else {
        Serial.println("xTaskCreate I2C_check: Task not created");
    }
    while (true);  // Bloquea el programa si falla la creación
  }

  if (xTaskCreate(readKY053, "ReadKY053", 4096, NULL, 2, &tsk2) != pdPASS) {
    Serial.print(F(pcTaskError));
    if (tsk2 != NULL) {
        Serial.println(pcTaskGetName(tsk2));
    } else {
        Serial.println("xTaskCreate readKY053: No task created");
    }
    while (true); // Bloquea el programa si falla la creación
  }

  if (xTaskCreate(keepalive, "KeepAlive", configMINIMAL_STACK_SIZE, NULL, 1, &tsk3) != pdPASS) {
    Serial.print(F(pcTaskError));
    if (tsk3 != NULL) {
        Serial.println(pcTaskGetName(tsk3));
    } else {
        Serial.println("Task not created");
    }
    while (true); // Bloquea el programa si falla la creación
  }

  if (xTaskCreate(pwmControlTask, "PWM CONTROL", 4096, NULL, 1, &tsk4) != pdPASS) {
    Serial.print(F(pcTaskError));
    if (tsk4 != NULL) {
        Serial.println(pcTaskGetName(tsk1));
    } else {
        Serial.println("xTaskCreate PWM_CONTROL: Task not created");
    }
    while (true);  // Bloquea el programa si falla la creación
  }
 
  vTaskStartScheduler();
}

void loop(){}

void I2C_check(void *pvParameters) {
  // Try to take the semaphore with a 500ms timeout
  if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(500)) && xSemaphoreTake(serialSemaphore, pdMS_TO_TICKS(500) )) {
    Serial.println("I2C_check: I2C scanning....");
    Serial.print("I2C_check: Semaphore status ");
    Serial.println(uxSemaphoreGetCount(i2cSemaphore));
    Serial.println("I2C_check: Semaphore available ");
    Wire.beginTransmission(KY053_ADDRESS);
    if (Wire.endTransmission() == 0) {
        Serial.printf("I2C_check: ADC found at: 0x%02X\n", KY053_ADDRESS);
    } else {
      Serial.println("I2C_check: ADC not found");
    }
    Serial.println("I2C_check: Scan Completed!!!!");
    xSemaphoreGive(i2cSemaphore);  // Libera el semáforo  
    
    Serial.println("I2C_check: Semaphore released I2C_CHECK");
    xSemaphoreGive(serialSemaphore); // Libera el semáforo del puerto serie

  } else {
    Serial.println("I2C_check: Semaphore not available");
  }
  vTaskDelete(tsk1);  // Se elimina a sí misma
}

void readKY053(void *parameter) {
  while (true)
  {
    if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(500)) && xSemaphoreTake(serialSemaphore, pdMS_TO_TICKS(500))) {
      Serial.println("readKY053: Semaphore available for KY053");
      //LEFT
      int16_t adcValue_0 = ads.readADC_SingleEnded(0); // Read channel A0
      float voltage_LEFT = adcValue_0 * VREF / 32768.0; // Convert ADC value to voltage (two)
      float resistance_LEFT = (voltage_LEFT * REF_RESISTANCE_LEFT) / ( VDD_PT100  - voltage_LEFT); // Calculate resistance
      float temperature_LEFT = (resistance_LEFT - PT100_R0) / (PT100_R0 * ALPHA); // Convert resistance to temperature 
      Serial.printf("readKY053: ADC LEFT: %d, Voltage: %.3f V, Resistance: %.2f Ω, Temperature: %.2f °C\n", adcValue_0, voltage_LEFT, resistance_LEFT, temperature_LEFT);
      //Right
      int16_t adcValue_1 = ads.readADC_SingleEnded(1); // Read channel A1
      float voltage_RIGHT = adcValue_1 * VREF / 32768.0; // Convert ADC value to voltage (two)
      float resistance_RIGHT = (voltage_RIGHT * REF_RESISTANCE_RIGHT) / ( VDD_PT100  - voltage_RIGHT); // Calculate resistance
      float temperature_RIGHT = (resistance_RIGHT - PT100_R0) / (PT100_R0 * ALPHA); // Convert resistance to temperature 
      Serial.printf("readKY053: ADC RIGHT: %d, Voltage: %.3f V, Resistance: %.2f Ω, Temperature: %.2f °C\n", adcValue_1, voltage_RIGHT, resistance_RIGHT, temperature_RIGHT);

      currentTemp = (temperature_LEFT + temperature_RIGHT) / 2.0;
    
      } else {
        Serial.println("readKY053: Error: No data available from KY-053 or Semaphore not available on KY053");
      }
      Serial.println("readKY053: Semaphore KY-053 released");
      xSemaphoreGive(i2cSemaphore);  // Libera el semáforo  
      xSemaphoreGive(serialSemaphore); // Libera el semáforo del puerto serie
      //Visual confirmation
      for (int i = 0; i < BLINK_3; i++) {
        digitalWrite(LED_BUILTIN, HIGH); 
        vTaskDelay(pdMS_TO_TICKS(TIMER_BLINK_SHORT)); 
        digitalWrite(LED_BUILTIN,LOW);
        vTaskDelay(pdMS_TO_TICKS(TIMER_BLINK_SHORT)); 
      }
    vTaskDelay(pdMS_TO_TICKS(TIMER_TEMPMEASUREMENT));  
  }
}

void keepalive(void *pvParameters) {
  const char *pcTaskRun = "Keepalive \n";
  const char *pcTaskError = "Keepalive ERROR \n";
  
  pinMode(LED_BUILTIN,OUTPUT);
  vTaskDelay(pdMS_TO_TICKS(100)); 

  while (true)
  {
    if (xSemaphoreTake(serialSemaphore, pdMS_TO_TICKS(500) )) {
        Serial.print(F(pcTaskRun)); // Message saved on ROM and not on
        xSemaphoreGive(serialSemaphore); // Libera el semáforo del puerto serie
    }
    digitalWrite(LED_BUILTIN, HIGH); 
    vTaskDelay(pdMS_TO_TICKS(TIMER_BLINK_MEDIUM)); 
    digitalWrite(LED_BUILTIN,LOW);
    vTaskDelay(pdMS_TO_TICKS(TIMER_BLINK_MEDIUM)); 

    vTaskDelay(pdMS_TO_TICKS(TIMER_KEEPALIVE));  
  }
  Serial.print(F(pcTaskError));
}

void pwmControlTask(void *pvParameters) {
    uint8_t dutyCycle = 0;     // Ciclo de trabajo inicial
    int8_t step = 10;           // Paso para cambiar el brillo

    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    vTaskDelay(pdMS_TO_TICKS(10)); 
    ledcAttachPin(LED_PIN, PWM_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(10)); 
   // ledcWrite(PWM_CHANNEL, 0);

  while (true) {

   /* if (xSemaphoreTake(serialSemaphore, pdMS_TO_TICKS(2000) )) {
      Serial.println("PWMcontrol: Semaphore available for PWM"); 
      xSemaphoreGive(serialSemaphore); // Libera el semáforo del puerto serie
      }
    */
    if (xSemaphoreTake(pwmMutex, portMAX_DELAY) == pdTRUE) {
     
     if (currentTemp < TEMP_MAX) {
      // Calcular el PWM con una rampa hasta 95% de DC
      pwmDutyCycle = map(currentTemp, TEMP_MAX ,TEMP_TARGET, PWM_MIN, PWM_RAMP_UP_MAX);
      float pwmPercentage = pwmDutyCycle / 255 * 100.0;

      // Control del PWM de forma gradual
      ledcWrite(PWM_CHANNEL, pwmDutyCycle);

      if (xSemaphoreTake(serialSemaphore, pdMS_TO_TICKS(2000) )) {
      Serial.printf("PWMcontrol: Temp: %.2f, PWM: %0.1f , DC : %0.1f %% \n", currentTemp, pwmDutyCycle, pwmPercentage); 
      xSemaphoreGive(serialSemaphore); // Libera el semáforo del puerto serie
      }
    } else if (currentTemp >= TEMP_MAX) {
      // Mantener el PWM en el valor máximo de 95% cuando se alcanza la temperatura máxima
      ledcWrite(PWM_CHANNEL, PWM_MIN);
    }
     /*
      // Escribir el duty cycle en el canal PWM
      ledcWrite(PWM_CHANNEL, dutyCycle);

      // Cambiar el duty cycle en cada iteración
      dutyCycle += step;

      // Invertir la dirección del cambio al alcanzar el límite
      if (dutyCycle == 0 || dutyCycle == 250) {
        step = -step;  // Cambia la dirección (subir o bajar brillo)
      }
      */
      // Liberar el mutex después de acceder al PWM
      xSemaphoreGive(pwmMutex);
    }

    // Retardo para suavizar el cambio
    vTaskDelay(pdMS_TO_TICKS(TASK_PWM_DELAY));
  }
  
}
