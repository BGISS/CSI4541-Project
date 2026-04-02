#include "tasks.h"
#include "communication.h"
#include <DHT11.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <Adafruit_MLX90614.h>
#include <Arduino.h>
#include "pitches.h"

//Thresholds
#define MAX_BODY_TEMP 38
#define MIN_BODY_TEMP 33
#define MAX_HEARTBEAT 200
#define MIN_HEARTBEAT 60

//Sensor threads
TaskHandle_t DHT11Handle = NULL;
TaskHandle_t MPU6050Handle = NULL;
TaskHandle_t MAX30102Handle = NULL;
TaskHandle_t MAX30205Handle = NULL;
TaskHandle_t MLX90614Handle = NULL;

//Buzzer
int buzzer = 26;
int buzzingTime = 100;

//MPU6050 Params
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//DHT11 Params
#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = 25;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

//MAX30102 Params
MAX30105 particleSensor;
#define SPO2_BUFFER_LENGTH 100
static uint32_t irBuffer [SPO2_BUFFER_LENGTH];
static uint32_t redBuffer[SPO2_BUFFER_LENGTH];

// MLX90614 Params
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//Strucs for wifi communication
AmbientTempData currentAmbientTemp;
PositionData    currentPosition;
HeartData       currentHeart;
HumanTempData currentHuman;

//Semaphore for I2C
SemaphoreHandle_t wireMutex = NULL;

void CreateAllTasks() {

  // ledcAttachPin(buzzer, 0);
  // ledcSetup(0, 3000, 8); // 3kHz

  wireMutex = xSemaphoreCreateMutex();
  // Create tasks and store handles
  Wire.begin(21,22);
  pinMode(buzzer,OUTPUT);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  
  Wire.write(0);     
  Wire.endTransmission(true);

  // Initialize MLX90614
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX90614 sensor. Check wiring.");
  } else {
    Serial.print("MLX90614 Emissivity = "); 
    Serial.println(mlx.readEmissivity());
    Serial.println("MLX90614 initialized successfully");
  }

  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) {
      Serial.println("MAX30102 not found. Check wiring.");
    } else {
        particleSensor.setup(100, 4, 2, 100, 411, 4096);
    }
  Wire.setClock(100000);

  xTaskCreatePinnedToCore(DHT11Task, "DHT11", 4096, NULL, 2, &DHT11Handle, 1);
  xTaskCreatePinnedToCore(MPU6050Task, "MPU6050", 4096, NULL, 2, &MPU6050Handle, 1);
  xTaskCreatePinnedToCore(MAX30102Task, "MAX30102", 16384, NULL, 2, &MAX30102Handle, 1);
  xTaskCreatePinnedToCore(MLX90614Task, "MLX90614", 4096, NULL, 2, &MLX90614Handle, 1);  

}

void DHT11Task(void *pvParameters){
  Serial.println("DHT11 Started");
  float temperature;
  float humidity;
  /* Measure temperature and humidity.  If the functions returns
     true, then a measurement is available. */
  for(;;){
    if( dht_sensor.measure( &temperature, &humidity ) == true )
    {
      currentAmbientTemp.temperature = temperature;
      currentAmbientTemp.humidity = humidity;
      updateAmbientTempData(currentAmbientTemp);
      Serial.print("DHT11: Got reading - Temp: ");
      Serial.print(temperature);
      Serial.print("°C, Humidity: ");
      Serial.print(humidity);
      Serial.println("%");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}


void MPU6050Task(void *pvParameters){
  
  for(;;){
    if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);  
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr,14,true); 
      AcX=Wire.read()<<8|Wire.read();   
      AcY=Wire.read()<<8|Wire.read(); 
      AcZ=Wire.read()<<8|Wire.read();  
      for (int i = 0; i < 8; i++) Wire.read();

      xSemaphoreGive(wireMutex);
      currentPosition.AcX = (AcX / 16384.0) * 9.81;
      currentPosition.AcY = (AcY / 16384.0) * 9.81;
      currentPosition.AcZ = (AcZ / 16384.0) * 9.81;
      updatePositionData(currentPosition);
     
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void MAX30102Task(void *pvParameters){
    const byte RATE_SIZE = 8;
    byte rates[RATE_SIZE] = {0};
    byte rateSpot = 0;
    long lastBeat = 0;
    bool  bufferFull = false; 
    float beatsPerMinute = 0.0f;
    int beatAvg = 0;
    uint16_t bufferIndex = 0;
    bool     bufferReady = false; 

    for (;;) {
      bool sampleReady = false;
      long irValue  = 0;
      long redValue = 0;
    if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(10)) == pdTRUE) {

      particleSensor.check();
        
      if (particleSensor.available()) {
        irValue    = particleSensor.getIR();
        redValue   = particleSensor.getRed();
        particleSensor.nextSample();
        sampleReady = true;
      }
      xSemaphoreGive(wireMutex);
    }
    if (!sampleReady) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    if (irValue < 50000) {
        Serial.println("No finger detected");
         bufferIndex  = 0;
         bufferReady = false;
         bufferFull = false;
         rateSpot = 0;
         beatAvg = 0;
         beatsPerMinute = 0.0f;
         memset(rates, 0, sizeof(rates));
         currentHeart.heartBeat = 0;
         currentHeart.spO2 = 0;
         updateHeartData(currentHeart);
         vTaskDelay(pdMS_TO_TICKS(500));
         continue;
    }

        // Heart rate using beat detection
        if (checkForBeat(irValue)) {
            long delta = millis() - lastBeat;
            lastBeat = millis();
            if (delta > 0) {
                beatsPerMinute = 60.0f / (delta / 1000.0f);
            }

            if (beatsPerMinute > 20 && beatsPerMinute < 255) {
                rates[rateSpot++] = (byte)beatsPerMinute;
                rateSpot %= RATE_SIZE;
                if (rateSpot == 0) bufferFull = true;

               if (bufferFull) {
                    beatAvg = 0;
                    for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
                    beatAvg /= RATE_SIZE;
                } else {
                    // Buffer still filling — average only the filled slots
                    int sum = 0;
                    for (byte x = 0; x < rateSpot; x++) sum += rates[x];
                    beatAvg = (rateSpot > 0) ? sum / rateSpot : 0;
                }
            }
        }

        // SpO2 using rolling average of R ratio
        static float redAC = 0, irAC = 0;
        static float redDC = 0, irDC = 0;
        redDC = 0.95 * redDC + 0.05 * redValue;
        irDC  = 0.95 * irDC  + 0.05 * irValue;
        redAC = redValue - redDC;
        irAC  = irValue  - irDC;

        static float R = 0;
        if (irAC != 0){
          R = 0.99 * R + 0.01 * ((redAC / redDC) / (irAC / irDC));
        }
        int spo2 = 104 - 17 * R;
        spo2 = constrain(spo2, 80, 100);
        currentHeart.heartBeat = beatAvg;
        currentHeart.spO2 = spo2;
        updateHeartData(currentHeart);
        if(beatAvg > 0){
          if(beatAvg < MIN_HEARTBEAT || beatAvg > MAX_HEARTBEAT){
            // startBuzzer();
          }
        }
        Serial.print("HR: ");   
        Serial.print(beatAvg);
        Serial.print(" BPM, SpO2: "); 
        Serial.print(spo2);
        Serial.println("%");
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void MLX90614Task(void *pvParameters) {
    for (;;) {
      float obj;
      float amb;
      
      if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        obj = mlx.readObjectTempC();
        amb = mlx.readAmbientTempC();
        xSemaphoreGive(wireMutex);
      }
      
      Serial.print("MLX90614 - Ambient: "); 
      Serial.print(amb);
      Serial.print("°C\tObject: "); 
      Serial.print(obj); 
      Serial.println("°C");
      
      currentHuman.temperature = obj;
      updateHumanTempData(currentHuman);
      
      if (obj < MIN_BODY_TEMP || obj > MAX_BODY_TEMP) {
        startBuzzer();
      }
      
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
 
void startBuzzer(){
  tone(18, NOTE_C5, 3000);
     
  // restart after two seconds 
  delay(2000);
}