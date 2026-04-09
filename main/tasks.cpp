#include "tasks.h"
#include "communication.h"
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <Adafruit_MLX90614.h>
#include <Arduino.h>
#include "pitches.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <DHT.h>

//Thresholds
volatile float MAX_BODY_TEMP_VAL = 38.0;
volatile float MIN_BODY_TEMP_VAL = 33.0;
volatile int   MAX_HEARTBEAT_VAL = 200;
volatile int   MIN_HEARTBEAT_VAL = 60;

//Sensor threads
TaskHandle_t DHT11Handle = NULL;
TaskHandle_t MPU6050Handle = NULL;
TaskHandle_t MAX30102Handle = NULL;
TaskHandle_t MAX30205Handle = NULL;
TaskHandle_t MLX90614Handle = NULL;
TaskHandle_t SPO2Handle = NULL;

//Buzzer
int buzzer = 26;
int buzzingTime = 100;

//MPU6050 Params
const int MPU_addr=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//DHT11 Params
#define DHT_SENSOR_TYPE DHT11
static const int DHT_SENSOR_PIN = 25;
DHT dht( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

//MAX30102 Params
MAX30105 particleSensor;
#define SPO2_BUFFER_LENGTH 100
static uint32_t irBuffer [SPO2_BUFFER_LENGTH];
static uint32_t redBuffer[SPO2_BUFFER_LENGTH];
const unsigned long COLLECTION_PERIOD_MS = 5000;
const int MAX_BEATS = 20;

struct BeatRecord {
  byte bpm;
};

BeatRecord beatHistory[MAX_BEATS];
int beatCount = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
unsigned long periodStartTime = 0;

// MLX90614 Params
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//Strucs for wifi communication
AmbientTempData currentAmbientTemp;
PositionData  currentPosition;
HeartData  currentHeart;
HumanTempData currentHuman;

//Semaphore for I2C
SemaphoreHandle_t wireMutex = NULL;
#include <Wire.h>

 // Create tasks and store handles
void CreateAllTasks() {
  wireMutex = xSemaphoreCreateMutex();
  Wire.begin(21,22);
  pinMode(buzzer,OUTPUT);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  
  Wire.write(0);     
  Wire.endTransmission(true);

  dht.begin();
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
        particleSensor.setup();
        particleSensor.setPulseAmplitudeRed(0x50);
    }
    Wire.setClock(100000);

  xTaskCreatePinnedToCore(DHT11Task, "DHT11", 4096, NULL, 2, &DHT11Handle, 1);
  xTaskCreatePinnedToCore(MPU6050Task, "MPU6050", 4096, NULL, 2, &MPU6050Handle, 1);
  xTaskCreatePinnedToCore(MLX90614Task, "MLX90614", 4096, NULL, 2, &MLX90614Handle, 1);
  xTaskCreatePinnedToCore(ThresholdSyncTask, "ThresholdSync", 16384, NULL, 1, NULL, 0);  
  xTaskCreatePinnedToCore(MAX30102Task, "SPO2", 16384, NULL, 2, &SPO2Handle, 1);
}

void DHT11Task(void *pvParameters){
  Serial.println("DHT11 Started");
  /* Measure temperature and humidity.  If the functions returns
     true, then a measurement is available. */
    for (;;) {
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (!isnan(temperature) && !isnan(humidity)) {
      currentAmbientTemp.temperature = temperature;
      currentAmbientTemp.humidity = humidity;
      updateAmbientTempData(currentAmbientTemp);
      Serial.print("Ambient Temperature: ");
      Serial.print(temperature);
      Serial.print("°C, Humidity: ");
      Serial.print(humidity);
      Serial.println("%");
    } else {
      Serial.println("DHT11: Failed to read");
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); 
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
      Serial.print("AcX: ");
      Serial.print(currentPosition.AcX);
      Serial.print("m/s^2 ");
      Serial.print("AcY: ");
      Serial.print(currentPosition.AcY);
      Serial.print("m/s^2 ");
      Serial.print("AcZ: ");
      Serial.print(currentPosition.AcZ);
      Serial.println("m/s^2");
     
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void MAX30102Task(void *pvParameters) {
  int32_t spo2 = 0;
  int8_t  validSPO2 = 0;
  int32_t heartRate = 0;
  int8_t  validHeartRate = 0;

  for (;;) {
    for (int i = 0; i < SPO2_BUFFER_LENGTH; i++) {
      while (!particleSensor.available()) {
        if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          particleSensor.check();
          xSemaphoreGive(wireMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
      }

      if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i]  = particleSensor.getIR();
        particleSensor.nextSample();
        xSemaphoreGive(wireMutex);
      }
    }

    if (irBuffer[SPO2_BUFFER_LENGTH - 1] < 50000) {
      Serial.println("SPO2: No finger detected");
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, SPO2_BUFFER_LENGTH,
      redBuffer,
      &spo2, &validSPO2,
      &heartRate, &validHeartRate
    );

    if (validSPO2 && spo2 > 0) {
      Serial.print("SPO2: ");
      Serial.print(spo2);
      Serial.println("%");
      currentHeart.spO2 = spo2;
    } else {
      Serial.println("SPO2: Invalid reading");
    }

    if (validHeartRate && heartRate > 0) {
      Serial.print("Heartrate: ");
      Serial.print(heartRate);
      Serial.println(" BPM");
      currentHeart.heartBeat = heartRate;
    }
    if(heartRate < MIN_HEARTBEAT_VAL || heartRate > MAX_HEARTBEAT_VAL){
      startBuzzer();
    }    
    updateHeartData(currentHeart);
    vTaskDelay(pdMS_TO_TICKS(5000));
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
      
      if (obj < MIN_BODY_TEMP_VAL || obj > MAX_BODY_TEMP_VAL) {
        startBuzzer();
      }
      
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
 
void startBuzzer(){
  tone(26, NOTE_C5, 3000);
  delay(2000);
}

void ThresholdSyncTask(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(httpMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
    HTTPClient http;
    WiFiClientSecure client;  
    client.setInsecure();
    http.begin(client, "https://healthmonitoringdashboard.onrender.com/thresholds");
    int code = http.GET();
    if (code == 200) {
      String body = http.getString();
      StaticJsonDocument<512> doc;
      if (deserializeJson(doc, body) == DeserializationError::Ok) {
        if (doc["bodyTemp"]["max"].is<float>())
          MAX_BODY_TEMP_VAL = doc["bodyTemp"]["max"].as<float>();
        if (doc["bodyTemp"]["min"].is<float>())
          MIN_BODY_TEMP_VAL = doc["bodyTemp"]["min"].as<float>();
        if (doc["heartRate"]["max"].is<int>())
          MAX_HEARTBEAT_VAL = doc["heartRate"]["max"].as<int>();
        if (doc["heartRate"]["min"].is<int>())
          MIN_HEARTBEAT_VAL = doc["heartRate"]["min"].as<int>();
      }
    }
    http.end();
    xSemaphoreGive(httpMutex);
  }
    vTaskDelay(pdMS_TO_TICKS(100000)); 
  }
}