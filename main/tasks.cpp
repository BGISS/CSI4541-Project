#include "tasks.h"
#include "communication.h"
#include <DHT11.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <Adafruit_MLX90614.h>

//Sensor threads
TaskHandle_t DHT11Handle = NULL;
TaskHandle_t MPU6050Handle = NULL;
TaskHandle_t MAX30102Handle = NULL;
TaskHandle_t MAX30205Handle = NULL;
TaskHandle_t MLX90614Handle = NULL;

//MPU6050 Params
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//DHT11 Params
#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = 18;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

//MAX30102 Params
MAX30105 particleSensor;
#define SPO2_BUFFER_LENGTH 100
uint32_t irBuffer[SPO2_BUFFER_LENGTH];
uint32_t redBuffer[SPO2_BUFFER_LENGTH];
int32_t  spo2Value;
int8_t   spo2Valid;
int32_t  heartRateValue;
int8_t   hrValid;

// MLX90614 Params
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//Strucs for wifi communication
AmbientTempData currentAmbientTemp;
PositionData    currentPosition;
HeartData       currentHeart;
HumanTempData currentHuman;

void CreateAllTasks() {
  // Create tasks and store handles
  xTaskCreatePinnedToCore(DHT11Task, "DHT11", 4096, NULL, 1, &DHT11Handle, 1);
  Wire.begin(21,22);
  if (!mlx.begin(0x5A, &Wire)) {
    Serial.println("MLX90614 not found. Check wiring.");
  } else {
    Serial.println("MLX90614 ready.");
  }
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  xTaskCreatePinnedToCore(MPU6050Task, "MPU6050", 4096, NULL, 1, &MPU6050Handle, 1);
  xTaskCreatePinnedToCore(MLX90614Task, "MLX90614", 4096, NULL, 1, &MLX90614Handle, 1);

  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) {
      Serial.println("MAX30102 not found. Check wiring.");
    } else {
        particleSensor.setup(60, 4, 2, 100, 411, 4096);
        //Changing the amplitude helps to adjust accuracy of the sensor
        particleSensor.setPulseAmplitudeRed(0x1F);
        particleSensor.setPulseAmplitudeIR(0x1F);
    }
  xTaskCreatePinnedToCore(MAX30102Task, "MAX30102", 16384, NULL, 1, &MAX30102Handle, 1);
}

void DHT11Task(void *pvParameters){
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
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}


void MPU6050Task(void *pvParameters){
  
  for(;;){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    // Serial.print("AcX = "); Serial.print(AcX);
    // Serial.print(" | AcY = "); Serial.print(AcY);
    // Serial.print(" | AcZ = "); Serial.print(AcZ);
    // Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //From the datasheet of MPU6050, we can know the temperature formula
    // Serial.print(" | GyX = "); Serial.print(GyX);
    // Serial.print(" | GyY = "); Serial.print(GyY);
    // Serial.print(" | GyZ = "); Serial.println(GyZ);
    currentPosition.AcX = AcX;
    currentPosition.AcY = AcY;
    currentPosition.AcZ = AcZ;
    updatePositionData(currentPosition);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void MAX30102Task(void *pvParameters){
    const byte RATE_SIZE = 8;
    byte rates[RATE_SIZE];
    byte rateSpot = 0;
    long lastBeat = 0;
    float beatsPerMinute;
    int beatAvg = 0;

    for (;;) {
        particleSensor.check();
        
        if (!particleSensor.available()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        long irValue  = particleSensor.getIR();
        long redValue = particleSensor.getRed();
        particleSensor.nextSample();

        if (irValue < 50000) {
            Serial.println("No finger detected");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Heart rate using beat detection
        if (checkForBeat(irValue)) {
            long delta = millis() - lastBeat;
            lastBeat = millis();
            beatsPerMinute = 60.0 / (delta / 1000.0);

            if (beatsPerMinute > 20 && beatsPerMinute < 200) {
                rates[rateSpot++] = (byte)beatsPerMinute;
                rateSpot %= RATE_SIZE;

                beatAvg = 0;
                for (byte x = 0; x < RATE_SIZE; x++)
                    beatAvg += rates[x];
                beatAvg /= RATE_SIZE;
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
        Serial.print("HR: ");   
        Serial.print(beatAvg);
        Serial.print(" BPM, SpO2: "); 
        Serial.print(spo2);
        Serial.println("%");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

 
void MLX90614Task(void *pvParameters) {
    for (;;) {
        // float tempC = -999.0f;
        // tempC = mlx.readObjectTempC();        
        // if (!isnan(tempC) && tempC > 0.0f && tempC < 100.0f) {
        //     currentHuman.temperature = tempC;
        //     updateHumanTempData(currentHuman);
        //     Serial.print("IR Body Temp: ");
        //     Serial.print(tempC, 2);
        //     Serial.println(" °C");
        // } else {
        //     Serial.println("MLX90614 read error");
        //     Serial.println(tempC); 
        // }
        float obj  = mlx.readObjectTempC();
        float amb  = mlx.readAmbientTempC();
        Serial.print("obj: "); Serial.print(obj);
        Serial.print("  amb: "); Serial.println(amb);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}