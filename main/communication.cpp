#include "communication.h"
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include "freertos/semphr.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

//Semaphore
static SemaphoreHandle_t dataMutex = NULL;

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define SSID_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define PASSWORD_CHAR_UUID  "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"
#define STATUS_CHAR_UUID    "d4e1f4f0-8c7e-4f4c-b4d4-3f4e8c7d6a5b"

static AmbientTempData latestAmbientTemp;
static PositionData latestPosition;
static HeartData latestHeart;
static HumanTempData latestHumanTemp;
Preferences preferences;

static String wifiSSID = "";
static String wifiPassword = "";
static bool credentialsReceived = false;
static BLECharacteristic* statusCharacteristic = nullptr;

static void initBLE();
static void initWiFi();

TaskHandle_t postDataHandle = NULL;

class CredentialsCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        value.trim();  // IMPORTANT
        String uuid = String(pCharacteristic->getUUID().toString().c_str());
        Serial.print("Received UUID: ");
        Serial.println(uuid);
        Serial.print("Value: ");
        Serial.println(value);
        if (uuid == SSID_CHAR_UUID) {
            wifiSSID = String(value.c_str());
        } 
        else if (uuid == PASSWORD_CHAR_UUID) {
            wifiPassword = String(value.c_str());
            
            // Both credentials received, try to connect
            if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
                credentialsReceived = true;
            }
        }
    }
};

void Communication_init() {
    dataMutex = xSemaphoreCreateMutex(); 
    // Load saved credentials from flash
    preferences.begin("wifi", false);
    // wifiSSID = preferences.getString("ssid", "");
    // wifiPassword = preferences.getString("password", "");
    wifiSSID = "That Zazaaa";
    wifiPassword = "thegriddy69";
    preferences.end();
    
    // If we have saved credentials, try to connect
    if (wifiSSID.length() > 0) {
        initWiFi();
        
        // If connection fails, start BLE
        if (WiFi.status() != WL_CONNECTED) {
            initBLE();
        }
    } else {
        // No saved credentials, start BLE
        initBLE();
    }
}

static void initBLE() {    
    BLEDevice::init("ESP32-Setup");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    // SSID characteristic
    BLECharacteristic *pSSIDChar = pService->createCharacteristic(
        SSID_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pSSIDChar->setCallbacks(new CredentialsCallbacks());
    
    // Password characteristic
    BLECharacteristic *pPasswordChar = pService->createCharacteristic(
        PASSWORD_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pPasswordChar->setCallbacks(new CredentialsCallbacks());
    
    statusCharacteristic = pService->createCharacteristic(
        STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    statusCharacteristic->setValue("waiting");
    
    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();
}

static void initWiFi() {
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
          Serial.println(WiFi.status());
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        
        // Save credentials to flash
        preferences.begin("wifi", false);
        preferences.putString("ssid", wifiSSID);
        preferences.putString("password", wifiPassword);
        preferences.end();
        
        // Update BLE status if active
        if (statusCharacteristic != nullptr) {
            statusCharacteristic->setValue("connected");
            statusCharacteristic->notify();
        }
        
        // Stop BLE to save resources
        BLEDevice::deinit(true);
        
        if (postDataHandle == NULL) {
            xTaskCreatePinnedToCore(postData, "postData", 8192, NULL, 1, &postDataHandle, 0);
        }
    } else {
        Serial.println("\nWiFi connection failed!");
        if (statusCharacteristic != nullptr) {
            statusCharacteristic->setValue("failed");
            statusCharacteristic->notify();
        }
    }
}

void updateAmbientTempData(AmbientTempData data) { 
    if (dataMutex == NULL) return;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        latestAmbientTemp = data;
        xSemaphoreGive(dataMutex);
    } 
}
void updatePositionData(PositionData data) {
    if (dataMutex == NULL) return; 
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        latestPosition = data;
        xSemaphoreGive(dataMutex);
    } 
}
void updateHeartData(HeartData data) { 
    if (dataMutex == NULL) return;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        latestHeart = data;
        xSemaphoreGive(dataMutex);
    } 
}
void updateHumanTempData(HumanTempData data){ 
    if (dataMutex == NULL) return; 
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        latestHumanTemp = data;
        xSemaphoreGive(dataMutex);
    } 
}

static void reconnectWiFi() {
    Serial.println("WiFi lost — attempting reconnect...");
    WiFi.disconnect();
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi reconnected.");
    } else {
        Serial.println("WiFi reconnect failed.");
    }
}

void postData(void* pvParameters) {
   for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            AmbientTempData ambientCopy;
            PositionData    positionCopy;
            HeartData       heartCopy;
            HumanTempData   humanTempCopy;

            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ambientCopy   = latestAmbientTemp;
                positionCopy  = latestPosition;
                heartCopy     = latestHeart;
                humanTempCopy = latestHumanTemp;
                xSemaphoreGive(dataMutex);
                char json[256];
                snprintf(json, sizeof(json),
                    "{"
                        "\"ambientTemp\":%.2f,\"humidity\":%.2f,"
                        "\"acX\":%.2f,\"acY\":%.2f,\"acZ\":%.2f,"
                        "\"spO2\":%.2f,\"heartBeat\":%.2f,"
                        "\"humanTemp\":%.2f"
                    "}",
                    ambientCopy.temperature, ambientCopy.humidity,
                    positionCopy.AcX, positionCopy.AcY, positionCopy.AcZ,
                    heartCopy.spO2, heartCopy.heartBeat,
                    humanTempCopy.temperature
                );
                HTTPClient http;
                WiFiClientSecure client;
                client.setInsecure();
                http.begin(client,"https://healthmonitoringdashboard.onrender.com/data");
                http.addHeader("Content-Type", "application/json");
                int httpCode = http.POST(json);
                // Serial.print(httpCode);
                http.end();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void handleClient() {
    // Check if we received credentials via BLE
    if (credentialsReceived) {
        credentialsReceived = false;
        if (statusCharacteristic != nullptr) {
            statusCharacteristic->setValue("connecting");
            statusCharacteristic->notify();
        }
        initWiFi();
    }
}
