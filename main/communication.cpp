#include "communication.h"
#include <WiFi.h>
#include <WebServer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define SSID_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define PASSWORD_CHAR_UUID  "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"
#define STATUS_CHAR_UUID    "d4e1f4f0-8c7e-4f4c-b4d4-3f4e8c7d6a5b"

// Server + shared data
WebServer server(80);
static SensorData latestData;
Preferences preferences;

static String wifiSSID = "";
static String wifiPassword = "";
static bool credentialsReceived = false;
static BLECharacteristic* statusCharacteristic = nullptr;

static void initBLE();
static void initWiFi();
static void handleDataRequest();
static void handleRoot();

class CredentialsCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        String uuid = pCharacteristic->getUUID().toString().c_str();
        
        if (uuid == SSID_CHAR_UUID) {
            wifiSSID = String(value.c_str());
            Serial.println("SSID received: " + wifiSSID);
        } 
        else if (uuid == PASSWORD_CHAR_UUID) {
            wifiPassword = String(value.c_str());
            Serial.println("Password received");
            
            // Both credentials received, try to connect
            if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
                credentialsReceived = true;
            }
        }
    }
};

void Communication_init() {
    // Load saved credentials from flash
    preferences.begin("wifi", false);
    wifiSSID = preferences.getString("ssid", "");
    wifiPassword = preferences.getString("password", "");
    preferences.end();
    
    // If we have saved credentials, try to connect
    if (wifiSSID.length() > 0) {
        Serial.println("Trying saved credentials...");
        initWiFi();
        
        // If connection fails, start BLE
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Saved credentials failed, starting BLE...");
            initBLE();
        }
    } else {
        // No saved credentials, start BLE
        Serial.println("No saved credentials, starting BLE...");
        initBLE();
    }
}

static void initBLE() {
    Serial.println("Starting BLE provisioning...");
    
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
    
    // Status characteristic (optional - for feedback to phone)
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
    
    Serial.println("BLE advertising started. Connect with phone app.");
}

static void initWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
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
        BLEDevice::deinit(false);
        
        // Start web server
        server.on("/", handleRoot);
        server.on("/data", handleDataRequest);
        server.begin();
        Serial.println("HTTP server started");
    } else {
        Serial.println("\nWiFi connection failed!");
        if (statusCharacteristic != nullptr) {
            statusCharacteristic->setValue("failed");
            statusCharacteristic->notify();
        }
    }
}

void updateSensorData(SensorData data) {
    latestData = data;
}

static void handleDataRequest() {
    String json = "{";
    json += "\"temperature\":" + String(latestData.temperature, 2) + ",";
    json += "\"humidity\":" + String(latestData.humidity, 2);
    json += "}";

    server.send(200, "application/json", json);
    Serial.println("HTTP -> " + json);
}

static void handleRoot() {
    String html = "<!DOCTYPE html><html><body style='font-family: Arial;'>";
    html += "<h1>ESP32 Sensor Data</h1>";
    html += "<p>Temperature: " + String(latestData.temperature, 1) + " °C</p>";
    html += "<p>Humidity: " + String(latestData.humidity, 1) + " %</p>";
    html += "<p><a href='/data'>Get JSON</a></p>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}

void handleClient() {
    // Check if we received credentials via BLE
    if (credentialsReceived) {
        credentialsReceived = false;
        Serial.println("Attempting WiFi connection...");
        if (statusCharacteristic != nullptr) {
            statusCharacteristic->setValue("connecting");
            statusCharacteristic->notify();
        }
        initWiFi();
    }
    
    server.handleClient();
}
