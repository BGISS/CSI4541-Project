#ifndef communication_h
#define communication_h

// Data structure
struct SensorData {
    float temperature;
    float humidity;
    unsigned long timestamp;
};

// Public API
void Communication_init();
void updateSensorData(SensorData data);
void handleClient();

#endif
