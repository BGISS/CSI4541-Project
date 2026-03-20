#ifndef communication_h
#define communication_h

// Data structures
struct AmbientTempData{
    float temperature;
    float humidity;
};
struct PositionData{
    float AcX;
    float AcY;
    float AcZ;
};
struct HeartData{
    float spO2;
    float heartBeat;
};
struct HumanTempData{
    float temperature;
};

// Public API
void Communication_init();
void updatePositionData(PositionData);
void updateHeartData(HeartData);
void updateAmbientTempData(AmbientTempData);
void updateHumanTempData(HumanTempData);
void postData(void* pvParameters);
void handleClient();

#endif
