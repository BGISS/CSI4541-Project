#ifndef tasks_h
#define tasks_h

void DHT11Task(void *pvParameters);
void MPU6050Task(void *pvParameters);
void MAX30102Task(void *pvParameters);
void MAX30205Task(void *pvParameters);
void findMAX30205();
void startBuzzer();
void CreateAllTasks();

#endif