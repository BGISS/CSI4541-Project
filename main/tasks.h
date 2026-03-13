#ifndef tasks_h
#define tasks_h

void DHT11Task(void *pvParameters);
void MPU6050Task(void *pvParameters);
void MAX30102Task(void *pvParameters);
void CreateAllTasks();

#endif