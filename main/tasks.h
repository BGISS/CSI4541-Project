#ifndef tasks_h
#define tasks_h

void DHT11Task(void *pvParameters);
void MPU6050Task(void *pvParameters);
void CreateAllTasks();
static bool measure_environment( float *temperature, float *humidity );

#endif