#ifndef GLOBALS_H
#define GLOBALS_H
#pragma once
#include "freertos/semphr.h"

extern SemaphoreHandle_t wifiPauseMutex;

//Thresholds
extern volatile float MAX_BODY_TEMP_VAL;
extern volatile float MIN_BODY_TEMP_VAL;
extern volatile int   MAX_HEARTBEAT_VAL;
extern volatile int   MIN_HEARTBEAT_VAL;

#endif