#ifndef USV_HOST_FREERTOS_H
#define USV_HOST_FREERTOS_H

#include <cstdint>
using BaseType_t = int;
struct HostMutex { bool available = true; };
using SemaphoreHandle_t = HostMutex*;
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(value) (value)
#define portMAX_DELAY 0xffffffffUL

#endif
