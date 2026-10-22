#ifndef USV_HOST_SEMPHR_H
#define USV_HOST_SEMPHR_H

#include "FreeRTOS.h"
#include "../HostTestSupport.h"

inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new HostMutex{}; }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t mutex, unsigned long) {
    if (HostTest::failNextMutexTake) {
        HostTest::failNextMutexTake = false;
        return pdFALSE;
    }
    return mutex && mutex->available ? pdTRUE : pdFALSE;
}
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t mutex) {
    if (!mutex) return pdFALSE;
    mutex->available = true;
    return pdTRUE;
}

#endif
