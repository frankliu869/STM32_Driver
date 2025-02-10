#include "bsp_clock.h"

uint32_t Clock_GetTimeStampMs(void)
{
    return HAL_GetTick();
}