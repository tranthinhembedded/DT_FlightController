#ifndef CORE_INC_PLATFORM_DELAY_H_
#define CORE_INC_PLATFORM_DELAY_H_

#include "main.h"

void Delay_Init(void);
void Delay_us(uint32_t us);
void Delay_ms_blocking(uint32_t ms);

#endif /* CORE_INC_PLATFORM_DELAY_H_ */