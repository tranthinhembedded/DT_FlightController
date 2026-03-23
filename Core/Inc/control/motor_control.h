#ifndef CORE_INC_CONTROL_MOTOR_CONTROL_H_
#define CORE_INC_CONTROL_MOTOR_CONTROL_H_

#include "main.h"
#include "platform/tim.h"
#include "arm_math.h"

#define MIN_ARM 1240

extern uint8_t enable_motor;
extern float32_t PWM_MOTOR[4];
extern uint32_t PWM_TIMER[4];

void MIX_THROTTLE(float32_t thr, float32_t* moment, float32_t* m);
void Control_Motor(void);

#endif /* CORE_INC_CONTROL_MOTOR_CONTROL_H_ */
