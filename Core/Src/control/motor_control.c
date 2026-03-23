#include "control/motor_control.h"

uint8_t enable_motor = 0;
float32_t PWM_MOTOR[4];
uint32_t PWM_TIMER[4];

void MIX_THROTTLE(float32_t thr, float32_t* moment, float32_t* m) {
    m[0] = thr - moment[0] + moment[1] + moment[2]; // Motor 1
    m[1] = thr - moment[0] - moment[1] - moment[2]; // Motor 2
    m[2] = thr + moment[0] - moment[1] + moment[2]; // Motor 3
    m[3] = thr + moment[0] + moment[1] - moment[2]; // Motor 4

    for (int i = 0; i < 4; i++) {
        if (m[i] > 1850) m[i] = 1850;
        if (m[i] < MIN_ARM) m[i] = MIN_ARM;
    }
}

void Control_Motor(void) {
    if (enable_motor) {
        for (int i = 0; i < 4; i++) PWM_TIMER[i] = (uint32_t)PWM_MOTOR[i];
        TIM3->CCR1 = PWM_TIMER[0]; TIM3->CCR2 = PWM_TIMER[1];
        TIM4->CCR1 = PWM_TIMER[2]; TIM4->CCR2 = PWM_TIMER[3];
    } else {
        TIM3->CCR1 = 1000; TIM3->CCR2 = 1000; TIM4->CCR1 = 1000; TIM4->CCR2 = 1000;
    }
}
