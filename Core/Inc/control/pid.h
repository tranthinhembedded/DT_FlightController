/*
 * pid.h
 *
 * Created on: Dec 14, 2025
 * Author: THINH
 * Updated: iNAV Style Support (D-on-Measurement & Dynamic FF)
 */

#ifndef CORE_INC_CONTROL_PID_H_
#define CORE_INC_CONTROL_PID_H_

#include "arm_math.h"

typedef struct {
    // --- Tunings ---
    float32_t kp;
    float32_t ki;
    float32_t kd;
    float32_t feed_forward;   // iNav: Dynamic FF Gain (Rate Loop)

    // --- State Variables ---
    float32_t error;
    float32_t prev_error;     // Dung cho Angle Loop (hoac legacy D-term)

    float32_t integral;
    float32_t derivative;

    // --- NEW: iNAV / Betaflight Style State Variables ---
    float32_t prev_measure;   // Luu gia tri do cu (Dung cho D-on-Measurement)
    float32_t prev_setpoint;  // Luu setpoint cu (Dung cho Dynamic Feed Forward)

    // --- Configs ---
    float32_t alpha_lpf;      // derivative low-pass filter factor (0-1)
    float32_t max_output;     // gioi han PID output (torque / rate)
    float32_t i_limit;        // anti-windup
    float32_t d_limit;        // giu de tuong thich
    float32_t output;

} PID_ALTIDUE_t;

void Reset_PID_ALTIDUE(PID_ALTIDUE_t *pid);

// Angle Loop (Outer Loop) - Van dung thuat toan co ban
void Caculate_PID_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback , float32_t dt);

// Rate Loop (Inner Loop) - Dung thuat toan iNav (D-on-Measurement & Dynamic FF)
void Caculate_PID_Rate_ALTIDUE(PID_ALTIDUE_t *pid , float32_t setpoint , float32_t feedback, float32_t dt);

#endif /* CORE_INC_CONTROL_PID_H_ */
