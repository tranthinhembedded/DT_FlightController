#ifndef CORE_INC_CONTROL_FLIGHT_CONTROL_H_
#define CORE_INC_CONTROL_FLIGHT_CONTROL_H_

#include "main.h"
#include "control/pid.h"
#include "sensor/complementary_filter.h"
#include "sensor/imu_config.h"
#include "control/motor_control.h"
#include "input/rc_input.h"

typedef enum {
    HOVER,      // Chế độ Angle (Tự cân bằng)
    RATE_MODE,  // Chế độ Rate (Bay nhào lộn / Khóa góc)
} MPC_Status_t;

typedef enum {
    ARM,
    NOT_ARM
} ARM_Status_t;

extern MPC_Status_t MPC_Status;
extern ARM_Status_t ARM_Status;
extern float32_t Throttle;
extern float32_t Moment[3];
extern float32_t angle_desired[3];
extern float32_t angle_rate_desired[3];

extern PID_ALTIDUE_t PID_RATE_ROLL, PID_RATE_PITCH, PID_RATE_YAW;
extern PID_ALTIDUE_t PID_ROLL, PID_PITCH, PID_YAW;

void MPC(void);
void RESET_ALL_PID(void);

#endif /* CORE_INC_CONTROL_FLIGHT_CONTROL_H_ */
