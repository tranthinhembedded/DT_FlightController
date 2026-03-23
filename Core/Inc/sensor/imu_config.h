#ifndef CORE_INC_SENSOR_IMU_CONFIG_H_
#define CORE_INC_SENSOR_IMU_CONFIG_H_

#include "main.h"
#include "sensor/complementary_filter.h"
#include "sensor/mag_calibration.h"
#include "platform/i2c.h"

// Define constants
#define HMC5883L_ADDR      (0x1E << 1)
#define HMC5883L_SCALE_GAUSS 0.00092f
#define HMC5883L_SCALE_UT    (HMC5883L_SCALE_GAUSS * 100.0f)
#define MPU6050_ADDR         (0x68 << 1)
#define GYRO_SENSITIVITY     65.5f
#define ACCEL_SENSITIVITY    4096.0f
#define GRAVITY_EARTH        9.80665f
#define MAG_LPF_ALPHA   0.08f

// Global variables
extern IMU_RAW_DATA_t MPU6500_RAW_DATA;
extern IMU_Data_t MPU6500_DATA;
extern MAG_RAW_DATA_t HMC5883L_RAW_DATA;
extern MAG_DATA_t HMC5883L_DATA;
extern MagCal_Simple_t MagCal;
extern Complimentary_Filter_t Complimentary_Filter;

extern float32_t gyro_final[3];
extern float32_t acc_filtered[3];
extern float32_t mag_filtered[3];
extern uint8_t is_calibrated;

// Functions
void MPU6050_Init(void);
void MPU6050_Calibrate(void);
void HMC5883L_Init(void);
void IMU_PROCESS(void);
void COMPASS_PROCESS(void);

#endif /* CORE_INC_SENSOR_IMU_CONFIG_H_ */
