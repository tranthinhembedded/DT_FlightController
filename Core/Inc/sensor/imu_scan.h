#ifndef CORE_INC_SENSOR_IMU_SCAN_H_
#define CORE_INC_SENSOR_IMU_SCAN_H_

#include "main.h"

#define ICM20602_REG_WHO_AM_I            0x75U
#define ICM20602_WHO_AM_I_VALUE          0x12U
#define ICM20602_SPI_READ_MASK           0x80U
#define ICM20602_SPI_TRANSFER_TIMEOUT_MS 10U
#define HMC5883L_I2C_ADDR_7BIT           0x1EU
#define HMC5883L_I2C_ADDR_8BIT           (HMC5883L_I2C_ADDR_7BIT << 1)
#define I2C_SCAN_START_ADDR_7BIT         0x08U
#define I2C_SCAN_END_ADDR_7BIT           0x77U
#define HMC5883L_I2C_PROBE_TIMEOUT_MS    2U

extern volatile uint8_t imu_scan_detected;
extern volatile uint8_t imu_scan_who_am_i;
extern volatile uint8_t imu_scan_expected_who_am_i;
extern volatile uint8_t imu_scan_last_hal_status;
extern volatile uint8_t imu_scan_register_addr;
extern volatile uint8_t imu_scan_read_command;
extern volatile uint32_t imu_scan_attempt_count;
extern volatile uint32_t imu_scan_match_count;
extern volatile uint32_t imu_scan_last_probe_tick_ms;

extern volatile uint8_t compass_scan_detected;
extern volatile uint8_t compass_scan_found_addr_7bit;
extern volatile uint8_t compass_scan_found_addr_8bit;
extern volatile uint8_t compass_scan_expected_addr_7bit;
extern volatile uint8_t compass_scan_last_addr_7bit;
extern volatile uint8_t compass_scan_last_hal_status;
extern volatile uint8_t compass_scan_device_count;
extern volatile uint32_t compass_scan_attempt_count;
extern volatile uint32_t compass_scan_match_count;
extern volatile uint32_t compass_scan_last_probe_tick_ms;

void IMU_Scan_Init(void);
HAL_StatusTypeDef IMU_Scan_Probe(void);
void Compass_Scan_Init(void);
HAL_StatusTypeDef Compass_Scan_Probe(void);

#endif /* CORE_INC_SENSOR_IMU_SCAN_H_ */
