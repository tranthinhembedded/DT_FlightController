#ifndef CORE_INC_SENSOR_CHECK_SENSOR_HEALTH_H_
#define CORE_INC_SENSOR_CHECK_SENSOR_HEALTH_H_

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

#define BMP280_I2C_ADDR_7BIT_PRIMARY     0x76U
#define BMP280_I2C_ADDR_7BIT_SECONDARY   0x77U
#define BMP280_REG_CHIP_ID               0xD0U
#define BMP280_CHIP_ID                   0x58U
#define BMP280_I2C_PROBE_TIMEOUT_MS      2U
#define BMP280_I2C_READ_TIMEOUT_MS       10U

/* IMU health */
extern volatile uint8_t sensor_health_imu_ok;
extern volatile uint8_t sensor_health_imu_who_am_i;
extern volatile uint8_t sensor_health_imu_expected_who_am_i;
extern volatile uint8_t sensor_health_imu_last_hal_status;
extern volatile uint8_t sensor_health_imu_register_addr;
extern volatile uint8_t sensor_health_imu_read_command;
extern volatile uint32_t sensor_health_imu_attempt_count;
extern volatile uint32_t sensor_health_imu_match_count;
extern volatile uint32_t sensor_health_imu_last_probe_tick_ms;

/* Compass health */
extern volatile uint8_t sensor_health_compass_ok;
extern volatile uint8_t sensor_health_compass_found_addr_7bit;
extern volatile uint8_t sensor_health_compass_found_addr_8bit;
extern volatile uint8_t sensor_health_compass_expected_addr_7bit;
extern volatile uint8_t sensor_health_compass_last_addr_7bit;
extern volatile uint8_t sensor_health_compass_last_hal_status;
extern volatile uint8_t sensor_health_compass_device_count;
extern volatile uint32_t sensor_health_compass_attempt_count;
extern volatile uint32_t sensor_health_compass_match_count;
extern volatile uint32_t sensor_health_compass_last_probe_tick_ms;

/* BMP280 health */
extern volatile uint8_t sensor_health_bmp280_ok;
extern volatile uint8_t sensor_health_bmp280_found_addr_7bit;
extern volatile uint8_t sensor_health_bmp280_found_addr_8bit;
extern volatile uint8_t sensor_health_bmp280_expected_addr_7bit_primary;
extern volatile uint8_t sensor_health_bmp280_expected_addr_7bit_secondary;
extern volatile uint8_t sensor_health_bmp280_last_addr_7bit;
extern volatile uint8_t sensor_health_bmp280_last_hal_status;
extern volatile uint8_t sensor_health_bmp280_chip_id;
extern volatile uint8_t sensor_health_bmp280_expected_chip_id;
extern volatile uint32_t sensor_health_bmp280_attempt_count;
extern volatile uint32_t sensor_health_bmp280_match_count;
extern volatile uint32_t sensor_health_bmp280_last_probe_tick_ms;

/* Aggregated health / safety */
extern volatile uint8_t sensor_health_all_ok;

/* API */
void SensorHealth_Init(void);
void SensorHealth_Reset(void);

HAL_StatusTypeDef SensorHealth_ProbeIMU(void);
HAL_StatusTypeDef SensorHealth_ProbeCompass(void);
HAL_StatusTypeDef SensorHealth_ProbeBMP280(void);

void SensorHealth_ProbeAll(void);

/* LED helper: safe = all sensors connected and identified correctly */
void SensorHealth_UpdateSafeLed(void);

#endif /* CORE_INC_SENSOR_CHECK_SENSOR_HEALTH_H_ */