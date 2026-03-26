#include "sensor/check_sensor_health.h"

#include "platform/i2c.h"
#include "platform/spi.h"
#include "platform/delay.h"
#include "platform/gpio.h"

volatile uint8_t sensor_health_imu_ok = 0U;
volatile uint8_t sensor_health_imu_who_am_i = 0U;
volatile uint8_t sensor_health_imu_expected_who_am_i = ICM20602_WHO_AM_I_VALUE;
volatile uint8_t sensor_health_imu_last_hal_status = (uint8_t)HAL_ERROR;
volatile uint8_t sensor_health_imu_register_addr = ICM20602_REG_WHO_AM_I;
volatile uint8_t sensor_health_imu_read_command = (ICM20602_REG_WHO_AM_I | ICM20602_SPI_READ_MASK);
volatile uint32_t sensor_health_imu_attempt_count = 0U;
volatile uint32_t sensor_health_imu_match_count = 0U;
volatile uint32_t sensor_health_imu_last_probe_tick_ms = 0U;

volatile uint8_t sensor_health_compass_ok = 0U;
volatile uint8_t sensor_health_compass_found_addr_7bit = 0xFFU;
volatile uint8_t sensor_health_compass_found_addr_8bit = 0xFFU;
volatile uint8_t sensor_health_compass_expected_addr_7bit = HMC5883L_I2C_ADDR_7BIT;
volatile uint8_t sensor_health_compass_last_addr_7bit = I2C_SCAN_START_ADDR_7BIT;
volatile uint8_t sensor_health_compass_last_hal_status = (uint8_t)HAL_ERROR;
volatile uint8_t sensor_health_compass_device_count = 0U;
volatile uint32_t sensor_health_compass_attempt_count = 0U;
volatile uint32_t sensor_health_compass_match_count = 0U;
volatile uint32_t sensor_health_compass_last_probe_tick_ms = 0U;

volatile uint8_t sensor_health_bmp280_ok = 0U;
volatile uint8_t sensor_health_bmp280_found_addr_7bit = 0xFFU;
volatile uint8_t sensor_health_bmp280_found_addr_8bit = 0xFFU;
volatile uint8_t sensor_health_bmp280_expected_addr_7bit_primary = BMP280_I2C_ADDR_7BIT_PRIMARY;
volatile uint8_t sensor_health_bmp280_expected_addr_7bit_secondary = BMP280_I2C_ADDR_7BIT_SECONDARY;
volatile uint8_t sensor_health_bmp280_last_addr_7bit = 0xFFU;
volatile uint8_t sensor_health_bmp280_last_hal_status = (uint8_t)HAL_ERROR;
volatile uint8_t sensor_health_bmp280_chip_id = 0x00U;
volatile uint8_t sensor_health_bmp280_expected_chip_id = BMP280_CHIP_ID;
volatile uint32_t sensor_health_bmp280_attempt_count = 0U;
volatile uint32_t sensor_health_bmp280_match_count = 0U;
volatile uint32_t sensor_health_bmp280_last_probe_tick_ms = 0U;

volatile uint8_t sensor_health_all_ok = 0U;

static void SensorHealth_UpdateCombinedStatus(void)
{
    sensor_health_all_ok =
        ((sensor_health_imu_ok != 0U) &&
         (sensor_health_compass_ok != 0U) &&
         (sensor_health_bmp280_ok != 0U)) ? 1U : 0U;
}

static void SensorHealth_IMU_Select(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static void SensorHealth_IMU_Deselect(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef SensorHealth_IMU_ReadRegister(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buffer[2] = { (uint8_t)(reg | ICM20602_SPI_READ_MASK), 0x00U };
    uint8_t rx_buffer[2] = { 0U, 0U };
    HAL_StatusTypeDef status;

    if (value == NULL)
    {
        return HAL_ERROR;
    }

    SensorHealth_IMU_Select();
    status = HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 2U, ICM20602_SPI_TRANSFER_TIMEOUT_MS);
    SensorHealth_IMU_Deselect();

    if (status == HAL_OK)
    {
        *value = rx_buffer[1];
    }

    return status;
}

static HAL_StatusTypeDef SensorHealth_BMP280_ReadRegister(uint8_t dev_addr_7bit, uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;

    if (value == NULL)
    {
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Read(&hi2c1,
                              (uint16_t)(dev_addr_7bit << 1),
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              value,
                              1U,
                              BMP280_I2C_READ_TIMEOUT_MS);

    return status;
}

void SensorHealth_Reset(void)
{
    sensor_health_imu_ok = 0U;
    sensor_health_imu_who_am_i = 0U;
    sensor_health_imu_expected_who_am_i = ICM20602_WHO_AM_I_VALUE;
    sensor_health_imu_last_hal_status = (uint8_t)HAL_ERROR;
    sensor_health_imu_register_addr = ICM20602_REG_WHO_AM_I;
    sensor_health_imu_read_command = (ICM20602_REG_WHO_AM_I | ICM20602_SPI_READ_MASK);
    sensor_health_imu_attempt_count = 0U;
    sensor_health_imu_match_count = 0U;
    sensor_health_imu_last_probe_tick_ms = 0U;

    sensor_health_compass_ok = 0U;
    sensor_health_compass_found_addr_7bit = 0xFFU;
    sensor_health_compass_found_addr_8bit = 0xFFU;
    sensor_health_compass_expected_addr_7bit = HMC5883L_I2C_ADDR_7BIT;
    sensor_health_compass_last_addr_7bit = I2C_SCAN_START_ADDR_7BIT;
    sensor_health_compass_last_hal_status = (uint8_t)HAL_ERROR;
    sensor_health_compass_device_count = 0U;
    sensor_health_compass_attempt_count = 0U;
    sensor_health_compass_match_count = 0U;
    sensor_health_compass_last_probe_tick_ms = 0U;

    sensor_health_bmp280_ok = 0U;
    sensor_health_bmp280_found_addr_7bit = 0xFFU;
    sensor_health_bmp280_found_addr_8bit = 0xFFU;
    sensor_health_bmp280_expected_addr_7bit_primary = BMP280_I2C_ADDR_7BIT_PRIMARY;
    sensor_health_bmp280_expected_addr_7bit_secondary = BMP280_I2C_ADDR_7BIT_SECONDARY;
    sensor_health_bmp280_last_addr_7bit = 0xFFU;
    sensor_health_bmp280_last_hal_status = (uint8_t)HAL_ERROR;
    sensor_health_bmp280_chip_id = 0x00U;
    sensor_health_bmp280_expected_chip_id = BMP280_CHIP_ID;
    sensor_health_bmp280_attempt_count = 0U;
    sensor_health_bmp280_match_count = 0U;
    sensor_health_bmp280_last_probe_tick_ms = 0U;

    sensor_health_all_ok = 0U;
}

void SensorHealth_Init(void)
{
    SensorHealth_Reset();
    SensorHealth_IMU_Deselect();
    Delay_ms_blocking(100U);
}

HAL_StatusTypeDef SensorHealth_ProbeIMU(void)
{
    uint8_t who_am_i = 0U;
    HAL_StatusTypeDef status = SensorHealth_IMU_ReadRegister(ICM20602_REG_WHO_AM_I, &who_am_i);

    sensor_health_imu_last_probe_tick_ms = HAL_GetTick();
    sensor_health_imu_attempt_count++;
    sensor_health_imu_last_hal_status = (uint8_t)status;

    if (status == HAL_OK)
    {
        sensor_health_imu_who_am_i = who_am_i;
        sensor_health_imu_ok = (who_am_i == sensor_health_imu_expected_who_am_i) ? 1U : 0U;
        if (sensor_health_imu_ok != 0U)
        {
            sensor_health_imu_match_count++;
        }
    }
    else
    {
        sensor_health_imu_ok = 0U;
    }

    SensorHealth_UpdateCombinedStatus();
    return status;
}

HAL_StatusTypeDef SensorHealth_ProbeCompass(void)
{
    HAL_StatusTypeDef expected_status = HAL_ERROR;

    sensor_health_compass_ok = 0U;
    sensor_health_compass_found_addr_7bit = 0xFFU;
    sensor_health_compass_found_addr_8bit = 0xFFU;
    sensor_health_compass_device_count = 0U;
    sensor_health_compass_last_probe_tick_ms = HAL_GetTick();
    sensor_health_compass_attempt_count++;

    for (uint8_t addr = I2C_SCAN_START_ADDR_7BIT; addr <= I2C_SCAN_END_ADDR_7BIT; addr++)
    {
        HAL_StatusTypeDef status;

        sensor_health_compass_last_addr_7bit = addr;
        status = HAL_I2C_IsDeviceReady(&hi2c1,
                                       (uint16_t)(addr << 1),
                                       1U,
                                       HMC5883L_I2C_PROBE_TIMEOUT_MS);

        if (addr == sensor_health_compass_expected_addr_7bit)
        {
            expected_status = status;
            sensor_health_compass_last_hal_status = (uint8_t)status;
        }

        if (status == HAL_OK)
        {
            sensor_health_compass_device_count++;

            if (sensor_health_compass_found_addr_7bit == 0xFFU)
            {
                sensor_health_compass_found_addr_7bit = addr;
                sensor_health_compass_found_addr_8bit = (uint8_t)(addr << 1);
            }

            if (addr == sensor_health_compass_expected_addr_7bit)
            {
                sensor_health_compass_ok = 1U;
                sensor_health_compass_found_addr_7bit = addr;
                sensor_health_compass_found_addr_8bit = (uint8_t)(addr << 1);
                sensor_health_compass_match_count++;
            }
        }
    }

    SensorHealth_UpdateCombinedStatus();
    return expected_status;
}

HAL_StatusTypeDef SensorHealth_ProbeBMP280(void)
{
    HAL_StatusTypeDef final_status = HAL_ERROR;
    const uint8_t candidate_addrs[2] = {
        BMP280_I2C_ADDR_7BIT_PRIMARY,
        BMP280_I2C_ADDR_7BIT_SECONDARY
    };

    sensor_health_bmp280_ok = 0U;
    sensor_health_bmp280_found_addr_7bit = 0xFFU;
    sensor_health_bmp280_found_addr_8bit = 0xFFU;
    sensor_health_bmp280_last_addr_7bit = 0xFFU;
    sensor_health_bmp280_chip_id = 0x00U;
    sensor_health_bmp280_last_probe_tick_ms = HAL_GetTick();
    sensor_health_bmp280_attempt_count++;

    for (uint32_t i = 0U; i < 2U; i++)
    {
        uint8_t addr = candidate_addrs[i];
        uint8_t chip_id = 0U;
        HAL_StatusTypeDef ready_status;
        HAL_StatusTypeDef read_status;

        sensor_health_bmp280_last_addr_7bit = addr;

        ready_status = HAL_I2C_IsDeviceReady(&hi2c1,
                                             (uint16_t)(addr << 1),
                                             1U,
                                             BMP280_I2C_PROBE_TIMEOUT_MS);

        sensor_health_bmp280_last_hal_status = (uint8_t)ready_status;
        final_status = ready_status;

        if (ready_status != HAL_OK)
        {
            continue;
        }

        read_status = SensorHealth_BMP280_ReadRegister(addr, BMP280_REG_CHIP_ID, &chip_id);
        sensor_health_bmp280_last_hal_status = (uint8_t)read_status;
        final_status = read_status;

        if (read_status != HAL_OK)
        {
            continue;
        }

        sensor_health_bmp280_chip_id = chip_id;

        if (chip_id == sensor_health_bmp280_expected_chip_id)
        {
            sensor_health_bmp280_ok = 1U;
            sensor_health_bmp280_found_addr_7bit = addr;
            sensor_health_bmp280_found_addr_8bit = (uint8_t)(addr << 1);
            sensor_health_bmp280_match_count++;
            break;
        }
    }

    SensorHealth_UpdateCombinedStatus();
    return final_status;
}

void SensorHealth_ProbeAll(void)
{
    (void)SensorHealth_ProbeIMU();
    (void)SensorHealth_ProbeCompass();
    (void)SensorHealth_ProbeBMP280();
}

void SensorHealth_UpdateSafeLed(void)
{
    /* PC13 active-low on many STM32 boards:
       all sensors OK => LED safe indication ON */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,
                      (sensor_health_all_ok != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}