#include "sensor/imu_scan.h"

#include "platform/i2c.h"
#include "platform/spi.h"
#include "platform/delay.h"

volatile uint8_t imu_scan_detected = 0U;
volatile uint8_t imu_scan_who_am_i = 0U;
volatile uint8_t imu_scan_expected_who_am_i = ICM20602_WHO_AM_I_VALUE;
volatile uint8_t imu_scan_last_hal_status = (uint8_t)HAL_ERROR;
volatile uint8_t imu_scan_register_addr = ICM20602_REG_WHO_AM_I;
volatile uint8_t imu_scan_read_command = (ICM20602_REG_WHO_AM_I | ICM20602_SPI_READ_MASK);
volatile uint32_t imu_scan_attempt_count = 0U;
volatile uint32_t imu_scan_match_count = 0U;
volatile uint32_t imu_scan_last_probe_tick_ms = 0U;

volatile uint8_t compass_scan_detected = 0U;
volatile uint8_t compass_scan_found_addr_7bit = 0xFFU;
volatile uint8_t compass_scan_found_addr_8bit = 0xFFU;
volatile uint8_t compass_scan_expected_addr_7bit = HMC5883L_I2C_ADDR_7BIT;
volatile uint8_t compass_scan_last_addr_7bit = I2C_SCAN_START_ADDR_7BIT;
volatile uint8_t compass_scan_last_hal_status = (uint8_t)HAL_ERROR;
volatile uint8_t compass_scan_device_count = 0U;
volatile uint32_t compass_scan_attempt_count = 0U;
volatile uint32_t compass_scan_match_count = 0U;
volatile uint32_t compass_scan_last_probe_tick_ms = 0U;

/* BMP280 scan state */
volatile uint8_t bmp280_scan_detected = 0U;
volatile uint8_t bmp280_scan_found_addr_7bit = 0xFFU;
volatile uint8_t bmp280_scan_found_addr_8bit = 0xFFU;
volatile uint8_t bmp280_scan_expected_addr_7bit_primary = BMP280_I2C_ADDR_7BIT_PRIMARY;
volatile uint8_t bmp280_scan_expected_addr_7bit_secondary = BMP280_I2C_ADDR_7BIT_SECONDARY;
volatile uint8_t bmp280_scan_last_addr_7bit = 0xFFU;
volatile uint8_t bmp280_scan_last_hal_status = (uint8_t)HAL_ERROR;
volatile uint8_t bmp280_scan_chip_id = 0x00U;
volatile uint8_t bmp280_scan_expected_chip_id = BMP280_CHIP_ID;
volatile uint32_t bmp280_scan_attempt_count = 0U;
volatile uint32_t bmp280_scan_match_count = 0U;
volatile uint32_t bmp280_scan_last_probe_tick_ms = 0U;

/* ON only when IMU + compass + BMP280 are all detected correctly */
volatile uint8_t sensors_scan_all_detected = 0U;

static void Sensors_Scan_UpdateCombinedStatus(void)
{
    sensors_scan_all_detected =
        ((imu_scan_detected != 0U) &&
         (compass_scan_detected != 0U) &&
         (bmp280_scan_detected != 0U))
            ? 1U
            : 0U;
}

static void IMU_Scan_Select(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static void IMU_Scan_Deselect(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef IMU_Scan_ReadRegister(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buffer[2] = {(uint8_t)(reg | ICM20602_SPI_READ_MASK), 0x00U};
    uint8_t rx_buffer[2] = {0U, 0U};
    HAL_StatusTypeDef status;

    if (value == NULL)
    {
        return HAL_ERROR;
    }

    IMU_Scan_Select();
    status = HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 2U, ICM20602_SPI_TRANSFER_TIMEOUT_MS);
    IMU_Scan_Deselect();

    if (status == HAL_OK)
    {
        *value = rx_buffer[1];
    }

    return status;
}

static HAL_StatusTypeDef BMP280_Scan_ReadRegister(uint8_t dev_addr_7bit, uint8_t reg, uint8_t *value)
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

void IMU_Scan_Init(void)
{
    imu_scan_detected = 0U;
    imu_scan_who_am_i = 0U;
    imu_scan_expected_who_am_i = ICM20602_WHO_AM_I_VALUE;
    imu_scan_last_hal_status = (uint8_t)HAL_ERROR;
    imu_scan_register_addr = ICM20602_REG_WHO_AM_I;
    imu_scan_read_command = (ICM20602_REG_WHO_AM_I | ICM20602_SPI_READ_MASK);
    imu_scan_attempt_count = 0U;
    imu_scan_match_count = 0U;
    imu_scan_last_probe_tick_ms = 0U;

    sensors_scan_all_detected = 0U;

    IMU_Scan_Deselect();
    Delay_ms_blocking(100U);
}

void Compass_Scan_Init(void)
{
    compass_scan_detected = 0U;
    compass_scan_found_addr_7bit = 0xFFU;
    compass_scan_found_addr_8bit = 0xFFU;
    compass_scan_expected_addr_7bit = HMC5883L_I2C_ADDR_7BIT;
    compass_scan_last_addr_7bit = I2C_SCAN_START_ADDR_7BIT;
    compass_scan_last_hal_status = (uint8_t)HAL_ERROR;
    compass_scan_device_count = 0U;
    compass_scan_attempt_count = 0U;
    compass_scan_match_count = 0U;
    compass_scan_last_probe_tick_ms = 0U;

    sensors_scan_all_detected = 0U;
}

void BMP280_Scan_Init(void)
{
    bmp280_scan_detected = 0U;
    bmp280_scan_found_addr_7bit = 0xFFU;
    bmp280_scan_found_addr_8bit = 0xFFU;
    bmp280_scan_expected_addr_7bit_primary = BMP280_I2C_ADDR_7BIT_PRIMARY;
    bmp280_scan_expected_addr_7bit_secondary = BMP280_I2C_ADDR_7BIT_SECONDARY;
    bmp280_scan_last_addr_7bit = 0xFFU;
    bmp280_scan_last_hal_status = (uint8_t)HAL_ERROR;
    bmp280_scan_chip_id = 0x00U;
    bmp280_scan_expected_chip_id = BMP280_CHIP_ID;
    bmp280_scan_attempt_count = 0U;
    bmp280_scan_match_count = 0U;
    bmp280_scan_last_probe_tick_ms = 0U;

    sensors_scan_all_detected = 0U;
}

HAL_StatusTypeDef IMU_Scan_Probe(void)
{
    uint8_t who_am_i = 0U;
    HAL_StatusTypeDef status = IMU_Scan_ReadRegister(ICM20602_REG_WHO_AM_I, &who_am_i);

    imu_scan_last_probe_tick_ms = HAL_GetTick();
    imu_scan_attempt_count++;
    imu_scan_last_hal_status = (uint8_t)status;

    if (status == HAL_OK)
    {
        imu_scan_who_am_i = who_am_i;
        imu_scan_detected = (who_am_i == imu_scan_expected_who_am_i) ? 1U : 0U;
        if (imu_scan_detected != 0U)
        {
            imu_scan_match_count++;
        }
    }
    else
    {
        imu_scan_detected = 0U;
    }

    Sensors_Scan_UpdateCombinedStatus();
    return status;
}

HAL_StatusTypeDef Compass_Scan_Probe(void)
{
    HAL_StatusTypeDef expected_status = HAL_ERROR;

    compass_scan_detected = 0U;
    compass_scan_found_addr_7bit = 0xFFU;
    compass_scan_found_addr_8bit = 0xFFU;
    compass_scan_device_count = 0U;
    compass_scan_last_probe_tick_ms = HAL_GetTick();
    compass_scan_attempt_count++;

    for (uint8_t addr = I2C_SCAN_START_ADDR_7BIT; addr <= I2C_SCAN_END_ADDR_7BIT; addr++)
    {
        HAL_StatusTypeDef status;

        compass_scan_last_addr_7bit = addr;
        status = HAL_I2C_IsDeviceReady(&hi2c1,
                                       (uint16_t)(addr << 1),
                                       1U,
                                       HMC5883L_I2C_PROBE_TIMEOUT_MS);

        if (addr == compass_scan_expected_addr_7bit)
        {
            expected_status = status;
            compass_scan_last_hal_status = (uint8_t)status;
        }

        if (status == HAL_OK)
        {
            compass_scan_device_count++;

            if (compass_scan_found_addr_7bit == 0xFFU)
            {
                compass_scan_found_addr_7bit = addr;
                compass_scan_found_addr_8bit = (uint8_t)(addr << 1);
            }

            if (addr == compass_scan_expected_addr_7bit)
            {
                compass_scan_detected = 1U;
                compass_scan_found_addr_7bit = addr;
                compass_scan_found_addr_8bit = (uint8_t)(addr << 1);
                compass_scan_match_count++;
            }
        }
    }

    Sensors_Scan_UpdateCombinedStatus();
    return expected_status;
}

HAL_StatusTypeDef BMP280_Scan_Probe(void)
{
    HAL_StatusTypeDef final_status = HAL_ERROR;
    const uint8_t candidate_addrs[2] = {
        BMP280_I2C_ADDR_7BIT_PRIMARY,
        BMP280_I2C_ADDR_7BIT_SECONDARY};

    bmp280_scan_detected = 0U;
    bmp280_scan_found_addr_7bit = 0xFFU;
    bmp280_scan_found_addr_8bit = 0xFFU;
    bmp280_scan_last_addr_7bit = 0xFFU;
    bmp280_scan_chip_id = 0x00U;
    bmp280_scan_last_probe_tick_ms = HAL_GetTick();
    bmp280_scan_attempt_count++;

    for (uint32_t i = 0U; i < 2U; i++)
    {
        uint8_t addr = candidate_addrs[i];
        uint8_t chip_id = 0U;
        HAL_StatusTypeDef ready_status;
        HAL_StatusTypeDef read_status;

        bmp280_scan_last_addr_7bit = addr;

        ready_status = HAL_I2C_IsDeviceReady(&hi2c1,
                                             (uint16_t)(addr << 1),
                                             1U,
                                             BMP280_I2C_PROBE_TIMEOUT_MS);

        bmp280_scan_last_hal_status = (uint8_t)ready_status;
        final_status = ready_status;

        if (ready_status != HAL_OK)
        {
            continue;
        }

        read_status = BMP280_Scan_ReadRegister(addr, BMP280_REG_CHIP_ID, &chip_id);
        bmp280_scan_last_hal_status = (uint8_t)read_status;
        final_status = read_status;

        if (read_status != HAL_OK)
        {
            continue;
        }

        bmp280_scan_chip_id = chip_id;

        if (chip_id == bmp280_scan_expected_chip_id)
        {
            bmp280_scan_detected = 1U;
            bmp280_scan_found_addr_7bit = addr;
            bmp280_scan_found_addr_8bit = (uint8_t)(addr << 1);
            bmp280_scan_match_count++;
            break;
        }
    }

    Sensors_Scan_UpdateCombinedStatus();
    return final_status;
}