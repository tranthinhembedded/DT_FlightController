#include "sensor/imu_config.h"
#include "platform/delay.h"

// Global variables definition
IMU_RAW_DATA_t MPU6500_RAW_DATA;
IMU_Data_t MPU6500_DATA;

MAG_RAW_DATA_t HMC5883L_RAW_DATA;
MAG_DATA_t HMC5883L_DATA;

MagCal_Simple_t MagCal = {
    .S = 1.0f,
    .state = MAG_CAL_DONE,
    .samples_target = 5000,
	.offset = {4.7840004f, -3.45000076f, 7.36000061f},
	.scale  = {0.937098265f, 0.954184234f, 1.13012183f}
};

Complimentary_Filter_t Complimentary_Filter = {
    .alpha[0] = 0.99, .alpha[1] = 0.99, .alpha[2] = 0.96,
};

// Private variables
static int16_t raw_acc[3];
static int16_t raw_gyro[3];
static float32_t acc_phys[3], gyro_phys[3];

float32_t gyro_final[3] = {0.0f, 0.0f, 0.0f};
float32_t acc_filtered[3] = {0.0f, 0.0f, 0.0f};
const float32_t alpha_acc_soft = 0.20f;
const float32_t alpha_gyro = 0.35f;
float32_t mag_filtered[3] = {0.0f, 0.0f, 0.0f};
uint8_t mag_lpf_inited = 0;

float32_t gyro_bias[3] = {0.0f, 0.0f, 0.0f};
float32_t accel_bias[3] = {0.0f, 0.0f, 0.0f};
uint8_t is_calibrated = 0;

void MPU6050_Init(void) {
    uint8_t data;
    data = 0x00; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);
    data = 0x06; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &data, 1, 100);
    data = 0x08; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 100);
    data = 0x10; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, 100);
    uint8_t bypass_en = 0x02; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x37, 1, &bypass_en, 1, 100);
}

void MPU6050_Calibrate(void) {
    uint8_t buffer[14];
    int32_t sum_acc[3] = {0}, sum_gyro[3] = {0};
    int16_t ra_acc[3], ra_gyro[3];
    int sample_count = 1000;

    for (int i = 0; i < sample_count; i++) {
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 14, 10) == HAL_OK) {
            ra_acc[0] = (int16_t)(buffer[0] << 8 | buffer[1]);
            ra_acc[1] = (int16_t)(buffer[2] << 8 | buffer[3]);
            ra_acc[2] = (int16_t)(buffer[4] << 8 | buffer[5]);
            ra_gyro[0] = (int16_t)(buffer[8] << 8 | buffer[9]);
            ra_gyro[1] = (int16_t)(buffer[10] << 8 | buffer[11]);
            ra_gyro[2] = (int16_t)(buffer[12] << 8 | buffer[13]);

            sum_acc[0] += ra_acc[0]; sum_acc[1] += ra_acc[1]; sum_acc[2] += ra_acc[2];
            sum_gyro[0] += ra_gyro[0]; sum_gyro[1] += ra_gyro[1]; sum_gyro[2] += ra_gyro[2];
        }
        Delay_us(2000);
    }
    gyro_bias[0] = (float)sum_gyro[0] / sample_count;
    gyro_bias[1] = (float)sum_gyro[1] / sample_count;
    gyro_bias[2] = (float)sum_gyro[2] / sample_count;
    accel_bias[0] = (float)sum_acc[0] / sample_count;
    accel_bias[1] = (float)sum_acc[1] / sample_count;
    accel_bias[2] = ((float)sum_acc[2] / sample_count) - 4096.0f;
    is_calibrated = 1;
}

void HMC5883L_Init(void) {
    uint8_t data;
    data = 0x70; HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x00, 1, &data, 1, 100);
    data = 0x20; HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x01, 1, &data, 1, 100);
    data = 0x00; HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
    Delay_us(10000);
}

void IMU_PROCESS(void) {
    uint8_t buffer[14];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 14, 10) == HAL_OK) {
        raw_acc[0] = (int16_t)(buffer[0] << 8 | buffer[1]);
        raw_acc[1] = (int16_t)(buffer[2] << 8 | buffer[3]);
        raw_acc[2] = (int16_t)(buffer[4] << 8 | buffer[5]);
        raw_gyro[0] = (int16_t)(buffer[8] << 8 | buffer[9]);
        raw_gyro[1] = (int16_t)(buffer[10] << 8 | buffer[11]);
        raw_gyro[2] = (int16_t)(buffer[12] << 8 | buffer[13]);

        float32_t acc_temp[3], gyro_temp[3];
        if (is_calibrated) {
            acc_temp[0] = (float32_t)raw_acc[0] - accel_bias[0];
            acc_temp[1] = (float32_t)raw_acc[1] - accel_bias[1];
            acc_temp[2] = (float32_t)raw_acc[2] - accel_bias[2];
            gyro_temp[0] = (float32_t)raw_gyro[0] - gyro_bias[0];
            gyro_temp[1] = (float32_t)raw_gyro[1] - gyro_bias[1];
            gyro_temp[2] = (float32_t)raw_gyro[2] - gyro_bias[2];
        } else {
            acc_temp[0] = raw_acc[0]; acc_temp[1] = raw_acc[1]; acc_temp[2] = raw_acc[2];
            gyro_temp[0] = raw_gyro[0]; gyro_temp[1] = raw_gyro[1]; gyro_temp[2] = raw_gyro[2];
        }

        acc_phys[0] = (acc_temp[0] / ACCEL_SENSITIVITY) * GRAVITY_EARTH;
        acc_phys[1] = -(acc_temp[1] / ACCEL_SENSITIVITY) * GRAVITY_EARTH;
        acc_phys[2] = -(acc_temp[2] / ACCEL_SENSITIVITY) * GRAVITY_EARTH;
        gyro_phys[0] = (gyro_temp[0] / GYRO_SENSITIVITY) * DEG_TO_RAD;
        gyro_phys[1] = -(gyro_temp[1] / GYRO_SENSITIVITY) * DEG_TO_RAD;
        gyro_phys[2] = -(gyro_temp[2] / GYRO_SENSITIVITY) * DEG_TO_RAD;

        // Accel LPF
        static uint8_t acc_lpf_inited = 0;
        if (!acc_lpf_inited) {
            acc_filtered[0] = acc_phys[0]; acc_filtered[1] = acc_phys[1]; acc_filtered[2] = acc_phys[2];
            acc_lpf_inited = 1;
        } else {
            for (int i = 0; i < 3; i++) acc_filtered[i] += alpha_acc_soft * (acc_phys[i] - acc_filtered[i]);
        }

        // Gyro LPF
        static uint8_t gyro_lpf_inited = 0;
        if (!gyro_lpf_inited) {
            gyro_final[0] = gyro_phys[0]; gyro_final[1] = gyro_phys[1]; gyro_final[2] = gyro_phys[2];
            gyro_lpf_inited = 1;
        } else {
            for (int i = 0; i < 3; i++) gyro_final[i] += alpha_gyro * (gyro_phys[i] - gyro_final[i]);
        }

        MPU6500_DATA.acc[0] = acc_filtered[0]; MPU6500_DATA.acc[1] = acc_filtered[1]; MPU6500_DATA.acc[2] = acc_filtered[2];
        MPU6500_DATA.w[0] = gyro_final[0]; MPU6500_DATA.w[1] = gyro_final[1]; MPU6500_DATA.w[2] = gyro_final[2];
    }
}

void COMPASS_PROCESS(void) {
    uint8_t buffer[6];
    int16_t raw_x, raw_y, raw_z;
    if (HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x03, 1, buffer, 6, 100) == HAL_OK) {
        raw_x = (int16_t)(buffer[0] << 8 | buffer[1]);
        raw_z = (int16_t)(buffer[2] << 8 | buffer[3]);
        raw_y = (int16_t)(buffer[4] << 8 | buffer[5]);

        HMC5883L_RAW_DATA.mag[0] = -(float32_t)raw_x * HMC5883L_SCALE_UT;
        HMC5883L_RAW_DATA.mag[1] = (float32_t)raw_y * HMC5883L_SCALE_UT;
        HMC5883L_RAW_DATA.mag[2] = -(float32_t)raw_z * HMC5883L_SCALE_UT;

        MagCal_Update(&MagCal, &HMC5883L_RAW_DATA, &HMC5883L_DATA);

        if (MagCal.state == MAG_CAL_DONE) {
            if (!mag_lpf_inited) {
                mag_filtered[0] = HMC5883L_DATA.mag_uT[0];
                mag_filtered[1] = HMC5883L_DATA.mag_uT[1];
                mag_filtered[2] = HMC5883L_DATA.mag_uT[2];
                mag_lpf_inited = 1;
            } else {
                mag_filtered[0] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[0] - mag_filtered[0]);
                mag_filtered[1] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[1] - mag_filtered[1]);
                mag_filtered[2] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[2] - mag_filtered[2]);
            }
            HMC5883L_DATA.mag_uT[0] = mag_filtered[0];
            HMC5883L_DATA.mag_uT[1] = mag_filtered[1];
            HMC5883L_DATA.mag_uT[2] = mag_filtered[2];
        }
    }
}
