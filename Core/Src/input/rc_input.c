#include "input/rc_input.h"

// --- BIẾN ĐỌC RC THÔ (RAW PWM) ---
// Đơn vị: micro giây (us). Giá trị chuẩn: 1000 - 2000
volatile uint32_t RC_Raw_Roll     = 0; // TIM5 CH1
volatile uint32_t RC_Raw_Pitch    = 0; // TIM5 CH2
volatile uint32_t RC_Raw_Throttle = 0; // TIM5 CH3
volatile uint32_t RC_Raw_Yaw      = 0; // TIM5 CH4
volatile uint32_t RC_Raw_SW_Arm   = 0; // TIM1 CH1
volatile uint32_t RC_Raw_SW_Mode  = 0; // TIM1 CH4

// Biến phụ trợ để tính toán thời gian
static uint32_t val_start_roll = 0, val_start_pitch = 0, val_start_thr = 0, val_start_yaw = 0;
static uint32_t val_start_arm = 0, val_start_mode = 0;

// HÀM ĐỌC ĐỘ RỘNG XUNG PWM (Input Capture)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    // --- XỬ LÝ TIM5 (4 Kênh chính: A, E, T, R) ---
    if (htim->Instance == TIM5)
    {
        // 1. ROLL (PA0 - CH1)
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                val_start_roll = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Sườn lên
            } else {
                RC_Raw_Roll = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) - val_start_roll; // Sườn xuống
            }
        }
        // 2. PITCH (PA1 - CH2)
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
                val_start_pitch = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            } else {
                RC_Raw_Pitch = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) - val_start_pitch;
            }
        }
        // 3. THROTTLE (PA2 - CH3)
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
                val_start_thr = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            } else {
                RC_Raw_Throttle = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3) - val_start_thr;
            }
        }
        // 4. YAW (PA3 - CH4)
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
                val_start_yaw = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            } else {
                RC_Raw_Yaw = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4) - val_start_yaw;
            }
        }
    }
    // --- XỬ LÝ TIM1 (2 Công tắc: SW_Arm, SW_Mode) ---
    // Lưu ý: TIM1 dùng PA8 (CH1) và PA11 (CH4)
    else if (htim->Instance == TIM1)
    {
        // 5. SW ARM (PA8 - CH1)
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {
                val_start_arm = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            } else {
                // Xử lý tràn Timer 16-bit (nếu có)
                uint32_t val_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                if (val_end >= val_start_arm)
                    RC_Raw_SW_Arm = val_end - val_start_arm;
                else
                    RC_Raw_SW_Arm = (0xFFFF - val_start_arm) + val_end;
            }
        }
        // 6. SW MODE (PA11 - CH4)
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET) {
                val_start_mode = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            } else {
                uint32_t val_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
                if (val_end >= val_start_mode)
                    RC_Raw_SW_Mode = val_end - val_start_mode;
                else
                    RC_Raw_SW_Mode = (0xFFFF - val_start_mode) + val_end;
            }
        }
    }
}
