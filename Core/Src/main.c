/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : FLIGHT CONTROLLER - RC RECEIVER READY
  * Updated         : iNav Style Support & Physical RC Setup
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "platform/dma.h"
#include "platform/i2c.h"
#include "platform/tim.h"
#include "platform/usart.h"
#include "platform/gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "sensor/imu_config.h"
#include "control/motor_control.h"
#include "control/flight_control.h"
#include "comm/telemetry.h"
#include "input/rc_input.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t current_time, prev_time, dt;
uint32_t max_dt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Delay_us1(uint32_t us) {
	uint32_t start = TIM2->CNT;
	while ((TIM2->CNT - start) < us);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2); // Start Timer 2 first for Delay_us

  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); Delay_us1(100000);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); Delay_us1(100000);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);


  /* HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1); // Roll
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2); // Pitch
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3); // Throttle
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4); // Yaw

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // SW Arm
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4); // SW Mode */

  TIM3->CCR1 = 1000; TIM3->CCR2 = 1000; TIM4->CCR1 = 1000; TIM4->CCR2 = 1000;

  MPU6050_Init();
  HMC5883L_Init();
  MPU6050_Calibrate();
  RESET_ALL_PID();
  current_time = TIM2->CNT;
  enable_motor = 0;
  ARM_Status = NOT_ARM;
  Throttle = 1000.0f;

  // Initialize RC values to neutral/min since RC is disabled
  RC_Raw_Roll = 1500;
  RC_Raw_Pitch = 1500;
  RC_Raw_Yaw = 1500;
  RC_Raw_Throttle = 1000;
  RC_Raw_SW_Arm = 1000;
  RC_Raw_SW_Mode = 1000;

  // UART1_StartRxToIdle_DMA();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    // 1. Capture Cycle Time (Loop Pacing)
    current_time = TIM2->CNT;
    dt = current_time - prev_time;
    prev_time = current_time;

    if (dt > 2000) dt = 1000;

    // Check max_dt (Lưu giá trị dt lớn nhất, bỏ qua vài vòng lặp đầu nếu cần, nhưng ở đây cứ lưu hết)
    if (dt > max_dt) max_dt = dt;

    // 2. UART Command (PID Tuning ONLY)
    /* if (line_ready) {
        char local[CMD_LINE_SIZE];
        __disable_irq();
        strncpy(local, cmd_ready, CMD_LINE_SIZE);
        local[CMD_LINE_SIZE-1] = 0;
        line_ready = 0;
        __enable_irq();
        ProcessLine(local);
    } */

    // 3. Scheduler
    static uint16_t loop_sched_count = 0;
    loop_sched_count++;

    // --- MEDIUM LOOP (100Hz) --- Call every 10ms (10 * 1ms = 10 calls)
    if (loop_sched_count % 10 == 0) {
        COMPASS_PROCESS();
        if (MagCal.state == MAG_CAL_DONE) {
             Complimentary_Filter_Update(&Complimentary_Filter, &HMC5883L_DATA);
        }
    }

    // --- SLOW LOOP (20Hz) --- Call every 50ms (50 * 1ms = 50 calls)
    if (loop_sched_count % 50 == 0) {
        /* if (HAL_GetTick() - last_telemetry_time > 100) { // Keep legacy check just in case or replace logic
             Send_Telemetry();
             last_telemetry_time = HAL_GetTick();
        } */
        // Other slow checks (Battery, Failsafe) goes here
    }

    // Reset counter to avoid overflow (optional, but good practice if using modulo)
    if (loop_sched_count >= 1000) loop_sched_count = 0;


    // --- FAST LOOP (1kHz) --- Always runs
    // 4. Sensor Update (IMU Only for Fast Loop)
    IMU_PROCESS();

    // 5. Update Filter & PID with REAL dt
    MPU6500_DATA.dt = (float32_t)dt * 1.0e-6f;
    Complimentary_Filter_Predict(&Complimentary_Filter, &MPU6500_DATA);

    // Note: Compass Update moved to Medium Loop

    // 6. Control Loop (MPC)
    MPC();

    // 7. Telemetry Moved to Slow Loop

    // 7. Precise Loop Pacing (Đảm bảo Loop 1000Hz - 1000us)
    while ((TIM2->CNT - current_time) < 1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
