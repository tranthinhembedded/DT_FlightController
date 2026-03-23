#include "comm/telemetry.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

float vbat = 11.1f; // Bạn cần đọc ADC pin thực tế để cập nhật biến này
uint32_t last_telemetry_time = 0;

static uint8_t rx_dma_buf[RX_DMA_SIZE];
static char cmd_work[CMD_LINE_SIZE];
static uint16_t cmd_len = 0;
char cmd_ready[CMD_LINE_SIZE];
volatile uint8_t line_ready = 0;

void UART1_StartRxToIdle_DMA(void) {
    HAL_UART_DMAStop(&huart1);
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_SIZE) != HAL_OK) {
        HAL_UART_DeInit(&huart1);
        HAL_UART_Init(&huart1);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_SIZE);
    }
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

// Xử lý lệnh Tune PID từ GUI
void ProcessLine(char *line) {
    size_t n = strlen(line);
    while (n > 0 && (line[n-1] == '\r' || line[n-1] == '\n')) {
        line[n-1] = 0; n--;
    }
    if (n == 0) return;

    char *tok = strtok(line, ":");
    if (!tok) return;

    if (strcmp(tok, "PID") == 0) {
        char *axis = strtok(NULL, ":");
        char *s_kp = strtok(NULL, ":");
        char *s_ki = strtok(NULL, ":");
        char *s_kd = strtok(NULL, ":");

        if (axis && s_kp && s_ki && s_kd) {
            float p = strtof(s_kp, NULL);
            float i = strtof(s_ki, NULL);
            float d = strtof(s_kd, NULL);

            // 1. TUNE ANGLE LOOP (ROLL & PITCH)
            if (strcmp(axis, "ANG") == 0) {
                PID_ROLL.kp = p; PID_ROLL.ki = i; PID_ROLL.kd = d;
                PID_PITCH.kp = p; PID_PITCH.ki = i; PID_PITCH.kd = d;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }

            // 2. TUNE YAW ANGLE LOOP (OUTER - MỚI)
            else if (strcmp(axis, "YANG") == 0) {
                PID_YAW.kp = p; PID_YAW.ki = i; PID_YAW.kd = d;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }

            // 3. TUNE YAW RATE LOOP (INNER - CŨ)
            else if (strcmp(axis, "YAW") == 0) {
                PID_RATE_YAW.kp = p; PID_RATE_YAW.ki = i; PID_RATE_YAW.kd = d;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }

            // ACK
            if (huart1.gState == HAL_UART_STATE_READY) {
                char ack[64];
                int len = snprintf(ack, sizeof(ack), "MSG:UPDATED %s P=%.3f\n", axis, p);
                if (len > 0) HAL_UART_Transmit(&huart1, (uint8_t*)ack, (uint16_t)len, 10);
            }
        }
    }
}

void Send_Telemetry(void) {
    if (huart1.gState != HAL_UART_STATE_READY) return;
    static char tx_buf[128];
    // Gửi dữ liệu góc để vẽ đồ thị
    int len = sprintf(tx_buf, "{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"v\":%.1f}\n",
            Complimentary_Filter.Euler_Angle_Deg[0],
            Complimentary_Filter.Euler_Angle_Deg[1],
            Complimentary_Filter.Euler_Angle_Deg[2],
            vbat);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tx_buf, len);
}

static void UART_ParseByte_ISR(uint8_t b) {
    if (b == '\r') return;
    if (b == '\n') {
        cmd_work[cmd_len] = 0;
        if (!line_ready) {
            strncpy(cmd_ready, cmd_work, CMD_LINE_SIZE);
            cmd_ready[CMD_LINE_SIZE-1] = 0;
            line_ready = 1;
        }
        cmd_len = 0;
        return;
    }
    if (b >= 32 && b <= 126) {
        if (cmd_len < (CMD_LINE_SIZE - 1)) cmd_work[cmd_len++] = (char)b;
        else cmd_len = 0;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        for (uint16_t i = 0; i < Size; i++) UART_ParseByte_ISR(rx_dma_buf[i]);
        UART1_StartRxToIdle_DMA();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        HAL_UART_DMAStop(&huart1);
        volatile uint32_t temp = huart->Instance->SR;
        temp = huart->Instance->DR;
        (void)temp;
        UART1_StartRxToIdle_DMA();
    }
}
