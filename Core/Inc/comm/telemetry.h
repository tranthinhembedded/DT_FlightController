#ifndef CORE_INC_COMM_TELEMETRY_H_
#define CORE_INC_COMM_TELEMETRY_H_

#include "main.h"
#include "platform/usart.h"
#include "control/flight_control.h"
#include "sensor/imu_config.h"

#define RX_DMA_SIZE    256
#define CMD_LINE_SIZE  128

extern float vbat;
extern volatile uint8_t line_ready;
extern char cmd_ready[CMD_LINE_SIZE];
extern uint32_t last_telemetry_time;

void UART1_StartRxToIdle_DMA(void);
void ProcessLine(char *line);
void Send_Telemetry(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* CORE_INC_COMM_TELEMETRY_H_ */
