# DT FlightController

Firmware flight controller cho drone 4 động cơ chạy trên `STM32F411`.

Project này tập trung vào pipeline điều khiển bay cơ bản:

`sensor -> filter -> attitude -> cascade PID -> mixer -> motor PWM`

Lưu ý: dù hàm điều khiển chính trong code đang tên là `MPC()`, thuật toán hiện tại thực tế là `cascade PID`, chưa phải model predictive control đúng nghĩa.

## Chức năng chính

- Đọc dữ liệu IMU từ `MPU6050`
- Đọc dữ liệu từ kế từ `HMC5883L`
- Ước lượng tư thế `roll / pitch / yaw` bằng complementary filter
- Điều khiển theo 2 mode:
  - `HOVER`: angle control
  - `RATE_MODE`: rate control
- Trộn moment điều khiển thành PWM cho 4 motor
- Hỗ trợ sẵn khung RC input và telemetry / PID tuning qua UART

## Kiến trúc code

Project đã được tổ chức lại theo module để dễ bảo trì:

- `Core/Src/main.c`
  Entry point của firmware, phụ trách init và scheduler.
- `Core/Src/platform`
  Các file gần với phần cứng và CubeMX như GPIO, TIM, I2C, USART, DMA, interrupt.
- `Core/Src/sensor`
  Đọc sensor, calibration, filter, attitude estimation.
- `Core/Src/control`
  PID, logic điều khiển bay, motor mixer.
- `Core/Src/input`
  RC input bằng timer input capture.
- `Core/Src/comm`
  Telemetry và command channel qua UART.

## Phần cứng / peripheral đang dùng

- `TIM2`
  Timer nền để đo loop time và tạo delay microsecond.
- `TIM3` + `TIM4`
  PWM điều khiển 4 motor.
- `I2C1`
  Giao tiếp với `MPU6050` và `HMC5883L`.
- `USART1`
  Telemetry / command channel.

Mapping nổi bật:

- `I2C1`: `PB8 / PB9`
- `TIM3_CH1 / CH2`: `PB4 / PB5`
- `TIM4_CH1 / CH2`: `PB6 / PB7`
- `USART1`: `PA9 / PA10`

## Workflow khi khởi động

Khi MCU boot, firmware đi qua luồng sau:

1. `HAL_Init()` và `SystemClock_Config()`
2. Init các peripheral nền:
   - `GPIO`
   - `DMA`
   - `TIM2`
   - `TIM3`
   - `TIM4`
   - `I2C1`
   - `USART1`
3. Start `TIM2` để dùng làm timebase microsecond
4. Blink LED báo sống
5. Start PWM cho 4 motor
6. Khởi tạo sensor:
   - `MPU6050_Init()`
   - `HMC5883L_Init()`
   - `MPU6050_Calibrate()`
7. Reset PID và đưa hệ thống về safe state

## Workflow runtime

Vòng lặp chính dùng scheduler nhiều tầng, lõi chạy khoảng `1 kHz`.

### Fast loop ~ 1 kHz

1. `IMU_PROCESS()`
   - đọc accel/gyro từ `MPU6050`
   - trừ bias
   - đổi sang đơn vị vật lý
   - low-pass filter
2. cập nhật `MPU6500_DATA.dt`
3. `Complimentary_Filter_Predict()`
   - predict attitude từ gyro
   - hiệu chỉnh `roll / pitch` bằng accel
4. `MPC()`
   - đọc setpoint
   - chạy control loop
   - trộn motor
   - xuất PWM
5. pacing loop để giữ nhịp `1000 us`

### Medium loop ~ 100 Hz

1. `COMPASS_PROCESS()`
   - đọc `HMC5883L`
   - scale và hiệu chuẩn từ kế
   - low-pass filter dữ liệu mag
2. `Complimentary_Filter_Update()`
   - hiệu chỉnh `yaw` bằng magnetometer

### Slow loop ~ 20 Hz

Dành cho:

- telemetry
- battery check
- failsafe
- các task chậm khác

Hiện tại nhánh này mới ở mức khung, chưa bật đầy đủ trong runtime path chính.

## Data flow tổng quát

`MPU6050 / HMC5883L`
-> `sensor/imu_config.c`
-> `sensor/complementary_filter.c`
-> `Complimentary_Filter.Euler_Angle_Deg[]` và `MPU6500_DATA.w[]`
-> `control/flight_control.c`
-> `Moment[3]`
-> `control/motor_control.c`
-> `TIM3 / TIM4 CCRx`
-> `ESC / motor`

## Logic điều khiển

### `HOVER`

Mode tự cân bằng theo góc:

1. RC stick tạo `angle_desired`
2. PID vòng ngoài tạo `angle_rate_desired`
3. PID vòng trong bám tốc độ góc thực
4. output cuối là moment điều khiển

### `RATE_MODE`

Mode bám tốc độ góc:

1. RC stick đi thẳng vào `angle_rate_desired`
2. PID rate bám trực tiếp tốc độ góc
3. output là moment điều khiển

## Trạng thái hiện tại của project

Theo code hiện tại, project:

- build được ổn định
- đã có pipeline sensor, filter, control, motor output
- nhưng vẫn đang ở trạng thái an toàn / bench-test nhiều hơn là bay thật bằng RC

Các điểm quan trọng:

- RC input start trong `main.c` đang comment
- `main.c` đang gán RC value giả lập
- `ARM_Status = NOT_ARM`
- `enable_motor = 0`
- logic arming / mode switching trong `flight_control.c` vẫn đang comment
- telemetry framework đã có nhưng chưa bật hoàn chỉnh

Vì vậy hiện tại motor vẫn bị giữ ở safe state thay vì nhận lệnh bay thực.

## Ghi chú bảo trì

- Giữ `main.c` làm file điều phối, không nhét nhiều thuật toán trực tiếp vào đây
- Logic sensor mới nên đặt trong `sensor/`
- Logic control mới nên đặt trong `control/`
- Các file trong `platform/` là phần gần CubeMX/generated, nên chỉnh sửa cẩn thận
- Nếu bật lại RC thật, cần đồng bộ:
  - `input/rc_input.c`
  - `platform/tim.c`
  - pin mapping trong `.ioc`
- Nếu bật telemetry thật, cần bật lại:
  - `UART1_StartRxToIdle_DMA()`
  - parse command trong main loop
  - `Send_Telemetry()`

## Tài liệu thêm

File tổng quan chi tiết trước đây hiện được chuyển về tài liệu gốc này.
