# Flight Controller Project Overview

## 1. Mục đích của project

Đây là firmware flight controller cho drone 4 động cơ chạy trên STM32F411.

Mục tiêu chính của project là:

- Đọc dữ liệu cảm biến quán tính và từ kế.
- Ước lượng tư thế drone theo `roll / pitch / yaw`.
- Tính lệnh điều khiển ổn định bay.
- Trộn lệnh điều khiển thành PWM cho 4 motor.
- Dự phòng khả năng nhận RC input và telemetry / PID tuning qua UART.

Lưu ý: dù hàm điều khiển chính đang được đặt tên là `MPC()`, thuật toán điều khiển hiện tại trong code là `cascade PID` chứ chưa phải model predictive control đúng nghĩa.

## 2. Cấu trúc thư mục hiện tại

Project đã được tách lại theo chức năng để dễ bảo trì:

- `Core/Src/main.c`: điểm vào của firmware, điều phối init và scheduler vòng lặp.
- `Core/Src/platform`: các file generated / phần cứng nền như GPIO, TIM, I2C, USART, DMA, interrupt.
- `Core/Src/sensor`: xử lý sensor, calibration, filter, attitude estimation.
- `Core/Src/control`: bộ điều khiển, PID, mixer motor.
- `Core/Src/input`: đọc RC input.
- `Core/Src/comm`: telemetry và PID tuning qua UART.

Các header tương ứng nằm trong `Core/Inc/...` theo cùng cấu trúc.

## 3. Vai trò từng khối chính

### `main`

`main.c` làm 3 việc chính:

1. Khởi tạo toàn bộ peripheral và module.
2. Chạy scheduler theo chu kỳ.
3. Gọi các module chức năng, không chứa quá nhiều xử lý chuyên biệt.

### `sensor`

Khối sensor gồm:

- `imu_config.c`: khởi tạo và đọc `MPU6050` và `HMC5883L`.
- `complementary_filter.c`: hợp nhất gyro + accel + mag để ra góc Euler.
- `mag_calibration.c`: hiệu chuẩn magnetometer.

Đầu ra quan trọng nhất của khối này là:

- `MPU6500_DATA.w[]`: tốc độ góc.
- `MPU6500_DATA.acc[]`: gia tốc đã lọc.
- `Complimentary_Filter.Euler_Angle_Deg[]`: góc `roll / pitch / yaw`.

Lưu ý: tên biến còn giữ `MPU6500_*` nhưng driver đang làm việc với `MPU6050`.

### `control`

Khối điều khiển gồm:

- `pid.c`: PID cho vòng angle và vòng rate.
- `flight_control.c`: logic bay chính, mode điều khiển, mapping stick, cascade PID.
- `motor_control.c`: mixer và xuất PWM cho 4 motor.

### `input`

`rc_input.c` chứa callback đọc độ rộng xung PWM từ RC receiver bằng `Input Capture`.

Khối này đã có logic đọc kênh `roll / pitch / throttle / yaw / arm / mode`, nhưng hiện chưa được nối hoàn chỉnh vào cấu hình timer đang dùng trong project.

### `comm`

`telemetry.c` hỗ trợ:

- nhận lệnh text qua UART để tune PID,
- gửi telemetry dạng JSON qua `USART1`.

Hiện tại phần này đang tồn tại trong code nhưng chưa bật trong vòng lặp chính.

## 4. Phần cứng và peripheral đang dùng

Từ code hiện tại có thể rút ra sơ đồ sử dụng peripheral như sau:

- `TIM2`: timer nền để đo thời gian loop và tạo delay microsecond.
- `TIM3` + `TIM4`: PWM điều khiển 4 motor.
- `I2C1`: giao tiếp cảm biến `MPU6050` và `HMC5883L`.
- `USART1`: telemetry / command channel.

Mapping đáng chú ý:

- `I2C1`: `PB8 / PB9`
- Motor PWM:
  - `TIM3_CH1` -> `PB4`
  - `TIM3_CH2` -> `PB5`
  - `TIM4_CH1` -> `PB6`
  - `TIM4_CH2` -> `PB7`
- `USART1`: `PA9 / PA10`

Ngoài ra code RC đang giả định dùng:

- `TIM5` cho 4 kênh chính trên `PA0 / PA1 / PA2 / PA3`
- `TIM1` cho `SW_Arm` và `SW_Mode`

Nhưng các timer input capture này hiện chưa được khởi tạo trong nhánh `platform/tim.c`.

## 5. Workflow khi khởi động

Khi MCU boot, firmware đi qua luồng sau:

1. `HAL_Init()` và `SystemClock_Config()`.
2. Khởi tạo peripheral nền:
   - `GPIO`
   - `DMA`
   - `TIM2`
   - `TIM3`
   - `TIM4`
   - `I2C1`
   - `USART1`
3. Start `TIM2` để dùng làm nguồn thời gian microsecond.
4. Blink LED báo sống.
5. Start PWM cho 4 kênh motor.
6. Set PWM motor ban đầu về `1000`.
7. Khởi tạo sensor:
   - `MPU6050_Init()`
   - `HMC5883L_Init()`
   - `MPU6050_Calibrate()`
8. Reset toàn bộ PID.
9. Đưa hệ thống về trạng thái an toàn:
   - `enable_motor = 0`
   - `ARM_Status = NOT_ARM`
   - `Throttle = 1000`
10. Gán giá trị RC giả lập vì RC thật đang bị tắt trong `main.c`.

## 6. Workflow runtime trong vòng lặp chính

Vòng lặp chính được thiết kế theo kiểu scheduler nhiều tầng, với lõi chạy ở khoảng `1 kHz`.

### 6.1. Đo chu kỳ loop

Mỗi vòng:

- đọc `TIM2->CNT`
- tính `dt`
- kẹp `dt` nếu quá lớn
- lưu `max_dt` để theo dõi loop time

Việc này cho phép khối filter và điều khiển dùng `dt` thực thay vì giả định cứng.

### 6.2. Scheduler theo tầng

Project đang chia xử lý thành 3 lớp:

- `Fast loop ~ 1 kHz`
- `Medium loop ~ 100 Hz`
- `Slow loop ~ 20 Hz`

### 6.3. Fast loop

Luồng fast loop hiện tại:

1. `IMU_PROCESS()`
   - đọc raw accel/gyro từ `MPU6050`
   - trừ bias sau calibration
   - đổi đơn vị vật lý
   - low-pass filter gyro và accel
   - cập nhật `MPU6500_DATA`
2. cập nhật `MPU6500_DATA.dt`
3. `Complimentary_Filter_Predict()`
   - dùng gyro để predict attitude
   - dùng accel để hiệu chỉnh `roll / pitch`
4. `MPC()`
   - đọc setpoint từ RC
   - chạy control loop
   - trộn motor
   - xuất PWM
5. chờ đủ `1000 us` để giữ nhịp loop

### 6.4. Medium loop

Mỗi 10 vòng fast loop:

1. `COMPASS_PROCESS()`
   - đọc `HMC5883L`
   - scale sang đơn vị từ trường
   - hiệu chuẩn bằng `MagCal_Update()`
   - low-pass filter magnetometer
2. nếu magnetometer đã sẵn sàng:
   - `Complimentary_Filter_Update()`
   - sửa `yaw` bằng dữ liệu từ kế

### 6.5. Slow loop

Mỗi 50 vòng:

- đây là chỗ dành cho telemetry, battery check, failsafe hoặc task tốc độ thấp.

Trong code hiện tại, phần telemetry ở nhánh này vẫn đang comment.

## 7. Data flow từ sensor tới motor

Luồng dữ liệu tổng quát của hệ thống như sau:

`MPU6050 / HMC5883L`
-> `imu_config.c`
-> `complementary_filter.c`
-> `Complimentary_Filter.Euler_Angle_Deg[]` và `MPU6500_DATA.w[]`
-> `flight_control.c`
-> `Moment[3]`
-> `motor_control.c`
-> `TIM3/TIM4 CCRx`
-> `ESC / motor`

Nếu nhìn theo tín hiệu:

- `acc + gyro` chủ yếu tạo `roll / pitch` và tốc độ góc.
- `mag` chủ yếu hiệu chỉnh `yaw`.
- `RC input` tạo setpoint mong muốn.
- `PID` biến sai số thành moment điều khiển.
- `mixer` biến moment + throttle thành 4 giá trị PWM motor.

## 8. Logic điều khiển hiện tại

Project có 2 mode điều khiển:

### `HOVER`

Đây là mode angle control:

1. Stick RC được đổi thành `angle_desired`.
2. Vòng PID ngoài tính `angle_rate_desired`.
3. Vòng PID trong bám tốc độ góc thực tế.
4. Output cuối cùng là `Moment[roll, pitch, yaw]`.

### `RATE_MODE`

Đây là mode rate control:

1. Stick RC đi thẳng thành `angle_rate_desired`.
2. PID rate bám trực tiếp tốc độ góc.
3. Output là moment điều khiển.

### Đặc điểm thực tế

- Vòng trong có `D-on-measurement`.
- Có `feed-forward` kiểu iNav cho rate loop.
- Mixer hiện dành cho quad 4 motor cấu hình chuẩn.

## 9. Motor mixing và output

`motor_control.c` thực hiện:

1. nhận `throttle` và `moment[3]`
2. tính 4 giá trị motor
3. clamp về dải an toàn
4. ghi ra:
   - `TIM3->CCR1`
   - `TIM3->CCR2`
   - `TIM4->CCR1`
   - `TIM4->CCR2`

Nếu `enable_motor == 0`, toàn bộ motor bị kéo về `1000` để đảm bảo safe state.

## 10. Telemetry và tuning

Khối `comm/telemetry.c` hỗ trợ:

- nhận command text để đổi PID,
- trả về telemetry góc và điện áp dạng JSON,
- dùng `USART1` với DMA RX.

Ví dụ ý tưởng command:

- `PID:ANG:kp:ki:kd`
- `PID:YANG:kp:ki:kd`
- `PID:YAW:kp:ki:kd`

Tuy nhiên ở code hiện tại:

- `UART1_StartRxToIdle_DMA()` đang chưa được gọi trong `main.c`
- khối parse command đang bị comment trong loop
- `Send_Telemetry()` cũng đang chưa được bật lại

Nói cách khác, telemetry framework đã có nhưng chưa được activate trong runtime path chính.

## 11. Trạng thái hiện tại của project

Theo code hiện tại, project đang ở trạng thái:

- build được,
- đã có sensor pipeline,
- đã có filter và control pipeline,
- đã có motor output pipeline,
- nhưng vẫn đang để chế độ an toàn / chưa bay thật bằng RC.

Các điểm quan trọng:

- RC input start trong `main.c` đang comment.
- `main.c` đang gán giá trị RC cố định.
- `ARM_Status` được đặt `NOT_ARM` sau init.
- `enable_motor` được đặt `0`.
- logic arming / mode switching trong `flight_control.c` hiện vẫn đang comment.

Kết quả là:

- code vẫn chạy sensor và control loop,
- nhưng motor sẽ luôn về mức an toàn thay vì nhận lệnh bay thực.

## 12. Những điểm cần nhớ khi bảo trì project

- `main.c` nên chỉ giữ vai trò điều phối, không nhét thêm xử lý thuật toán vào đây.
- Mọi logic sensor mới nên để trong `sensor/`.
- Mọi logic control mới nên để trong `control/`.
- Các file trong `platform/` là phần gần CubeMX/generated, nên sửa thận trọng để tránh xung đột khi regenerate.
- Nếu bật lại RC thật, cần đồng bộ giữa:
  - `input/rc_input.c`
  - `platform/tim.c`
  - mapping chân trong `.ioc`
- Nếu bật telemetry thật, cần bật lại:
  - `UART1_StartRxToIdle_DMA()`
  - parse command trong loop
  - `Send_Telemetry()`

## 13. Tóm tắt ngắn

Đây là một firmware flight controller STM32F411 cho quadcopter, với pipeline khá rõ:

`sensor -> filter -> attitude -> cascade PID -> mixer -> motor PWM`

Codebase hiện đã có cấu trúc đủ tốt để phát triển tiếp, nhưng trạng thái runtime hiện tại vẫn đang thiên về bench test / safe test hơn là chế độ bay thật bằng RC receiver.
