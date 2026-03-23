/*
 * complementary_filter.c
 *
 * Created on: Dec 14, 2025
 * Author: THINH
 * Fixed: Integration factor, Yaw wrapping, Singularity protection
 */

#include "sensor/complementary_filter.h"
#include <math.h>

static float32_t Angle_Diff(float32_t a, float32_t b) {
    float32_t diff = a - b;
    while (diff > M_PI)  diff -= 2.0f * M_PI;
    while (diff < -M_PI) diff += 2.0f * M_PI;
    return diff;
}
void Complimentary_Filter_Reset(Complimentary_Filter_t * imu){

	for(int i = 0 ; i < 3 ; i ++){
		imu->Euler_Angle_Deg[i] = 0;
		imu->Euler_Angle_Rad[i] = 0;
	}
	imu->q[0] = 1.0f; imu->q[1] = 0.0f; imu->q[2] = 0.0f; imu->q[3] = 0.0f;
	imu->predict_count = 0;
	imu->update_count = 0;
	imu->status = Fusion_START;
	imu->Fusion_OK = 0;
}

void Complimentary_Filter_Predict(Complimentary_Filter_t * imu ,IMU_Data_t * imu_data){
	   // 1. Normalize Accel (Bảo vệ chia 0)
	   float32_t a_square = imu_data->acc[0]*imu_data->acc[0] + imu_data->acc[1]*imu_data->acc[1] + imu_data->acc[2]*imu_data->acc[2];
	   if(a_square < 1e-6f) return; // Accel lỗi hoặc rơi tự do -> không tính toán

	   float32_t a_norm_val = sqrtf(a_square);
	   float32_t ax = imu_data->acc[0] / a_norm_val;
	   float32_t ay = imu_data->acc[1] / a_norm_val;
	   float32_t az = imu_data->acc[2] / a_norm_val;

	   // Gyro rates (rad/s)
	   float32_t p = imu_data->w[0];
	   float32_t q = imu_data->w[1];
	   float32_t r = imu_data->w[2];

	   switch(imu->status){
		  case Fusion_START:
			  // Khởi tạo góc ban đầu từ Accel thuần túy
			  imu->Euler_Angle_Rad[0] = atan2f(-ay, -az); // Roll
			  imu->Euler_Angle_Rad[1] = atan2f(ax, sqrtf(ay*ay + az*az)); // Pitch
			  imu->Euler_Angle_Rad[2] = 0; // Yaw tạm thời bằng 0 (sẽ update sau bởi Mag)

			  // Update Degree variables
			  imu->Euler_Angle_Deg[0] = imu->Euler_Angle_Rad[0] * RAD_TO_DEG;
			  imu->Euler_Angle_Deg[1] = imu->Euler_Angle_Rad[1] * RAD_TO_DEG;
			  imu->Euler_Angle_Deg[2] = 0;

			  imu->status = Fusion_RUN;
			  break;

		  case Fusion_RUN:
			  //Tính góc từ Accel (Measurement)
			  float32_t Phi_Acc = atan2f(-ay, -az);
			  float32_t Theta_Acc = atan2f(ax, sqrtf(ay*ay + az*az));

			  //Gyro Integration (Prediction)
			  float32_t phi = imu->Euler_Angle_Rad[0];
			  float32_t theta = imu->Euler_Angle_Rad[1];

			  // Bảo vệ Gimbal Lock (Khi Pitch ~ 90 độ, cos(theta) -> 0, tan(theta) -> inf)
			  float32_t cos_theta = cosf(theta);
			  float32_t tan_theta = tanf(theta);

			  if (fabsf(cos_theta) < 1e-4f) {
				  // Nếu dựng đứng, bỏ qua phần chuyển đổi Euler rate phức tạp, dùng p,q,r thô
				  cos_theta = 1e-4f;
				  tan_theta = 0.0f;
			  }

			  // Kinematics Equation (Body rate -> Euler rate)
			  float32_t phi_dot   = p + sinf(phi) * tan_theta * q + cosf(phi) * tan_theta * r;
			  float32_t theta_dot = cosf(phi) * q - sinf(phi) * r;
			  float32_t psi_dot   = (sinf(phi) * q + cosf(phi) * r) / cos_theta;

			  // Tích phân Euler (FIX LỖI 0.5 Ở ĐÂY -> NHÂN 1.0)
			  imu->Euler_Angle_Rad[0] += phi_dot * imu_data->dt;
			  imu->Euler_Angle_Rad[1] += theta_dot * imu_data->dt;
			  imu->Euler_Angle_Rad[2] += psi_dot * imu_data->dt;

			  //Complementary Filter (Fusion)
			  // ROLL
			  imu->Euler_Angle_Rad[0] = imu->alpha[0] * imu->Euler_Angle_Rad[0] + (1.0f - imu->alpha[0]) * Phi_Acc;

			  // PITCH
			  imu->Euler_Angle_Rad[1] = imu->alpha[1] * imu->Euler_Angle_Rad[1] + (1.0f - imu->alpha[1]) * Theta_Acc;

			  // YAW (Chỉ tích phân Gyro ở đây, việc sửa bằng Mag nằm ở hàm Update)
			  // Wrap Yaw trong khoảng -PI đến PI
			  if (imu->Euler_Angle_Rad[2] > M_PI) imu->Euler_Angle_Rad[2] -= 2.0f * M_PI;
			  if (imu->Euler_Angle_Rad[2] < -M_PI) imu->Euler_Angle_Rad[2] += 2.0f * M_PI;

			  // Convert sang độ
              for(int i = 0 ; i < 3 ; i ++){
            	  imu->Euler_Angle_Deg[i] = imu->Euler_Angle_Rad[i] * RAD_TO_DEG;
              }

              imu->predict_count++;
              if(imu->predict_count > 100){ // Đợi ổn định tầm 100 mẫu (0.4s) rồi mới cho là OK
            	  imu->Fusion_OK = 1;
              }
			  break;

		  case Fusion_RESET :
			  Complimentary_Filter_Reset(imu);
			  break;
		  case Fusion_STOP:
		                // Khi STOP, không làm gì cả
		                break;

		  		  default:
		  		      break;
		}
}

void Complimentary_Filter_Update(Complimentary_Filter_t * imu ,MAG_DATA_t * mag){
	            float32_t mx = mag->mag_uT[0];
		        float32_t my = mag->mag_uT[1];
		        float32_t mz = mag->mag_uT[2];

		        float32_t mag_norm = sqrtf(mx*mx + my*my + mz*mz);
		        if (mag_norm < 1e-6f) return; // Bảo vệ chia 0

		        mx /= mag_norm;
		        my /= mag_norm;
		        mz /= mag_norm;

		        // Tilt Compensation (Dùng góc Roll/Pitch đã tính ở Predict để bù nghiêng cho la bàn)
		        float32_t phi = imu->Euler_Angle_Rad[0];
		        float32_t theta = imu->Euler_Angle_Rad[1];

		        float32_t cRoll = cosf(phi), sRoll = sinf(phi);
		        float32_t cPitch = cosf(theta), sPitch = sinf(theta);

		        float32_t mx2 = mx * cPitch + my * sRoll * sPitch + mz * cRoll * sPitch;
		        float32_t my2 = my * cRoll - mz * sRoll;

		        // Tính Yaw từ Mag
		        float32_t psi_mag = atan2f(-my2, mx2);

		        if(imu->update_count == 0){
		        	// Lần đầu tiên: Gán trực tiếp
		        	imu->Euler_Angle_Rad[2] = psi_mag;
		        }
		        else{

		        	float32_t yaw_err = Angle_Diff(psi_mag, imu->Euler_Angle_Rad[2]);
		        	imu->Euler_Angle_Rad[2] += (1.0f - imu->alpha[2]) * yaw_err;

		        	// Wrap lại sau khi cộng
					if (imu->Euler_Angle_Rad[2] > M_PI) imu->Euler_Angle_Rad[2] -= 2.0f * M_PI;
					else if (imu->Euler_Angle_Rad[2] < -M_PI) imu->Euler_Angle_Rad[2] += 2.0f * M_PI;
		        }

		        imu->Euler_Angle_Deg[2] = imu->Euler_Angle_Rad[2] * RAD_TO_DEG;
		        imu->update_count++;
}
