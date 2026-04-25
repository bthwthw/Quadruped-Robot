// File:          main_SMC_controller.cpp
// Date:
// Description:   Kiểm tra tư thế đứng tĩnh (Squat) cấu hình >>
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <cmath>

extern "C" {
    #include "SMC_controller.h"
    #include "SMC_controller_initialize.h"
}

using namespace webots;

// --- HÀM TÍNH ĐỘNG HỌC NGHỊCH ---
void calculate_Leg_IK(double t_total, double offset, double knee_dir, double* q_ref_pair) {
    double S = 0.0; // Sải bước = 0 vì chỉ đứng im            
    double H = 0.0; // Độ cao nhấc chân = 0
    
    // Đã biết L1 = 0.2, L2 = 0.2. Tổng dài 0.4.
    double y0 = -0.25;          
    double L1 = 0.2;           
    double L2 = 0.2; 

    double x = 0.0;
    double y = y0; // Luôn luôn ở độ cao squat

    double L_square = x * x + y * y;
    double L_length = sqrt(L_square);
    
    if (L_length >= (L1 + L2)) {
        double scale = (L1 + L2 - 0.001) / L_length; 
        x *= scale;
        y *= scale;
        L_square = x * x + y * y; 
    }

    double D = (L_square - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (D > 1.0) D = 1.0; 
    if (D < -1.0) D = -1.0;
    
    double q2 = knee_dir * acos(D);
    double alpha = atan2(y, x); 
    double beta = atan2(L2 * sin(q2), L1 + L2 * cos(q2));
    double q1 = alpha - beta;

    q_ref_pair[0] = q1; 
    q_ref_pair[1] = q2; 
}

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  const char* motorNames[8] = {
      "motor_FL_hip", "motor_FL_knee", 
      "motor_FR_hip", "motor_FR_knee", 
      "motor_BR_hip", "motor_BR_knee",  
      "motor_BL_hip", "motor_BL_knee" 
  };
  const char* sensorNames[8] = {
      "sensor_FL_hip", "sensor_FL_knee", 
      "sensor_FR_hip", "sensor_FR_knee",
      "sensor_BR_hip", "sensor_BR_knee",  
      "sensor_BL_hip", "sensor_BL_knee"
  };
  
  Motor* motors[8];
  PositionSensor* sensors[8];
  
  for (int i = 0; i < 8; i++) {
      motors[i] = robot->getMotor(motorNames[i]);
      sensors[i] = robot->getPositionSensor(sensorNames[i]); 
      sensors[i]->enable(timeStep);
  }
  
  SMC_controller_initialize();
  
  double q_ref[8] = {0}; 
  double dq_ref[8] = {0};
  double ddq_ref[8] = {0};
  double q[8] = {0};
  double dq[8] = {0};
  double q_prev[8] = {0};
  double q_ref_prev[8] = {0};
  
  double dt = timeStep / 1000.0;
  double lambda_gain[2] = {30.0, 30.0};  

  // Main loop
  while (robot->step(timeStep) != -1) {
    double t = robot->getTime();
    
    // BƯỚC 1: LƯU LẠI QUỸ ĐẠO CŨ 
    for (int i=0; i<8; i++) {
        q_ref_prev[i] = q_ref[i];
    }

    // BƯỚC 2: TÍNH TOÁN VỊ TRÍ MỤC TIÊU (SQUAT MÃI MÃI)
    double q_target[8] = {0};
    
    // Dùng -1.0 cho toàn bộ (Cấu hình >>)
    calculate_Leg_IK(0.0, 0.0, -1.0, &q_target[0]); 
    calculate_Leg_IK(0.0, 0.0, -1.0, &q_target[2]); 
    calculate_Leg_IK(0.0, 0.0, -1.0, &q_target[4]); 
    calculate_Leg_IK(0.0, 0.0, -1.0, &q_target[6]); 
    
    double ratio = t / 2.0; 
    if (ratio > 1.0) ratio = 1.0; // BÍ QUYẾT: Khóa chặt tỷ lệ ở mức 100% sau 2 giây!
    
    for(int i = 0; i < 8; i++) {
        q_ref[i] = q_target[i] * ratio; 
    }
    
    // BƯỚC 3: TÍNH TOÁN dq_ref (CÓ LỌC EMA)
    double alpha_filter = 0.1; 
    for (int i = 0; i < 8; i++) {
        double raw_dq_ref = (q_ref[i] - q_ref_prev[i]) / dt; 
        
        if (raw_dq_ref > 20.0) raw_dq_ref = 20.0;
        if (raw_dq_ref < -20.0) raw_dq_ref = -20.0;
        
        dq_ref[i] = alpha_filter * raw_dq_ref + (1.0 - alpha_filter) * dq_ref[i]; 
        ddq_ref[i] = 0.0; 
    }
    
    // BƯỚC 4: ĐỌC SENSOR
    for (int i = 0; i < 8; i++) {
          q[i] = sensors[i]->getValue();
          dq[i] = (q[i] - q_prev[i]) / dt;
          q_prev[i] = q[i];
      }

    // BƯỚC 5: XUẤT LỰC QUA SMC    
    for (int leg = 0; leg < 4; leg++) {
          int hip_idx = leg * 2;
          int knee_idx = leg * 2 + 1;
          
          double q_leg[2] = {q[hip_idx], q[knee_idx]};
          double dq_leg[2] = {dq[hip_idx], dq[knee_idx]};
          double q_ref_leg[2] = {q_ref[hip_idx], q_ref[knee_idx]};
          double dq_ref_leg[2] = {dq_ref[hip_idx], dq_ref[knee_idx]};
          double ddq_ref_leg[2] = {ddq_ref[hip_idx], ddq_ref[knee_idx]};
          double tau_leg[2] = {0.0, 0.0};
          
          double K_gain_dynamic[2];
          if (leg == 0 || leg == 1) { 
              K_gain_dynamic[0] = 40.0; 
              K_gain_dynamic[1] = 40.0;  
          } else {                    
              K_gain_dynamic[0] = 35.0;  
              K_gain_dynamic[1] = 35.0;  
          }
          
          SMC_controller(q_ref_leg, dq_ref_leg, ddq_ref_leg, q_leg, dq_leg, K_gain_dynamic, lambda_gain, tau_leg);
          
          motors[hip_idx]->setTorque(tau_leg[0]);
          motors[knee_idx]->setTorque(tau_leg[1]);
      }
  };

  delete robot;
  return 0;
}