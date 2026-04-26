#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <cmath>

extern "C" {
    #include "SMC_controller.h"
    #include "SMC_controller_initialize.h"
}

using namespace webots;

// --- HÀM TÍNH ĐỘNG HỌC NGHỊCH (GIẢI BẰNG HÀM COSIN) ---
void calculate_Leg_IK(double t_total, double offset, double* q_ref_pair) {
    double T = 0.5;       // chu kỳ      
    double S = 0.08;      // sải chân 
    double H = 0.03;      // nhấc chân 
    double y0 = -0.23;          
    double L1 = 0.2;           
    double L2 = 0.2; 

    // Theta
    double theta = (t_total / T + offset) * 2.0 * M_PI;
    double theta_mod = fmod(theta, 2.0 * M_PI);
    if (theta_mod < 0) theta_mod += 2.0 * M_PI;

    double x = 0.0, y = 0.0;

    // Quỹ đạo Elip thuần
    if (theta_mod < M_PI) {
        x = (S / 2.0) * cos(theta_mod); 
        y = y0 + H * sin(theta_mod);           
    } else {
        x = -(S / 2.0) * cos(theta_mod - M_PI);  
        y = y0;                            
    }

    // Khóa Toán Học
    double L_square = x * x + y * y;
    double max_reach = L1 + L2 - 0.001;
    if (L_square > max_reach * max_reach) {
        double scale = max_reach / sqrt(L_square);
        x *= scale; y *= scale;
        L_square = x * x + y * y;
    }

    // ĐỊNH LÝ HÀM COSIN (Chuẩn xác, không bị nhảy góc)
    double cos_q2 = (L_square - L1*L1 - L2*L2) / (2.0 * L1 * L2);
    if (cos_q2 > 1.0) cos_q2 = 1.0;
    if (cos_q2 < -1.0) cos_q2 = -1.0;

    // CHÌA KHÓA: Ép knee_dir ngầm định là âm (-) để khớp gập ra sau
    double q2 = -acos(cos_q2); 

    // Góc háng
    double alpha = atan2(y, x); 
    double beta = atan2(L2 * sin(q2), L1 + L2 * cos(q2));
    double q1_math = alpha - beta;

    // Bù trừ Webots
    q_ref_pair[0] = q1_math + M_PI / 2.0; 
    q_ref_pair[1] = q2;        
}

// --- HÀM TẠO QUỸ ĐẠO TROT ---
void generate_Trot_Trajectory(double t, double* q_ref) {
    calculate_Leg_IK(t, 0.0, &q_ref[0]); // FL
    calculate_Leg_IK(t, 0.5, &q_ref[2]); // FR
    calculate_Leg_IK(t, 0.0, &q_ref[4]); // BR
    calculate_Leg_IK(t, 0.5, &q_ref[6]); // BL
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
    
    double q_ref[8] = {0}, dq_ref[8] = {0}, ddq_ref[8] = {0};
    double q[8] = {0}, dq[8] = {0}, q_prev[8] = {0}, q_ref_prev[8] = {0};
    double dt = timeStep / 1000.0;
    
    double alpha_filter = 0.15;
    bool first_step = true;

    double tau_integral[8] = {0};

    generate_Trot_Trajectory(0.0, q_ref);
    double start_time = robot->getTime();

    while (robot->step(timeStep) != -1) {
        double t = robot->getTime();
        double t_elapsed = t - start_time;

        for (int i=0; i<8; i++) q_ref_prev[i] = q_ref[i];
        
        if (t_elapsed < 2.0) {
            generate_Trot_Trajectory(0.0, q_ref); 
        } else {
            generate_Trot_Trajectory(t - 2.0, q_ref); 
        }

        for (int i = 0; i < 8; i++) {
            dq_ref[i] = (q_ref[i] - q_ref_prev[i]) / dt;
            if (dq_ref[i] > 20.0) dq_ref[i] = 20.0;
            if (dq_ref[i] < -20.0) dq_ref[i] = -20.0;
            ddq_ref[i] = 0.0;
        }

        for (int i = 0; i < 8; i++) {
            if (sensors[i] != NULL) q[i] = sensors[i]->getValue();
            if (first_step) q_prev[i] = q[i];
            
            // Tính đạo hàm thô
            double raw_dq = (q[i] - q_prev[i]) / dt;
            
            // ÁP DỤNG BỘ LỌC (Cực kỳ quan trọng để SMC không bị giật)
            dq[i] = alpha_filter * raw_dq + (1.0 - alpha_filter) * dq[i];
            
            if (dq[i] > 30.0) dq[i] = 30.0;
            if (dq[i] < -30.0) dq[i] = -30.0;
            q_prev[i] = q[i];
        }
        first_step = false;

        for (int leg = 0; leg < 4; leg++) {
            int h = leg * 2; int k = leg * 2 + 1;
            
            if (t_elapsed < 1.0) {
                // POSITION CONTROL ĐỨNG DẬY
                if (motors[h] != NULL) motors[h]->setPosition(q_ref[h]);
                if (motors[k] != NULL) motors[k]->setPosition(q_ref[k]);
            } else {
                // SMC TROT
                if (motors[h] != NULL) motors[h]->setPosition(INFINITY);
                if (motors[k] != NULL) motors[k]->setPosition(INFINITY);

                double q_leg[2] = {q[h], q[k]}, dq_leg[2] = {dq[h], dq[k]};
                double q_r_l[2] = {q_ref[h], q_ref[k]}, dq_r_l[2] = {dq_ref[h], dq_ref[k]};
                double ddq_r_l[2] = {0, 0}, tau_leg[2] = {0, 0};
                
                double K_gain[2];
                double lambda_gain_tuned[2];

                if (t_elapsed < 2.0) {
                    if (leg < 2) { 
                        K_gain[0] = 150.0; K_gain[1] = 80.0;  
                        lambda_gain_tuned[0] = 200.0; lambda_gain_tuned[1] = 200.0;
                    } else {
                        K_gain[0] = 50.0; K_gain[1] = 80.0; 
                        lambda_gain_tuned[0] = 100.0; lambda_gain_tuned[1] = 200.0;
                    }
                } else {
                    if (leg < 2) {
                        K_gain[0] = 100.0; K_gain[1] = 40.0;  
                        lambda_gain_tuned[0] = 150.0; lambda_gain_tuned[1] = 150.0;
                    } else {
                        K_gain[0] = 30.0;  
                        K_gain[1] = 40.0;  
                        lambda_gain_tuned[0] = 60.0;  
                        lambda_gain_tuned[1] = 150.0; 
                    }
                }

                double Phi = 5.0; // Độ dày lớp biên (Boundary Layer) để chống giật (Chattering)
                
                for(int j = 0; j < 2; j++) {
                    // 1. Tính sai số vị trí và vận tốc
                    double e = q_r_l[j] - q_leg[j];
                    double de = dq_r_l[j] - dq_leg[j];
                    
                    // 2. Phương trình mặt trượt (Sliding Surface)
                    double s = de + lambda_gain_tuned[j] * e;
                    
                    // 3. Hàm Saturation mềm (thay cho hàm Sign để gối không bị giật)
                    double sat_s = s / Phi;
                    if (sat_s > 1.0) sat_s = 1.0;
                    if (sat_s < -1.0) sat_s = -1.0;
                    
                    // 4. Xuất lực Torque
                    tau_leg[j] = K_gain[j] * sat_s;
                }
                // TÍCH PHÂN BÙ TRỌNG LỰC
                double Ki_hip = (leg < 2) ? 70.0 : 40.0;   
                double Ki_knee = 15.0;  

                tau_integral[h] += Ki_hip * (q_ref[h] - q[h]) * dt;
                tau_integral[k] += Ki_knee * (q_ref[k] - q[k]) * dt;

                double max_int_hip = (leg < 2) ? 45.0 : 25.0; 
                double max_int_knee = 12.0;

                if(tau_integral[h] > max_int_hip) tau_integral[h] = max_int_hip;
                if(tau_integral[h] < -max_int_hip) tau_integral[h] = -max_int_hip;
                if(tau_integral[k] > max_int_knee) tau_integral[k] = max_int_knee;
                if(tau_integral[k] < -max_int_knee) tau_integral[k] = -max_int_knee;

                tau_leg[0] += tau_integral[h];
                tau_leg[1] += tau_integral[k];

                // GIẢM CHẤN NHÂN TẠO
                double damping_coefficient = 1.5; 
                tau_leg[0] -= damping_coefficient * dq_leg[0];
                tau_leg[1] -= damping_coefficient * dq_leg[1];

                for(int j=0; j<2; j++) {
                    if (tau_leg[j] > 100.0) tau_leg[j] = 100.0;
                    if (tau_leg[j] < -100.0) tau_leg[j] = -100.0;
                }

                if (motors[h] != NULL) motors[h]->setTorque(tau_leg[0]); 
                if (motors[k] != NULL) motors[k]->setTorque(tau_leg[1]);
            }
        }
    }
    delete robot;
    return 0;
}