#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <cmath>

extern "C" {
    #include "SMC_controller.h"
    #include "SMC_controller_initialize.h"
}

using namespace webots;

// --- HÀM TÍNH ĐỘNG HỌC NGHỊCH (QUỸ ĐẠO BÁN ELIP) ---
void calculate_Leg_IK(double t_total, double offset, double knee_dir, double* q_ref_pair, bool is_right_leg) {
    double T = 0.4;            
    double S = 0.15;            
    double H = 0.08; 
    double y0 = -0.28;
            
    double L1 = 0.2;           
    double L2 = 0.2; 

    double phase = fmod(t_total / T + offset, 1.0);
    double x = 0.0, y = 0.0;

    // Lật ngược quỹ đạo X nếu là chân bên phải (FR, BR)
    double direction_X = is_right_leg ? -1.0 : 1.0; 

    if (phase < 0.5) {
        // PHA LĂNG (SWING)
        double theta = phase * 2.0 * M_PI; 
        x = direction_X * -(S / 2.0) * cos(theta); // Thêm direction_X vào X
        y = y0 + H * sin(theta);           
    } else {
        // PHA TỰA (STANCE)
        double theta = (phase - 0.5) * 2.0 * M_PI; 
        x = direction_X * (S / 2.0) * cos(theta);  // Thêm direction_X vào X
        y = y0;                            
    }

    // --- ĐỘNG HỌC NGHỊCH (GIỮ NGUYÊN BẢO VỆ TOÁN HỌC) ---
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
    
    // ĐÂY LÀ CHÌA KHÓA: Tính q1 theo toán học trước
    double q1_math = alpha - beta;

    // BÙ TRỪ TỌA ĐỘ: Cộng thêm 90 độ (PI/2) để khớp với trục Webots
    double q1_webots = q1_math + (M_PI / 2.0); 

    q_ref_pair[0] = q1_webots; // Gán góc đã bù trừ cho Háng
    q_ref_pair[1] = q2;        // Khớp gối giữ nguyên
}

// --- HÀM TẠO QUỸ ĐẠO TROT ---
void generate_Trot_Trajectory(double t, double* q_ref) {
    // Truyền false cho chân Trái, true cho chân Phải
    calculate_Leg_IK(t, 0.0, -1.0, &q_ref[0], false); // FL (Trái)
    calculate_Leg_IK(t, 0.5, -1.0, &q_ref[2], true);  // FR (Phải)
    calculate_Leg_IK(t, 0.0, -1.0, &q_ref[4], true);  // BR (Phải)
    calculate_Leg_IK(t, 0.5, -1.0, &q_ref[6], false); // BL (Trái)
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
    double lambda_gain[2] = {150.0, 150.0};
    double alpha_filter = 0.05;
    bool first_step = true;

    // Khởi tạo vị trí q_ref ban đầu để tránh giật ở vòng lặp đầu
    generate_Trot_Trajectory(0.0, q_ref);

    while (robot->step(timeStep) != -1) {
        double t = robot->getTime();
        
        // CẬP NHẬT QUỸ ĐẠO TROT TRỰC TIẾP
        for (int i=0; i<8; i++) q_ref_prev[i] = q_ref[i];
        generate_Trot_Trajectory(t, q_ref);

        // TÍNH dq_ref VỚI BỘ LỌC
        for (int i = 0; i < 8; i++) {
            double raw_dq_ref = (q_ref[i] - q_ref_prev[i]) / dt; 
            if (raw_dq_ref > 20.0) raw_dq_ref = 20.0;
            if (raw_dq_ref < -20.0) raw_dq_ref = -20.0;
            dq_ref[i] = alpha_filter * raw_dq_ref + (1.0 - alpha_filter) * dq_ref[i]; 
            ddq_ref[i] = 0.0;
        }

        // ĐỌC SENSOR VÀ KHÓA MÕM VẬN TỐC
        for (int i = 0; i < 8; i++) {
            if (sensors[i] != NULL) {
                q[i] = sensors[i]->getValue();
            } else {
                q[i] = 0.0;
            }
            if (first_step) {
                q_prev[i] = q[i];
            }
            
            // Tính vận tốc thực tế
            dq[i] = (q[i] - q_prev[i]) / dt;
            
            // KẸP CHẶT: Chó không bao giờ quất chân nhanh hơn 30 rad/s
            if (dq[i] > 30.0) dq[i] = 30.0;
            if (dq[i] < -30.0) dq[i] = -30.0;
            
            q_prev[i] = q[i];
        }
        first_step = false;

        // ĐIỀU KHIỂN TỪNG CHÂN
        for (int leg = 0; leg < 4; leg++) {
            int h = leg * 2; int k = leg * 2 + 1;
            double q_leg[2] = {q[h], q[k]}, dq_leg[2] = {dq[h], dq[k]};
            double q_r_l[2] = {q_ref[h], q_ref[k]}, dq_r_l[2] = {dq_ref[h], dq_ref[k]};
            double ddq_r_l[2] = {0, 0}, tau_leg[2] = {0, 0};
            
            double K_gain[2] = {80.0, 80.0};
            
            // if (leg == 0 || leg == 1) { 
                // Chân trước gánh ngực nặng -> BƠM MẠNH LÊN 150!
                // K_gain_dynamic[0] = 150.0; // Hip (Háng)
                // K_gain_dynamic[1] = 100.0; // Knee (Gối)
            // } else {                    
                // Chân sau gánh mông nhẹ -> Nhẹ nhàng thôi để không chổng mông
                // K_gain_dynamic[0] = 100.0;  // Hip
                // K_gain_dynamic[1] = 60.0;  // Knee
            // }
            
            SMC_controller(q_r_l, dq_r_l, ddq_r_l, q_leg, dq_leg, K_gain, lambda_gain, tau_leg);
            
            // KHÓA MÕM LỰC: Ép chặt ở mức an toàn vật lý của motor
            if (tau_leg[0] > 45.0) tau_leg[0] = 45.0;
            if (tau_leg[0] < -45.0) tau_leg[0] = -45.0;
            if (tau_leg[1] > 45.0) tau_leg[1] = 45.0;
            if (tau_leg[1] < -45.0) tau_leg[1] = -45.0;
            
            // ĐẢO DẤU LỰC (vẫn giữ nguyên để bù trừ hệ tọa độ)
            if (motors[h] != NULL) motors[h]->setTorque(tau_leg[0]); 
            if (motors[k] != NULL) motors[k]->setTorque(tau_leg[1]);
            
            std::cout << "Time: " << t << " | Leg " << leg << " | Tau_Hip: " << tau_leg[0] << " | Tau_Knee: " << tau_leg[1] << std::endl;
        }
    }
    delete robot;
    return 0;
}