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
    double T = 1.0;            
    double T_swing = 0.5;
    double S = 0.1;            
    double H = 0.05;
    double y0 = -0.25;          
    double t_a = 0.1;
    double L1 = 0.2;           
    double L2 = 0.2; 

    double v = S / (T_swing - t_a);
    double a = v / t_a;

    double t_local = fmod(t_total + offset, T);
    double x = 0.0, y = 0.0, s_t = 0.0;

    if (t_local < T_swing) {
        if (t_local <= t_a) s_t = 0.5 * a * pow(t_local, 2);
        else if (t_local <= T_swing - t_a) s_t = 0.5 * v * t_a + v * (t_local - t_a);
        else s_t = S - 0.5 * a * pow(T_swing - t_local, 2);

        x = (S / 2.0) * cos(M_PI * (1.0 - s_t / S));
        y = y0 + H * sin(M_PI * (1.0 - s_t / S));
    } else {
        double t_stance = t_local - T_swing;
        if (t_stance <= t_a) s_t = 0.5 * a * pow(t_stance, 2);
        else if (t_stance <= T_swing - t_a) s_t = 0.5 * v * t_a + v * (t_stance - t_a);
        else s_t = S - 0.5 * a * pow(T_swing - t_stance, 2);

        x = (S / 2.0) - s_t;
        y = y0;
    }

    double L_square = x * x + y * y;
    double L_length = sqrt(L_square);
    
    if (L_length >= (L1 + L2)) {
        double scale = (L1 + L2 - 0.001) / L_length; 
        x *= scale;
        y *= scale;
        L_square = x * x + y * y; 
    }

    double D = (L_square - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (D > 1.0) D = 1.0; if (D < -1.0) D = -1.0;
    
    double q2 = knee_dir * acos(D);
    double alpha = atan2(y, x); 
    double beta = atan2(L2 * sin(q2), L1 + L2 * cos(q2));
    double q1 = alpha - beta;

    q_ref_pair[0] = q1; 
    q_ref_pair[1] = q2; 
}

// --- HÀM TẠO QUỸ ĐẠO TROT ---
void generate_Trot_Trajectory(double t, double* q_ref) {
    calculate_Leg_IK(t, 0.0, -1.0, &q_ref[0]); // FL
    calculate_Leg_IK(t, 0.5, -1.0, &q_ref[2]); // FR (Lệch 0.5s)
    calculate_Leg_IK(t, 0.0, -1.0, &q_ref[4]); // BR
    calculate_Leg_IK(t, 0.5, -1.0, &q_ref[6]); // BL (Lệch 0.5s)
}

int main(int argc, char **argv) {
    Robot *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();

    const char* motorNames[8] = {
        "motor_FL_hip", "motor_FL_knee", "motor_FR_hip", "motor_FR_knee",
        "motor_BR_hip", "motor_BR_knee", "motor_BL_hip", "motor_BL_knee"
    };
    const char* sensorNames[8] = {
        "sensor_FL_hip", "sensor_FL_knee", "sensor_FR_hip", "sensor_FR_knee",
        "sensor_BR_hip", "sensor_BR_knee", "sensor_BL_hip", "sensor_BL_knee"
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
    double lambda_gain[2] = {30.0, 30.0};
    double alpha_filter = 0.1;

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
        }

        // ĐỌC SENSOR
        for (int i = 0; i < 8; i++) {
            q[i] = sensors[i]->getValue();
            dq[i] = (q[i] - q_prev[i]) / dt;
            q_prev[i] = q[i];
        }

        // ĐIỀU KHIỂN TỪNG CHÂN
        for (int leg = 0; leg < 4; leg++) {
            int h = leg * 2; int k = leg * 2 + 1;
            double q_leg[2] = {q[h], q[k]}, dq_leg[2] = {dq[h], dq[k]};
            double q_r_l[2] = {q_ref[h], q_ref[k]}, dq_r_l[2] = {dq_ref[h], dq_ref[k]};
            double ddq_r_l[2] = {0, 0}, tau_leg[2] = {0, 0};
            
            double K_gain[2] = {(leg < 2) ? 80.0 : 40.0, (leg < 2) ? 80.0 : 40.0};
            
            SMC_controller(q_r_l, dq_r_l, ddq_r_l, q_leg, dq_leg, K_gain, lambda_gain, tau_leg);
            
            motors[h]->setTorque(tau_leg[0]);
            motors[k]->setTorque(tau_leg[1]);
        }
    }
    delete robot;
    return 0;
}