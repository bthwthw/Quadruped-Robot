#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "GaitPlanner.hpp"
#include "LegController.hpp"

using namespace webots;

int main(int argc, char **argv) {
    Robot *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();

    const char* motorNames[8] = {"motor_FL_hip", "motor_FL_knee", "motor_FR_hip", "motor_FR_knee", "motor_BR_hip", "motor_BR_knee", "motor_BL_hip", "motor_BL_knee"};
    const char* sensorNames[8] = {"sensor_FL_hip", "sensor_FL_knee", "sensor_FR_hip", "sensor_FR_knee", "sensor_BR_hip", "sensor_BR_knee", "sensor_BL_hip", "sensor_BL_knee"};
    
    Motor* motors[8];
    PositionSensor* sensors[8];
    for (int i = 0; i < 8; i++) {
        motors[i] = robot->getMotor(motorNames[i]);
        sensors[i] = robot->getPositionSensor(sensorNames[i]); 
        sensors[i]->enable(timeStep);
    }

    // --- KHỞI TẠO CÁC MODULE ---
    GaitPlanner planner;
    LegController legs[4]; // 4 bộ điều khiển SMC riêng biệt cho 4 chân

    double q_ref[8] = {0}, dq_ref[8] = {0}, q[8] = {0}, dq[8] = {0}, q_prev[8] = {0}, q_ref_prev[8] = {0};
    double dt = timeStep / 1000.0;
    double alpha_filter = 0.15;
    bool first_step = true;

    planner.get_Trot(0.0, q_ref);
    double start_time = robot->getTime();

    while (robot->step(timeStep) != -1) {
        double t_elapsed = robot->getTime() - start_time;

        for (int i=0; i<8; i++) q_ref_prev[i] = q_ref[i];
        
        // 1. TÍNH TOÁN QUỸ ĐẠO BẰNG PLANNER
        if (t_elapsed < 2.0) {
            planner.get_Trot(0.0, q_ref); 
        } else {
            planner.get_Trot(t_elapsed - 2.0, q_ref); 
        }

        // 2. ĐỌC CẢM BIẾN & LỌC NHIỄU
        for (int i = 0; i < 8; i++) {
            dq_ref[i] = (q_ref[i] - q_ref_prev[i]) / dt;
            if (dq_ref[i] > 20.0) dq_ref[i] = 20.0; else if (dq_ref[i] < -20.0) dq_ref[i] = -20.0;

            if (sensors[i] != NULL) q[i] = sensors[i]->getValue();
            if (first_step) q_prev[i] = q[i];
            
            double raw_dq = (q[i] - q_prev[i]) / dt;
            dq[i] = alpha_filter * raw_dq + (1.0 - alpha_filter) * dq[i];
            if (dq[i] > 30.0) dq[i] = 30.0; else if (dq[i] < -30.0) dq[i] = -30.0;
            
            q_prev[i] = q[i];
        }
        first_step = false;

        // 3. ĐIỀU KHIỂN ĐỘNG CƠ
        for (int leg = 0; leg < 4; leg++) {
            int h = leg * 2; int k = leg * 2 + 1;
            
            if (t_elapsed < 0.2) {
                // Đứng dậy bằng Position Control
                if (motors[h] != NULL) motors[h]->setPosition(q_ref[h]);
                if (motors[k] != NULL) motors[k]->setPosition(q_ref[k]);
            } else {
                // Chuyển sang Torque Control
                if (motors[h] != NULL) motors[h]->setPosition(INFINITY);
                if (motors[k] != NULL) motors[k]->setPosition(INFINITY);

                // Gom dữ liệu của chân hiện tại
                double q_r_leg[2] = {q_ref[h], q_ref[k]}, dq_r_leg[2] = {dq_ref[h], dq_ref[k]};
                double q_leg[2] = {q[h], q[k]}, dq_leg[2] = {dq[h], dq[k]};
                double tau_out[2] = {0, 0};

                // GỌI MODULE SMC BÙ TRỌNG LỰC
                legs[leg].compute_SMC(leg, dt, q_r_leg, q_leg, dq_r_leg, dq_leg, tau_out);

                // Xuất lực
                if (motors[h] != NULL) motors[h]->setTorque(tau_out[0]); 
                if (motors[k] != NULL) motors[k]->setTorque(tau_out[1]);
            }
        }
    }
    delete robot;
    return 0;
}