#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <iostream> 
#include "GaitPlanner.hpp"
#include "LegController.hpp"   // model-free
#include "LegController2.hpp"  // model-based

using namespace webots;

int main(int argc, char **argv) {
    Robot *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();
    
    Keyboard *keyboard = robot->getKeyboard();
    keyboard->enable(timeStep);

    const char* motorNames[8] = {"motor_FL_hip", "motor_FL_knee", "motor_FR_hip", "motor_FR_knee", "motor_BR_hip", "motor_BR_knee", "motor_BL_hip", "motor_BL_knee"};
    const char* sensorNames[8] = {"sensor_FL_hip", "sensor_FL_knee", "sensor_FR_hip", "sensor_FR_knee", "sensor_BR_hip", "sensor_BR_knee", "sensor_BL_hip", "sensor_BL_knee"};
    
    Motor* motors[8];
    PositionSensor* sensors[8];
    for (int i = 0; i < 8; i++) {
        motors[i] = robot->getMotor(motorNames[i]);
        sensors[i] = robot->getPositionSensor(sensorNames[i]); 
        sensors[i]->enable(timeStep);
    }

    GaitPlanner planner;
    LegController legs_free[4];   
    LegController2 legs_based[4]; 

    double q_ref[8] = {0}, dq_ref[8] = {0}, q[8] = {0}, dq[8] = {0}, q_prev[8] = {0}, q_ref_prev[8] = {0};
    double dt = timeStep / 1000.0;
    double alpha_filter = 0.15;
    bool first_step = true;
    
    char current_gait = 'T';
    char prev_gait = 'T';      
    int current_controller = 2; 
    int prev_controller = 2;    
    
    double internal_gait_time = 0.0;
    double speed_factor = 1.0;   
    int direction = 1;            
    int prev_display_speed = 100;
    int prev_direction = 1;
    int prev_key = -1;

    planner.get_Trot(0.0, q_ref);
    double start_time = robot->getTime();

    std::cout << "RUNNING: Model-based SMC and Trot gait" << std::endl;
    std::cout << "===========================================" << std::endl;
    std::cout << "CHANGE GAIT: key T (Trot), B (Bound), P (Pace), W (Walk)" << std::endl;
    std::cout << "CHANGE SMC Controller: key 1 (Model-free), 2 (Model-based)" << std::endl;
    std::cout << "Speed: Key Up (Increase), Down (Decrease)" << std::endl;
    std::cout << "Direction: Key Right (Forward), Left (Backward)" << std::endl;

    while (robot->step(timeStep) != -1) {
        double t_elapsed = robot->getTime() - start_time;
        
        int key = keyboard->getKey();
        if (key == 'T' || key == 't') current_gait = 'T';
        else if (key == 'B' || key == 'b') current_gait = 'B';
        else if (key == 'P' || key == 'p') current_gait = 'P';
        else if (key == 'W' || key == 'w') current_gait = 'W';
        
        else if (key == '1') current_controller = 1; 
        else if (key == '2') current_controller = 2; 
        
        else if (key == Keyboard::UP && key != prev_key) {
          speed_factor += 0.1; 
          if (speed_factor > 3.0) speed_factor = 3.0; // Max speed 300%
        }
        else if (key == Keyboard::DOWN && key != prev_key) {
          speed_factor -= 0.1;
          if (speed_factor < 0.2) speed_factor = 0.2; // Min speed 20%
        }
        else if (key == Keyboard::RIGHT && key != prev_key) direction = 1;
        else if (key == Keyboard::LEFT && key != prev_key) direction = -1;
        prev_key = key;
        
        // =========================================================

        // Log 
        if (current_controller != prev_controller) {
            if (current_controller == 1) {
                std::cout << ">> [Controller] MODEL-FREE SMC"<< std::endl;
            } else {
                std::cout << ">> [Controller] MODEL-BASED SMC" << std::endl;
            }
            prev_controller = current_controller; 
        }

        if (current_gait != prev_gait) {
            std::cout << ">> [Gait] ";
            switch (current_gait) {
                case 'T': std::cout << "TROT" << std::endl; break;
                case 'B': std::cout << "BOUND" << std::endl; break;
                case 'P': std::cout << "PACE" << std::endl; break;
                case 'W': std::cout << "WALK" << std::endl; break;
            }
            prev_gait = current_gait;
        }
        
        int current_display_speed = (int)(speed_factor * 100 + 0.5);
        if (current_display_speed != prev_display_speed) {
            std::cout << ">> [Speed] " << current_display_speed << "%" << std::endl;
            prev_display_speed = current_display_speed;
        }
        
        if (direction != prev_direction) {
            if (direction == 1) std::cout << ">> [Direction] Forward >>>" << std::endl;
            else std::cout << ">> [Direction] <<< Backward" << std::endl;
            prev_direction = direction;
        }
        // =========================================================

        // gait planner 
        for (int i=0; i<8; i++) q_ref_prev[i] = q_ref[i];     
        
        if (t_elapsed >= 1.5) { 
            internal_gait_time += dt * speed_factor * direction; 
        }
        
        switch (current_gait) {
            case 'T': planner.get_Trot(internal_gait_time, q_ref); break;
            case 'B': planner.get_Bound(internal_gait_time, q_ref); break;
            case 'P': planner.get_Pace(internal_gait_time, q_ref); break;
            case 'W': planner.get_Walk(internal_gait_time, q_ref); break;
            default:  planner.get_Trot(internal_gait_time, q_ref); break;
        }
        // =========================================================

        // read sensor 
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
        // =========================================================

        // motor control 
        for (int leg = 0; leg < 4; leg++) {
            int h = leg * 2; int k = leg * 2 + 1;
            
            if (t_elapsed < 1.5) { 
                if (motors[h] != NULL) motors[h]->setPosition(q_ref[h]);
                if (motors[k] != NULL) motors[k]->setPosition(q_ref[k]);
            } else {
                if (motors[h] != NULL) motors[h]->setPosition(INFINITY);
                if (motors[k] != NULL) motors[k]->setPosition(INFINITY);

                double q_r_leg[2] = {q_ref[h], q_ref[k]}, dq_r_leg[2] = {dq_ref[h], dq_ref[k]}, ddq_r_leg[2] = {0, 0};
                double q_leg[2] = {q[h], q[k]}, dq_leg[2] = {dq[h], dq[k]};
                double tau_out[2] = {0, 0};

                if (current_controller == 1) {
                    legs_free[leg].compute_SMC(leg, dt, q_r_leg, q_leg, dq_r_leg, dq_leg, tau_out);
                } else {
                    legs_based[leg].compute_SMC(leg, dt, q_r_leg, q_leg, dq_r_leg, dq_leg, ddq_r_leg, tau_out);
                }

                if (motors[h] != NULL) motors[h]->setTorque(tau_out[0]); 
                if (motors[k] != NULL) motors[k]->setTorque(tau_out[1]);
            }
        }
    }
    delete robot;
    return 0;
}