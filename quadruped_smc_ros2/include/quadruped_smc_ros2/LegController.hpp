#ifndef LEG_CONTROLLER_HPP
#define LEG_CONTROLLER_HPP

#include <cmath>

class LegController {
private:
    double tau_integral[2] = {0};

public:
    void compute_SMC(int leg_id, double dt, const double* q_ref, const double* q, const double* dq_ref, const double* dq, double* tau_out) {
        // Gain Scheduling
        double K_gain[2] = { (leg_id < 2) ? 100.0 : 30.0, 40.0 };
        double lambda_gain[2] = { (leg_id < 2) ? 150.0 : 60.0, 150.0 };
        double Phi = 5.0; 

        // SMC Core
        for(int j = 0; j < 2; j++) {
            double e = q_ref[j] - q[j];
            double de = dq_ref[j] - dq[j];
            double s = de + lambda_gain[j] * e;
            
            double sat_s = s / Phi;
            if (sat_s > 1.0) sat_s = 1.0;
            if (sat_s < -1.0) sat_s = -1.0;
            
            tau_out[j] = K_gain[j] * sat_s;
        }

        // Bù Trọng Lực
        double Ki_hip = (leg_id < 2) ? 70.0 : 40.0;   
        double Ki_knee = 15.0;  

        tau_integral[0] += Ki_hip * (q_ref[0] - q[0]) * dt;
        tau_integral[1] += Ki_knee * (q_ref[1] - q[1]) * dt;

        double max_int_hip = (leg_id < 2) ? 45.0 : 25.0; 
        double max_int_knee = 12.0;

        if(tau_integral[0] > max_int_hip) tau_integral[0] = max_int_hip;
        else if(tau_integral[0] < -max_int_hip) tau_integral[0] = -max_int_hip;
        
        if(tau_integral[1] > max_int_knee) tau_integral[1] = max_int_knee;
        else if(tau_integral[1] < -max_int_knee) tau_integral[1] = -max_int_knee;

        tau_out[0] += tau_integral[0];
        tau_out[1] += tau_integral[1];

        // Giảm chấn
        tau_out[0] -= 1.5 * dq[0];
        tau_out[1] -= 1.5 * dq[1];

        // Kẹp Max Torque
        for(int j=0; j<2; j++) {
            if (tau_out[j] > 100.0) tau_out[j] = 100.0;
            if (tau_out[j] < -100.0) tau_out[j] = -100.0;
        }
    }
};

#endif