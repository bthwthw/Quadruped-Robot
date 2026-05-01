#ifndef LEG_CONTROLLER2_HPP
#define LEG_CONTROLLER2_HPP

#include <cmath>

class LegController2 {
private:
    double tau_integral[2] = {0};

    double m1 = 1.0;          // Khối lượng đùi (kg)
    double m2 = 0.5;          // Khối lượng cẳng (kg)
    double L1 = 0.2;          // Chiều dài đùi (m)
    double L2 = 0.2;          // Chiều dài cẳng (m)
    double Lc1 = 0.1;         // Khoảng cách đến khối tâm đùi
    double Lc2 = 0.1;         // Khoảng cách đến khối tâm cẳng
    
    // Quán tính I1, I2 
    double I1 = 0.003433;     // kg.m^2
    double I2 = 0.001695;     // kg.m^2
    
    double g = 9.81; 

public:
    void compute_SMC(int leg_id, double dt, const double* q_ref, const double* q, const double* dq_ref, const double* dq, const double* ddq_ref, double* tau_out) {
        
        // bu goc ban dau 
        double q1_webots = q[0];
        double q2_webots = q[1];

        double offset_hip = -M_PI / 2.0; 
        double offset_knee = 0.0; 

        double q1 = q1_webots + offset_hip; 
        double q2 = q2_webots + offset_knee; 
        
        double dq1 = dq[0];   
        double dq2 = dq[1];   
        
        // G(q) 
        double G1 = (m1 * Lc1 + m2 * L1) * g * cos(q1) + m2 * Lc2 * g * cos(q1 + q2);
        double G2 = m2 * Lc2 * g * cos(q1 + q2);

        // M(q) 
        double M11 = m1 * Lc1 * Lc1 + I1 + m2 * (L1 * L1 + Lc2 * Lc2 + 2 * L1 * Lc2 * cos(q2)) + I2;
        double M12 = m2 * (Lc2 * Lc2 + L1 * Lc2 * cos(q2)) + I2;
        double M21 = M12;
        double M22 = m2 * Lc2 * Lc2 + I2;

        // C(q, dq)
        double h_coriolis = -m2 * L1 * Lc2 * sin(q2);
        double C11 = h_coriolis * dq2;
        double C12 = h_coriolis * (dq1 + dq2);
        double C21 = -h_coriolis * dq1;
        double C22 = 0.0;

        // feed forward 
        double ddq1_r = ddq_ref[0];
        double ddq2_r = ddq_ref[1];
        
        double tau_ff_hip  = (M11 * ddq1_r + M12 * ddq2_r) + (C11 * dq1 + C12 * dq2) + G1;
        double tau_ff_knee = (M21 * ddq1_r + M22 * ddq2_r) + (C21 * dq1 + C22 * dq2) + G2;

        // SMC 
        double K_gain[2] = { 15.0, 8.0 }; 
        double lambda_gain[2] = { 100.0, 100.0 };
        double Phi = 3.0; 

        double tau_smc[2] = {0, 0};

        for(int j = 0; j < 2; j++) {
            double e = q_ref[j] - q[j];
            double de = dq_ref[j] - dq[j];
            double s = de + lambda_gain[j] * e;
            
            double sat_s = s / Phi;
            if (sat_s > 1.0) sat_s = 1.0;
            if (sat_s < -1.0) sat_s = -1.0;
            
            tau_smc[j] = K_gain[j] * sat_s;
        }

        // integral 
        double Ki_hip = 5.0;   
        double Ki_knee = 2.0;   

        tau_integral[0] += Ki_hip * (q_ref[0] - q[0]) * dt;
        tau_integral[1] += Ki_knee * (q_ref[1] - q[1]) * dt;

        double max_int = 5.0;  
        if(tau_integral[0] > max_int) tau_integral[0] = max_int;
        else if(tau_integral[0] < -max_int) tau_integral[0] = -max_int;
        
        if(tau_integral[1] > max_int) tau_integral[1] = max_int;
        else if(tau_integral[1] < -max_int) tau_integral[1] = -max_int;

        // output 
        tau_out[0] = tau_ff_hip  + tau_smc[0] + tau_integral[0] - 1.0 * dq[0];
        tau_out[1] = tau_ff_knee + tau_smc[1] + tau_integral[1] - 1.0 * dq[1];

        // Max Torque
        for(int j=0; j<2; j++) {
            if (tau_out[j] > 100.0) tau_out[j] = 100.0;
            if (tau_out[j] < -100.0) tau_out[j] = -100.0;
        }
    }
};

#endif