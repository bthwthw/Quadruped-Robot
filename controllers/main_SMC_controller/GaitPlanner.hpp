#ifndef GAIT_PLANNER_HPP
#define GAIT_PLANNER_HPP

#include <cmath>

class GaitPlanner {
public:
    double T, S, H, y0, L1, L2;

    GaitPlanner() {
        T = 0.5;      // Chu kỳ
        S = 0.08;     // Sải chân
        H = 0.03;     // Nhấc chân
        y0 = -0.23;   // Chiều cao thân
        L1 = 0.2;     // Đùi
        L2 = 0.2;     // Cẳng
    }

    // Động học nghịch 
    void calculate_Leg_IK(double t_total, double offset, double* q_ref_pair) {
        double theta = (t_total / T + offset) * 2.0 * M_PI;
        double theta_mod = fmod(theta, 2.0 * M_PI);
        if (theta_mod < 0) theta_mod += 2.0 * M_PI;

        double x = 0.0, y = 0.0;
        if (theta_mod < M_PI) {
            x = (S / 2.0) * cos(theta_mod); 
            y = y0 + H * sin(theta_mod);           
        } else {
            x = -(S / 2.0) * cos(theta_mod - M_PI);  
            y = y0;                            
        }

        double L_square = x * x + y * y;
        double max_reach = L1 + L2 - 0.001;
        if (L_square > max_reach * max_reach) {
            double scale = max_reach / sqrt(L_square);
            x *= scale; y *= scale;
            L_square = x * x + y * y;
        }

        double cos_q2 = (L_square - L1*L1 - L2*L2) / (2.0 * L1 * L2);
        if (cos_q2 > 1.0) cos_q2 = 1.0;
        if (cos_q2 < -1.0) cos_q2 = -1.0;

        double q2 = -acos(cos_q2); 
        double alpha = atan2(y, x); 
        double beta = atan2(L2 * sin(q2), L1 + L2 * cos(q2));

        q_ref_pair[0] = (alpha - beta) + M_PI / 2.0; 
        q_ref_pair[1] = q2;        
    }


    void get_Trot(double t, double* q_ref) {
        calculate_Leg_IK(t, 0.0, &q_ref[0]); // FL
        calculate_Leg_IK(t, 0.5, &q_ref[2]); // FR
        calculate_Leg_IK(t, 0.0, &q_ref[4]); // BR
        calculate_Leg_IK(t, 0.5, &q_ref[6]); // BL
    }

    void get_Bound(double t, double* q_ref) {
        calculate_Leg_IK(t, 0.0, &q_ref[0]); // FL
        calculate_Leg_IK(t, 0.0, &q_ref[2]); // FR 
        calculate_Leg_IK(t, 0.5, &q_ref[4]); // BR
        calculate_Leg_IK(t, 0.5, &q_ref[6]); // BL 
    }
    
    void get_Pace(double t, double* q_ref) {
        calculate_Leg_IK(t, 0.0, &q_ref[0]); // FL 
        calculate_Leg_IK(t, 0.5, &q_ref[2]); // FR 
        calculate_Leg_IK(t, 0.5, &q_ref[4]); // BR 
        calculate_Leg_IK(t, 0.0, &q_ref[6]); // BL 
    }
    
    void get_Walk(double t, double* q_ref) {
        calculate_Leg_IK(t, 0.00, &q_ref[0]); // FL (Nhấc đầu tiên)
        calculate_Leg_IK(t, 0.50, &q_ref[2]); // FR (Nhấc thứ 3)
        calculate_Leg_IK(t, 0.25, &q_ref[4]); // BR (Nhấc thứ 2)
        calculate_Leg_IK(t, 0.75, &q_ref[6]); // BL (Nhấc cuối cùng)
    }
};

#endif