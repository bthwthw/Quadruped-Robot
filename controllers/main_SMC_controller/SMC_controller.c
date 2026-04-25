/*
 * SMC_controller.c
 *
 * Code generation for function 'SMC_controller'
 *
 */

/* Include files */
#include "SMC_controller.h"
#include <math.h>

/* Function Definitions */
void SMC_controller(const double q_ref[2], const double dq_ref[2],
                    const double ddq_ref[2], const double q[2],
                    const double dq[2], const double K_val[2],
                    const double lambda_val[2], double tau[2])
{
  double M[4];
  double b_q[2];
  double de[2];
  double s[2];
  double d;
  double d1;
  double lambda_idx_0;
  double lambda_idx_3;
  double sat_s_idx_0;
  double sat_s_idx_1;
  int i;
  /*  Ma trận hệ số mặt trượt */
  /*  Độ lợi điều khiển */
  /*  Bề dày lớp biên để chống rung  */
  /*     %% 2. Tính toán Ma trận Động lực học  */
  lambda_idx_3 = cos(q[1]);
  M[0] = lambda_idx_3 / 50.0 + 0.040128;
  lambda_idx_3 = lambda_idx_3 / 100.0 + 0.0066948;
  lambda_idx_0 = sin(q[1]);
  /*     %% 3. Thuật toán SMC */
  /*  Sai số vị trí */
  /*  Sai số vận tốc */
  de[0] = dq[0] - dq_ref[0];
  b_q[0] = q[0] - q_ref[0];
  de[1] = dq[1] - dq_ref[1];
  b_q[1] = q[1] - q_ref[1];
  /*  Điều khiển tương đương  */
  /*  Điều khiển bù  */
  d = de[0] + (lambda_val[0] * b_q[0] + 0.0 * b_q[1]);
  if (d > 0.1) {
    sat_s_idx_0 = 1.0;
  } else if (d < -0.1) {
    sat_s_idx_0 = -1.0;
  } else {
    sat_s_idx_0 = d / 0.1;
  }
  d = de[1] + (0.0 * b_q[0] + b_q[1] * lambda_val[1]);
  if (d > 0.1) {
    sat_s_idx_1 = 1.0;
  } else if (d < -0.1) {
    sat_s_idx_1 = -1.0;
  } else {
    sat_s_idx_1 = d / 0.1;
  }
  /*     %% 4. Đầu ra mô-men xoắn ([2x1]) */
  b_q[0] = ddq_ref[0] - (lambda_val[0] * de[0] + 0.0 * de[1]);
  b_q[1] = ddq_ref[1] - (0.0 * de[0] + de[1] * lambda_val[1]);
  de[0] = M[0] * b_q[0] + b_q[1] * lambda_idx_3;
  de[1] = b_q[0] * lambda_idx_3 + 0.0066948 * b_q[1];
  b_q[0] = dq[1] * lambda_idx_0 * -0.01 * dq[0] +
           dq[1] * (lambda_idx_0 * (dq[0] + dq[1]) * -0.01);
  b_q[1] = dq[0] * (dq[0] * lambda_idx_0 / 100.0) + 0.0 * dq[1];
  lambda_idx_0 = cos(q[0] + q[1]) * 0.4905;
  s[0] = lambda_idx_0 + cos(q[0]) * 1.962;
  s[1] = lambda_idx_0;
  M[0] = -M[0];
  M[1] = -lambda_idx_3;
  M[2] = -lambda_idx_3;
  M[3] = -0.0066948;
  lambda_idx_0 = K_val[0];
  lambda_idx_3 = K_val[1];
  for (i = 0; i < 2; i++) {
    d = M[i + 2];
    d1 = M[i];
    tau[i] = ((de[i] + b_q[i]) + s[i]) +
             ((d1 * lambda_idx_0 + d * 0.0) * sat_s_idx_0 +
              (d1 * 0.0 + d * lambda_idx_3) * sat_s_idx_1);
  }
}

/* End of code generation (SMC_controller.c) */
