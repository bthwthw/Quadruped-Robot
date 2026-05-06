/*
 * SMC_controller.h  –  patched for ROS2 build (no MATLAB dependency)
 */

#ifndef SMC_CONTROLLER_H
#define SMC_CONTROLLER_H

#include "rtwtypes.h"   /* standalone – không dùng tmwtypes.h */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void SMC_controller(const double q_ref[2], const double dq_ref[2],
                           const double ddq_ref[2], const double q[2],
                           const double dq[2], const double K_val[2],
                           const double lambda_val[2], double tau[2]);

#ifdef __cplusplus
}
#endif

#endif /* SMC_CONTROLLER_H */
