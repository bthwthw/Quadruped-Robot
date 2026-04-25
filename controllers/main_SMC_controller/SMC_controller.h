/*
 * SMC_controller.h
 *
 * Code generation for function 'SMC_controller'
 *
 */

#ifndef SMC_CONTROLLER_H
#define SMC_CONTROLLER_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void SMC_controller(const double q_ref[2], const double dq_ref[2],
                           const double ddq_ref[2], const double q[2],
                           const double dq[2], const double K_val[2],
                           const double lambda_val[2], double tau[2]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (SMC_controller.h) */
