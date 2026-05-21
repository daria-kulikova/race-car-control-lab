/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_linear_model_with_params_nx9_nu3_np31_H_
#define ACADOS_SOLVER_linear_model_with_params_nx9_nu3_np31_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NX     9
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NZ     0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NU     3
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NP     148
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NP_GLOBAL     0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NBX    9
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NBX0   9
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NBU    3
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSBX   6
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSBU   0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSH    1
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSH0   0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSG    0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSPHI  0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSHN   1
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSGN   0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSPHIN 0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSPHI0 0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSBXN  6
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NS     7
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NS0    0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NSN    7
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NG     0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NBXN   9
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NGN    0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NY0    6
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NY     6
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NYN    0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_N      40
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NH     1
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NHN    1
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NH0    0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NPHI0  0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NPHI   0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NPHIN  0
#define LINEAR_MODEL_WITH_PARAMS_NX9_NU3_NP31_NR     0

#ifdef __cplusplus
extern "C" {
#endif

// ** capsule for solver data **
typedef struct linear_model_with_params_nx9_nu3_np31_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *discr_dyn_phi_fun;
    external_function_external_param_casadi *discr_dyn_phi_fun_jac_ut_xt;




    // cost

    external_function_external_param_casadi *cost_y_fun;
    external_function_external_param_casadi *cost_y_fun_jac_ut_xt;



    external_function_external_param_casadi cost_y_0_fun;
    external_function_external_param_casadi cost_y_0_fun_jac_ut_xt;



    external_function_external_param_casadi cost_y_e_fun;
    external_function_external_param_casadi cost_y_e_fun_jac_ut_xt;


    // constraints
    external_function_external_param_casadi *nl_constr_h_fun_jac;
    external_function_external_param_casadi *nl_constr_h_fun;








    external_function_external_param_casadi nl_constr_h_e_fun_jac;
    external_function_external_param_casadi nl_constr_h_e_fun;


    void * custom_update_memory;

} linear_model_with_params_nx9_nu3_np31_solver_capsule;

ACADOS_SYMBOL_EXPORT linear_model_with_params_nx9_nu3_np31_solver_capsule * linear_model_with_params_nx9_nu3_np31_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_free_capsule(linear_model_with_params_nx9_nu3_np31_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_create(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_reset(linear_model_with_params_nx9_nu3_np31_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of linear_model_with_params_nx9_nu3_np31_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_create_with_discretization(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_update_time_steps(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_update_qp_solver_cond_N(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_update_params(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_update_params_sparse(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_set_p_global_and_precompute_dependencies(linear_model_with_params_nx9_nu3_np31_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_solve(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_setup_qp_matrices_and_factorize(linear_model_with_params_nx9_nu3_np31_solver_capsule* capsule);



ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_free(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void linear_model_with_params_nx9_nu3_np31_acados_print_stats(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int linear_model_with_params_nx9_nu3_np31_acados_custom_update(linear_model_with_params_nx9_nu3_np31_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *linear_model_with_params_nx9_nu3_np31_acados_get_nlp_in(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *linear_model_with_params_nx9_nu3_np31_acados_get_nlp_out(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *linear_model_with_params_nx9_nu3_np31_acados_get_sens_out(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *linear_model_with_params_nx9_nu3_np31_acados_get_nlp_solver(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *linear_model_with_params_nx9_nu3_np31_acados_get_nlp_config(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *linear_model_with_params_nx9_nu3_np31_acados_get_nlp_opts(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *linear_model_with_params_nx9_nu3_np31_acados_get_nlp_dims(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *linear_model_with_params_nx9_nu3_np31_acados_get_nlp_plan(linear_model_with_params_nx9_nu3_np31_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_linear_model_with_params_nx9_nu3_np31_H_
