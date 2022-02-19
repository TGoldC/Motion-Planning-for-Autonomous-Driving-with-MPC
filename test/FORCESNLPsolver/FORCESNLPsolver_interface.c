/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/FORCESNLPsolver.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "FORCESNLPsolver_model.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, FORCESNLPsolver_callback_float *data, FORCESNLPsolver_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((FORCESNLPsolver_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void FORCESNLPsolver_casadi2forces(FORCESNLPsolver_float *x,        /* primal vars                                         */
                                 FORCESNLPsolver_float *y,        /* eq. constraint multiplers                           */
                                 FORCESNLPsolver_float *l,        /* ineq. constraint multipliers                        */
                                 FORCESNLPsolver_float *p,        /* parameters                                          */
                                 FORCESNLPsolver_float *f,        /* objective function (scalar)                         */
                                 FORCESNLPsolver_float *nabla_f,  /* gradient of objective function                      */
                                 FORCESNLPsolver_float *c,        /* dynamics                                            */
                                 FORCESNLPsolver_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCESNLPsolver_float *h,        /* inequality constraints                              */
                                 FORCESNLPsolver_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCESNLPsolver_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const FORCESNLPsolver_callback_float *in[4];
    FORCESNLPsolver_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	FORCESNLPsolver_float w[38];
	
    /* temporary storage for CasADi sparse output */
    FORCESNLPsolver_callback_float this_f;
    FORCESNLPsolver_float nabla_f_sparse[7];
    FORCESNLPsolver_float h_sparse[10];
    FORCESNLPsolver_float nabla_h_sparse[27];
    FORCESNLPsolver_float c_sparse[5];
    FORCESNLPsolver_float nabla_c_sparse[21];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
    in[0] = x;
    in[1] = p;
    in[2] = l;
    in[3] = y;

	if ((0 <= stage && stage <= 8))
	{
		
		
		if( &this_f )
		{
			out[0] = &this_f;
			FORCESNLPsolver_objective_0(in, out, NULL, w, 0);
		}
		
		if( nabla_f )
		{
			out[0] = nabla_f_sparse;
			FORCESNLPsolver_dobjective_0(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_dobjective_0_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_dobjective_0_sparsity_out(0)[1];
			colind = FORCESNLPsolver_dobjective_0_sparsity_out(0) + 2;
			row = FORCESNLPsolver_dobjective_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		if( c )
		{
			out[0] = c_sparse;
			FORCESNLPsolver_dynamics_0(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_dynamics_0_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_dynamics_0_sparsity_out(0)[1];
			colind = FORCESNLPsolver_dynamics_0_sparsity_out(0) + 2;
			row = FORCESNLPsolver_dynamics_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}
		
		if( nabla_c )
		{
			out[0] = nabla_c_sparse;
			FORCESNLPsolver_ddynamics_0(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_ddynamics_0_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_ddynamics_0_sparsity_out(0)[1];
			colind = FORCESNLPsolver_ddynamics_0_sparsity_out(0) + 2;
			row = FORCESNLPsolver_ddynamics_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}
		
		if( h )
		{
			out[0] = h_sparse;
			FORCESNLPsolver_inequalities_0(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_inequalities_0_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_inequalities_0_sparsity_out(0)[1];
			colind = FORCESNLPsolver_inequalities_0_sparsity_out(0) + 2;
			row = FORCESNLPsolver_inequalities_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		
		if( nabla_h )
		{
			out[0] = nabla_h_sparse;
			FORCESNLPsolver_dinequalities_0(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_dinequalities_0_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_dinequalities_0_sparsity_out(0)[1];
			colind = FORCESNLPsolver_dinequalities_0_sparsity_out(0) + 2;
			row = FORCESNLPsolver_dinequalities_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
	if ((9 == stage))
	{
		
		
		if( &this_f )
		{
			out[0] = &this_f;
			FORCESNLPsolver_objective_1(in, out, NULL, w, 0);
		}
		
		if( nabla_f )
		{
			out[0] = nabla_f_sparse;
			FORCESNLPsolver_dobjective_1(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_dobjective_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_dobjective_1_sparsity_out(0)[1];
			colind = FORCESNLPsolver_dobjective_1_sparsity_out(0) + 2;
			row = FORCESNLPsolver_dobjective_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		if( h )
		{
			out[0] = h_sparse;
			FORCESNLPsolver_inequalities_1(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_inequalities_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_inequalities_1_sparsity_out(0)[1];
			colind = FORCESNLPsolver_inequalities_1_sparsity_out(0) + 2;
			row = FORCESNLPsolver_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		
		if( nabla_h )
		{
			out[0] = nabla_h_sparse;
			FORCESNLPsolver_dinequalities_1(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_dinequalities_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_dinequalities_1_sparsity_out(0)[1];
			colind = FORCESNLPsolver_dinequalities_1_sparsity_out(0) + 2;
			row = FORCESNLPsolver_dinequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((FORCESNLPsolver_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
