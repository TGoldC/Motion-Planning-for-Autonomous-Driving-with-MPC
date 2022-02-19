/*
FORCESNLPsolver : A fast customized optimization solver.

Copyright (C) 2013-2021 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

#include "mex.h"
#include "math.h"
#include "../include/FORCESNLPsolver.h"
#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif



/* copy functions */

void copyCArrayToM_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_double(double *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double) (*src++) ;
    }
}

void copyMValueToC_double(double * src, double * dest)
{
	*dest = (double) *src;
}

/* copy functions */

void copyCArrayToM_FORCESNLPsolver_float(FORCESNLPsolver_float *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_FORCESNLPsolver_float(double *src, FORCESNLPsolver_float *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (FORCESNLPsolver_float) (*src++) ;
    }
}

void copyMValueToC_FORCESNLPsolver_float(double * src, FORCESNLPsolver_float * dest)
{
	*dest = (FORCESNLPsolver_float) *src;
}

/* copy functions */

void copyCArrayToM_FORCESNLPsolver_int(FORCESNLPsolver_int *src, double *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (double)*src++;
    }
}

void copyMArrayToC_FORCESNLPsolver_int(double *src, FORCESNLPsolver_int *dest, solver_int32_default dim) 
{
    solver_int32_default i;
    for( i = 0; i < dim; i++ ) 
    {
        *dest++ = (FORCESNLPsolver_int) (*src++) ;
    }
}

void copyMValueToC_FORCESNLPsolver_int(double * src, FORCESNLPsolver_int * dest)
{
	*dest = (FORCESNLPsolver_int) *src;
}



extern void (FORCESNLPsolver_float *x, FORCESNLPsolver_float *y, FORCESNLPsolver_float *l, FORCESNLPsolver_float *p, FORCESNLPsolver_float *f, FORCESNLPsolver_float *nabla_f, FORCESNLPsolver_float *c, FORCESNLPsolver_float *nabla_c, FORCESNLPsolver_float *h, FORCESNLPsolver_float *nabla_h, FORCESNLPsolver_float *hess, solver_int32_default stage, solver_int32_default iteration, solver_int32_default threadID);
FORCESNLPsolver_extfunc pt2function_FORCESNLPsolver = &;


/* Some memory for mex-function */
static FORCESNLPsolver_params params;
static FORCESNLPsolver_output output;
static FORCESNLPsolver_info info;

/* THE mex-function */
void mexFunction( solver_int32_default nlhs, mxArray *plhs[], solver_int32_default nrhs, const mxArray *prhs[] )  
{
	/* file pointer for printing */
	FILE *fp = NULL;

	/* define variables */	
	mxArray *par;
	mxArray *outvar;
	const mxArray *PARAMS = prhs[0]; 
	double *pvalue;
	solver_int32_default i;
	solver_int32_default exitflag;
	const solver_int8_default *fname;
	const solver_int8_default *outputnames[10] = {"x01","x02","x03","x04","x05","x06","x07","x08","x09","x10"};
	const solver_int8_default *infofields[7] = { "it", "res_eq", "rsnorm", "pobj", "solvetime",  "fevalstime", "QPtime"};
	
	/* Check for proper number of arguments */
    if (nrhs != 1)
	{
		mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help FORCESNLPsolver_mex' for details.");
	}    
	if (nlhs > 3) 
	{
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help FORCESNLPsolver_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) 
	{
		mexErrMsgTxt("PARAMS must be a structure.");
	}
	 

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "xinit");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.xinit not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.xinit must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.xinit must be of size [5 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.xinit,5);

	}
	par = mxGetField(PARAMS, 0, "x0");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.x0 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.x0 must be a double.");
    }
    if( mxGetM(par) != 70 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.x0 must be of size [70 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.x0,70);

	}
	par = mxGetField(PARAMS, 0, "all_parameters");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	
	{
        mexErrMsgTxt("PARAMS.all_parameters not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.all_parameters must be a double.");
    }
    if( mxGetM(par) != 100 || mxGetN(par) != 1 ) 
	{
    mexErrMsgTxt("PARAMS.all_parameters must be of size [100 x 1]");
    }
#endif	 
	if ( (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMArrayToC_double(mxGetPr(par), params.all_parameters,100);

	}
	par = mxGetField(PARAMS, 0, "ToleranceInequalities");
	if ( (par != NULL) && (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMValueToC_FORCESNLPsolver_float(mxGetPr(par), &params.ToleranceInequalities);

	}


	par = mxGetField(PARAMS, 0, "ToleranceEqualities");
	if ( (par != NULL) && (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMValueToC_FORCESNLPsolver_float(mxGetPr(par), &params.ToleranceEqualities);

	}


	par = mxGetField(PARAMS, 0, "reinitialize");
	if ( (par != NULL) && (mxGetN(par) != 0) && (mxGetM(par) != 0) )
	{
		copyMValueToC_FORCESNLPsolver_int(mxGetPr(par), &params.reinitialize);

	}




	#if SET_PRINTLEVEL_FORCESNLPsolver > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) 
		{
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* call solver */
	exitflag = FORCESNLPsolver_solve(&params, &output, &info, fp, pt2function_FORCESNLPsolver);
	
	#if SET_PRINTLEVEL_FORCESNLPsolver > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) 
		{
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 10, outputnames);
		outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM_double( output.x01, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "x01", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM_double( output.x02, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "x02", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM_double( output.x03, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "x03", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM_double( output.x04, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "x04", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM_double( output.x05, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "x05", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM_double( output.x06, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "x06", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM_double( output.x07, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "x07", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM_double( output.x08, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "x08", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM_double( output.x09, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "x09", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM_double( output.x10, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "x10", outvar);



	/* copy exitflag */
	if( nlhs > 1 )
	{
	plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
	*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
	        plhs[2] = mxCreateStructMatrix(1, 1, 7, infofields);
        
		/* iterations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it;
		mxSetField(plhs[2], 0, "it", outvar);

		/* res_eq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_eq;
		mxSetField(plhs[2], 0, "res_eq", outvar);

		/* rsnorm */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rsnorm;
		mxSetField(plhs[2], 0, "rsnorm", outvar);

		/* pobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.pobj;
		mxSetField(plhs[2], 0, "pobj", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.solvetime;
		mxSetField(plhs[2], 0, "solvetime", outvar);

		/* fevals time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.fevalstime;
		mxSetField(plhs[2], 0, "fevalstime", outvar);
		
		/* QP time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.QPtime;
		mxSetField(plhs[2], 0, "QPtime", outvar);		

	}
}
