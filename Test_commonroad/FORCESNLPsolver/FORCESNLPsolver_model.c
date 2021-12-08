/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) FORCESNLPsolver_model_ ## ID
#endif

#include <math.h> 
#include "FORCESNLPsolver_model.h"

#ifndef casadi_real
#define casadi_real FORCESNLPsolver_float
#endif

#ifndef casadi_int
#define casadi_int solver_int32_default
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_f3 CASADI_PREFIX(f3)
#define casadi_f4 CASADI_PREFIX(f4)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#if 0
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s1[4] = {0, 1, 0, 0};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s3[12] = {1, 6, 0, 1, 2, 2, 3, 3, 3, 0, 0, 0};
static const casadi_int casadi_s4[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s5[23] = {4, 6, 0, 3, 6, 7, 8, 11, 14, 0, 1, 2, 0, 1, 3, 0, 1, 0, 1, 2, 0, 1, 3};
static const casadi_int casadi_s6[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s7[13] = {2, 6, 0, 0, 0, 2, 4, 4, 4, 0, 1, 0, 1};

/* FORCESNLPsolver_objective_0:(i0[6],i1[0])->(o0,o1[1x6,3nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6;
  a0=-100.;
  a1=arg[0]? arg[0][3] : 0;
  a1=(a0*a1);
  a2=1.0000000000000001e-01;
  a3=arg[0]? arg[0][0] : 0;
  a4=casadi_sq(a3);
  a4=(a2*a4);
  a1=(a1+a4);
  a4=1.0000000000000000e-02;
  a5=arg[0]? arg[0][1] : 0;
  a6=casadi_sq(a5);
  a6=(a4*a6);
  a1=(a1+a6);
  if (res[0]!=0) res[0][0]=a1;
  a3=(a3+a3);
  a2=(a2*a3);
  if (res[1]!=0) res[1][0]=a2;
  a5=(a5+a5);
  a4=(a4*a5);
  if (res[1]!=0) res[1][1]=a4;
  if (res[1]!=0) res[1][2]=a0;
  return 0;
}

int FORCESNLPsolver_objective_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

int FORCESNLPsolver_objective_0_alloc_mem(void) {
  return 0;
}

int FORCESNLPsolver_objective_0_init_mem(int mem) {
  return 0;
}

void FORCESNLPsolver_objective_0_free_mem(int mem) {
}

int FORCESNLPsolver_objective_0_checkout(void) {
  return 0;
}

void FORCESNLPsolver_objective_0_release(int mem) {
}

void FORCESNLPsolver_objective_0_incref(void) {
}

void FORCESNLPsolver_objective_0_decref(void) {
}

casadi_int FORCESNLPsolver_objective_0_n_in(void) { return 2;}

casadi_int FORCESNLPsolver_objective_0_n_out(void) { return 2;}

casadi_real FORCESNLPsolver_objective_0_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* FORCESNLPsolver_objective_0_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

const char* FORCESNLPsolver_objective_0_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_objective_0_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_objective_0_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    default: return 0;
  }
}

int FORCESNLPsolver_objective_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_dynamics_0:(i0[6],i1[0])->(o0[4],o1[4x6,14nz]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][2] : 0;
  a1=1.6666666666666666e-02;
  a2=arg[0]? arg[0][4] : 0;
  a3=arg[0]? arg[0][5] : 0;
  a4=cos(a3);
  a5=(a2*a4);
  a6=2.;
  a7=5.0000000000000003e-02;
  a8=arg[0]? arg[0][0] : 0;
  a9=(a7*a8);
  a9=(a2+a9);
  a10=arg[0]? arg[0][1] : 0;
  a11=(a7*a10);
  a11=(a3+a11);
  a12=cos(a11);
  a13=(a9*a12);
  a13=(a6*a13);
  a5=(a5+a13);
  a13=(a7*a8);
  a13=(a2+a13);
  a14=(a7*a10);
  a14=(a3+a14);
  a15=cos(a14);
  a16=(a13*a15);
  a16=(a6*a16);
  a5=(a5+a16);
  a16=1.0000000000000001e-01;
  a17=(a16*a8);
  a17=(a2+a17);
  a18=(a16*a10);
  a18=(a3+a18);
  a19=cos(a18);
  a20=(a17*a19);
  a5=(a5+a20);
  a5=(a1*a5);
  a0=(a0+a5);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0]? arg[0][3] : 0;
  a5=sin(a3);
  a20=(a2*a5);
  a21=sin(a11);
  a22=(a9*a21);
  a22=(a6*a22);
  a20=(a20+a22);
  a22=sin(a14);
  a23=(a13*a22);
  a23=(a6*a23);
  a20=(a20+a23);
  a23=sin(a18);
  a24=(a17*a23);
  a20=(a20+a24);
  a20=(a1*a20);
  a0=(a0+a20);
  if (res[0]!=0) res[0][1]=a0;
  a0=(a6*a8);
  a0=(a8+a0);
  a20=(a6*a8);
  a0=(a0+a20);
  a0=(a0+a8);
  a0=(a1*a0);
  a0=(a2+a0);
  if (res[0]!=0) res[0][2]=a0;
  a0=(a6*a10);
  a0=(a10+a0);
  a8=(a6*a10);
  a0=(a0+a8);
  a0=(a0+a10);
  a0=(a1*a0);
  a0=(a3+a0);
  if (res[0]!=0) res[0][3]=a0;
  a0=(a7*a12);
  a0=(a6*a0);
  a10=(a7*a15);
  a10=(a6*a10);
  a0=(a0+a10);
  a10=(a16*a19);
  a0=(a0+a10);
  a0=(a1*a0);
  if (res[1]!=0) res[1][0]=a0;
  a0=(a7*a21);
  a0=(a6*a0);
  a10=(a7*a22);
  a10=(a6*a10);
  a0=(a0+a10);
  a10=(a16*a23);
  a0=(a0+a10);
  a0=(a1*a0);
  if (res[1]!=0) res[1][1]=a0;
  if (res[1]!=0) res[1][2]=a16;
  a0=sin(a11);
  a10=(a7*a0);
  a10=(a9*a10);
  a10=(a6*a10);
  a8=sin(a14);
  a20=(a7*a8);
  a20=(a13*a20);
  a20=(a6*a20);
  a10=(a10+a20);
  a20=sin(a18);
  a24=(a16*a20);
  a24=(a17*a24);
  a10=(a10+a24);
  a10=(a1*a10);
  a10=(-a10);
  if (res[1]!=0) res[1][3]=a10;
  a11=cos(a11);
  a10=(a7*a11);
  a10=(a9*a10);
  a10=(a6*a10);
  a14=cos(a14);
  a7=(a7*a14);
  a7=(a13*a7);
  a7=(a6*a7);
  a10=(a10+a7);
  a18=cos(a18);
  a7=(a16*a18);
  a7=(a17*a7);
  a10=(a10+a7);
  a10=(a1*a10);
  if (res[1]!=0) res[1][4]=a10;
  if (res[1]!=0) res[1][5]=a16;
  a16=1.;
  if (res[1]!=0) res[1][6]=a16;
  if (res[1]!=0) res[1][7]=a16;
  a12=(a6*a12);
  a4=(a4+a12);
  a15=(a6*a15);
  a4=(a4+a15);
  a4=(a4+a19);
  a4=(a1*a4);
  if (res[1]!=0) res[1][8]=a4;
  a21=(a6*a21);
  a5=(a5+a21);
  a22=(a6*a22);
  a5=(a5+a22);
  a5=(a5+a23);
  a5=(a1*a5);
  if (res[1]!=0) res[1][9]=a5;
  if (res[1]!=0) res[1][10]=a16;
  a5=sin(a3);
  a5=(a2*a5);
  a0=(a9*a0);
  a0=(a6*a0);
  a5=(a5+a0);
  a8=(a13*a8);
  a8=(a6*a8);
  a5=(a5+a8);
  a20=(a17*a20);
  a5=(a5+a20);
  a5=(a1*a5);
  a5=(-a5);
  if (res[1]!=0) res[1][11]=a5;
  a3=cos(a3);
  a2=(a2*a3);
  a9=(a9*a11);
  a9=(a6*a9);
  a2=(a2+a9);
  a13=(a13*a14);
  a6=(a6*a13);
  a2=(a2+a6);
  a17=(a17*a18);
  a2=(a2+a17);
  a1=(a1*a2);
  if (res[1]!=0) res[1][12]=a1;
  if (res[1]!=0) res[1][13]=a16;
  return 0;
}

int FORCESNLPsolver_dynamics_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f1(arg, res, iw, w, mem);
}

int FORCESNLPsolver_dynamics_0_alloc_mem(void) {
  return 0;
}

int FORCESNLPsolver_dynamics_0_init_mem(int mem) {
  return 0;
}

void FORCESNLPsolver_dynamics_0_free_mem(int mem) {
}

int FORCESNLPsolver_dynamics_0_checkout(void) {
  return 0;
}

void FORCESNLPsolver_dynamics_0_release(int mem) {
}

void FORCESNLPsolver_dynamics_0_incref(void) {
}

void FORCESNLPsolver_dynamics_0_decref(void) {
}

casadi_int FORCESNLPsolver_dynamics_0_n_in(void) { return 2;}

casadi_int FORCESNLPsolver_dynamics_0_n_out(void) { return 2;}

casadi_real FORCESNLPsolver_dynamics_0_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* FORCESNLPsolver_dynamics_0_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

const char* FORCESNLPsolver_dynamics_0_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_dynamics_0_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_dynamics_0_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s5;
    default: return 0;
  }
}

int FORCESNLPsolver_dynamics_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_inequalities_0:(i0[6],i1[0])->(o0[2],o1[2x6,4nz]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5;
  a0=arg[0]? arg[0][2] : 0;
  a1=casadi_sq(a0);
  a2=arg[0]? arg[0][3] : 0;
  a3=casadi_sq(a2);
  a1=(a1+a3);
  if (res[0]!=0) res[0][0]=a1;
  a1=2.;
  a1=(a0+a1);
  a3=casadi_sq(a1);
  a4=2.5000000000000000e+00;
  a4=(a2-a4);
  a5=casadi_sq(a4);
  a3=(a3+a5);
  if (res[0]!=0) res[0][1]=a3;
  a0=(a0+a0);
  if (res[1]!=0) res[1][0]=a0;
  a1=(a1+a1);
  if (res[1]!=0) res[1][1]=a1;
  a2=(a2+a2);
  if (res[1]!=0) res[1][2]=a2;
  a4=(a4+a4);
  if (res[1]!=0) res[1][3]=a4;
  return 0;
}

int FORCESNLPsolver_inequalities_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f2(arg, res, iw, w, mem);
}

int FORCESNLPsolver_inequalities_0_alloc_mem(void) {
  return 0;
}

int FORCESNLPsolver_inequalities_0_init_mem(int mem) {
  return 0;
}

void FORCESNLPsolver_inequalities_0_free_mem(int mem) {
}

int FORCESNLPsolver_inequalities_0_checkout(void) {
  return 0;
}

void FORCESNLPsolver_inequalities_0_release(int mem) {
}

void FORCESNLPsolver_inequalities_0_incref(void) {
}

void FORCESNLPsolver_inequalities_0_decref(void) {
}

casadi_int FORCESNLPsolver_inequalities_0_n_in(void) { return 2;}

casadi_int FORCESNLPsolver_inequalities_0_n_out(void) { return 2;}

casadi_real FORCESNLPsolver_inequalities_0_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* FORCESNLPsolver_inequalities_0_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

const char* FORCESNLPsolver_inequalities_0_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_inequalities_0_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_inequalities_0_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s6;
    case 1: return casadi_s7;
    default: return 0;
  }
}

int FORCESNLPsolver_inequalities_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_objective_1:(i0[6],i1[0])->(o0,o1[1x6,3nz]) */
static int casadi_f3(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6;
  a0=-100.;
  a1=arg[0]? arg[0][3] : 0;
  a1=(a0*a1);
  a2=1.0000000000000001e-01;
  a3=arg[0]? arg[0][0] : 0;
  a4=casadi_sq(a3);
  a4=(a2*a4);
  a1=(a1+a4);
  a4=1.0000000000000000e-02;
  a5=arg[0]? arg[0][1] : 0;
  a6=casadi_sq(a5);
  a6=(a4*a6);
  a1=(a1+a6);
  if (res[0]!=0) res[0][0]=a1;
  a3=(a3+a3);
  a2=(a2*a3);
  if (res[1]!=0) res[1][0]=a2;
  a5=(a5+a5);
  a4=(a4*a5);
  if (res[1]!=0) res[1][1]=a4;
  if (res[1]!=0) res[1][2]=a0;
  return 0;
}

int FORCESNLPsolver_objective_1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f3(arg, res, iw, w, mem);
}

int FORCESNLPsolver_objective_1_alloc_mem(void) {
  return 0;
}

int FORCESNLPsolver_objective_1_init_mem(int mem) {
  return 0;
}

void FORCESNLPsolver_objective_1_free_mem(int mem) {
}

int FORCESNLPsolver_objective_1_checkout(void) {
  return 0;
}

void FORCESNLPsolver_objective_1_release(int mem) {
}

void FORCESNLPsolver_objective_1_incref(void) {
}

void FORCESNLPsolver_objective_1_decref(void) {
}

casadi_int FORCESNLPsolver_objective_1_n_in(void) { return 2;}

casadi_int FORCESNLPsolver_objective_1_n_out(void) { return 2;}

casadi_real FORCESNLPsolver_objective_1_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* FORCESNLPsolver_objective_1_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

const char* FORCESNLPsolver_objective_1_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_objective_1_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_objective_1_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    default: return 0;
  }
}

int FORCESNLPsolver_objective_1_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_inequalities_1:(i0[6],i1[0])->(o0[2],o1[2x6,4nz]) */
static int casadi_f4(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5;
  a0=arg[0]? arg[0][2] : 0;
  a1=casadi_sq(a0);
  a2=arg[0]? arg[0][3] : 0;
  a3=casadi_sq(a2);
  a1=(a1+a3);
  if (res[0]!=0) res[0][0]=a1;
  a1=2.;
  a1=(a0+a1);
  a3=casadi_sq(a1);
  a4=2.5000000000000000e+00;
  a4=(a2-a4);
  a5=casadi_sq(a4);
  a3=(a3+a5);
  if (res[0]!=0) res[0][1]=a3;
  a0=(a0+a0);
  if (res[1]!=0) res[1][0]=a0;
  a1=(a1+a1);
  if (res[1]!=0) res[1][1]=a1;
  a2=(a2+a2);
  if (res[1]!=0) res[1][2]=a2;
  a4=(a4+a4);
  if (res[1]!=0) res[1][3]=a4;
  return 0;
}

int FORCESNLPsolver_inequalities_1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f4(arg, res, iw, w, mem);
}

int FORCESNLPsolver_inequalities_1_alloc_mem(void) {
  return 0;
}

int FORCESNLPsolver_inequalities_1_init_mem(int mem) {
  return 0;
}

void FORCESNLPsolver_inequalities_1_free_mem(int mem) {
}

int FORCESNLPsolver_inequalities_1_checkout(void) {
  return 0;
}

void FORCESNLPsolver_inequalities_1_release(int mem) {
}

void FORCESNLPsolver_inequalities_1_incref(void) {
}

void FORCESNLPsolver_inequalities_1_decref(void) {
}

casadi_int FORCESNLPsolver_inequalities_1_n_in(void) { return 2;}

casadi_int FORCESNLPsolver_inequalities_1_n_out(void) { return 2;}

casadi_real FORCESNLPsolver_inequalities_1_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* FORCESNLPsolver_inequalities_1_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

const char* FORCESNLPsolver_inequalities_1_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_inequalities_1_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_inequalities_1_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s6;
    case 1: return casadi_s7;
    default: return 0;
  }
}

int FORCESNLPsolver_inequalities_1_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
