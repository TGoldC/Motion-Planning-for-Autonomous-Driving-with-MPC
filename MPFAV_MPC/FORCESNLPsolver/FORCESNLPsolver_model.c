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
#define casadi_f5 CASADI_PREFIX(f5)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
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

static const casadi_int casadi_s0[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s3[10] = {1, 7, 0, 0, 0, 0, 0, 0, 0, 0};
static const casadi_int casadi_s4[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s5[31] = {5, 7, 0, 4, 8, 9, 10, 14, 18, 21, 0, 1, 2, 4, 0, 1, 3, 4, 0, 1, 0, 1, 2, 4, 0, 1, 3, 4, 0, 1, 4};

/* FORCESNLPsolver_objective_0:(i0[7],i1[2])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
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

casadi_int FORCESNLPsolver_objective_0_n_out(void) { return 1;}

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
    default: return 0;
  }
}

int FORCESNLPsolver_objective_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_dobjective_0:(i0[7],i1[2])->(o0[1x7,0nz]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  return 0;
}

int FORCESNLPsolver_dobjective_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f1(arg, res, iw, w, mem);
}

int FORCESNLPsolver_dobjective_0_alloc_mem(void) {
  return 0;
}

int FORCESNLPsolver_dobjective_0_init_mem(int mem) {
  return 0;
}

void FORCESNLPsolver_dobjective_0_free_mem(int mem) {
}

int FORCESNLPsolver_dobjective_0_checkout(void) {
  return 0;
}

void FORCESNLPsolver_dobjective_0_release(int mem) {
}

void FORCESNLPsolver_dobjective_0_incref(void) {
}

void FORCESNLPsolver_dobjective_0_decref(void) {
}

casadi_int FORCESNLPsolver_dobjective_0_n_in(void) { return 2;}

casadi_int FORCESNLPsolver_dobjective_0_n_out(void) { return 1;}

casadi_real FORCESNLPsolver_dobjective_0_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* FORCESNLPsolver_dobjective_0_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

const char* FORCESNLPsolver_dobjective_0_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_dobjective_0_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_dobjective_0_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

int FORCESNLPsolver_dobjective_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_dynamics_0:(i0[7],i1[2])->(o0[5]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][2] : 0;
  a1=1.6666666666666666e-02;
  a2=arg[0]? arg[0][5] : 0;
  a3=arg[0]? arg[0][6] : 0;
  a4=cos(a3);
  a4=(a2*a4);
  a5=2.;
  a6=5.0000000000000003e-02;
  a7=arg[0]? arg[0][1] : 0;
  a8=(a6*a7);
  a8=(a2+a8);
  a9=2.5789127999999999e+00;
  a10=(a2/a9);
  a11=arg[0]? arg[0][4] : 0;
  a12=tan(a11);
  a10=(a10*a12);
  a12=(a6*a10);
  a12=(a3+a12);
  a13=cos(a12);
  a13=(a8*a13);
  a13=(a5*a13);
  a4=(a4+a13);
  a13=(a6*a7);
  a13=(a2+a13);
  a14=(a8/a9);
  a15=arg[0]? arg[0][0] : 0;
  a16=(a6*a15);
  a16=(a11+a16);
  a16=tan(a16);
  a14=(a14*a16);
  a16=(a6*a14);
  a16=(a3+a16);
  a17=cos(a16);
  a17=(a13*a17);
  a17=(a5*a17);
  a4=(a4+a17);
  a17=1.0000000000000001e-01;
  a18=(a17*a7);
  a18=(a2+a18);
  a19=(a13/a9);
  a6=(a6*a15);
  a6=(a11+a6);
  a6=tan(a6);
  a19=(a19*a6);
  a6=(a17*a19);
  a6=(a3+a6);
  a20=cos(a6);
  a20=(a18*a20);
  a4=(a4+a20);
  a4=(a1*a4);
  a0=(a0+a4);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0]? arg[0][3] : 0;
  a4=sin(a3);
  a4=(a2*a4);
  a12=sin(a12);
  a8=(a8*a12);
  a8=(a5*a8);
  a4=(a4+a8);
  a16=sin(a16);
  a13=(a13*a16);
  a13=(a5*a13);
  a4=(a4+a13);
  a6=sin(a6);
  a6=(a18*a6);
  a4=(a4+a6);
  a4=(a1*a4);
  a0=(a0+a4);
  if (res[0]!=0) res[0][1]=a0;
  a0=(a5*a15);
  a0=(a15+a0);
  a4=(a5*a15);
  a0=(a0+a4);
  a0=(a0+a15);
  a0=(a1*a0);
  a0=(a11+a0);
  if (res[0]!=0) res[0][2]=a0;
  a0=(a5*a7);
  a0=(a7+a0);
  a4=(a5*a7);
  a0=(a0+a4);
  a0=(a0+a7);
  a0=(a1*a0);
  a2=(a2+a0);
  if (res[0]!=0) res[0][3]=a2;
  a14=(a5*a14);
  a10=(a10+a14);
  a5=(a5*a19);
  a10=(a10+a5);
  a18=(a18/a9);
  a17=(a17*a15);
  a11=(a11+a17);
  a11=tan(a11);
  a18=(a18*a11);
  a10=(a10+a18);
  a1=(a1*a10);
  a3=(a3+a1);
  if (res[0]!=0) res[0][4]=a3;
  return 0;
}

int FORCESNLPsolver_dynamics_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f2(arg, res, iw, w, mem);
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

casadi_int FORCESNLPsolver_dynamics_0_n_out(void) { return 1;}

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
    default: return 0;
  }
}

int FORCESNLPsolver_dynamics_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_ddynamics_0:(i0[7],i1[2])->(o0[5x7,21nz]) */
static int casadi_f3(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a4, a5, a6, a7, a8, a9;
  a0=1.6666666666666666e-02;
  a1=2.;
  a2=arg[0]? arg[0][5] : 0;
  a3=5.0000000000000003e-02;
  a4=arg[0]? arg[0][1] : 0;
  a5=(a3*a4);
  a5=(a2+a5);
  a6=arg[0]? arg[0][6] : 0;
  a7=(a3*a4);
  a7=(a2+a7);
  a8=2.5789127999999999e+00;
  a9=(a7/a8);
  a10=arg[0]? arg[0][4] : 0;
  a11=arg[0]? arg[0][0] : 0;
  a12=(a3*a11);
  a12=(a10+a12);
  a13=tan(a12);
  a14=(a9*a13);
  a14=(a3*a14);
  a14=(a6+a14);
  a15=sin(a14);
  a12=cos(a12);
  a12=casadi_sq(a12);
  a16=(a3/a12);
  a16=(a9*a16);
  a17=(a3*a16);
  a18=(a15*a17);
  a18=(a5*a18);
  a18=(a1*a18);
  a19=1.0000000000000001e-01;
  a4=(a19*a4);
  a4=(a2+a4);
  a20=(a5/a8);
  a21=(a3*a11);
  a21=(a10+a21);
  a22=tan(a21);
  a23=(a20*a22);
  a23=(a19*a23);
  a23=(a6+a23);
  a24=sin(a23);
  a21=cos(a21);
  a21=casadi_sq(a21);
  a25=(a3/a21);
  a25=(a20*a25);
  a26=(a19*a25);
  a27=(a24*a26);
  a27=(a4*a27);
  a18=(a18+a27);
  a18=(a0*a18);
  a18=(-a18);
  if (res[0]!=0) res[0][0]=a18;
  a18=cos(a14);
  a17=(a18*a17);
  a17=(a5*a17);
  a17=(a1*a17);
  a27=cos(a23);
  a26=(a27*a26);
  a26=(a4*a26);
  a17=(a17+a26);
  a17=(a0*a17);
  if (res[0]!=0) res[0][1]=a17;
  if (res[0]!=0) res[0][2]=a19;
  a16=(a1*a16);
  a25=(a1*a25);
  a16=(a16+a25);
  a25=(a4/a8);
  a11=(a19*a11);
  a11=(a10+a11);
  a17=cos(a11);
  a17=casadi_sq(a17);
  a26=(a19/a17);
  a26=(a25*a26);
  a16=(a16+a26);
  a16=(a0*a16);
  if (res[0]!=0) res[0][3]=a16;
  a8=(a2/a8);
  a16=tan(a10);
  a26=(a8*a16);
  a26=(a3*a26);
  a26=(a6+a26);
  a28=cos(a26);
  a29=(a3*a28);
  a29=(a1*a29);
  a30=cos(a14);
  a31=(a3*a30);
  a32=1.9388014980576313e-02;
  a33=(a32*a13);
  a34=(a3*a33);
  a35=(a15*a34);
  a35=(a5*a35);
  a31=(a31-a35);
  a31=(a1*a31);
  a29=(a29+a31);
  a31=cos(a23);
  a35=(a19*a31);
  a32=(a32*a22);
  a36=(a19*a32);
  a37=(a24*a36);
  a37=(a4*a37);
  a35=(a35-a37);
  a29=(a29+a35);
  a29=(a0*a29);
  if (res[0]!=0) res[0][4]=a29;
  a29=sin(a26);
  a35=(a3*a29);
  a35=(a1*a35);
  a14=sin(a14);
  a37=(a3*a14);
  a34=(a18*a34);
  a34=(a5*a34);
  a37=(a37+a34);
  a37=(a1*a37);
  a35=(a35+a37);
  a23=sin(a23);
  a37=(a19*a23);
  a36=(a27*a36);
  a36=(a4*a36);
  a37=(a37+a36);
  a35=(a35+a37);
  a35=(a0*a35);
  if (res[0]!=0) res[0][5]=a35;
  if (res[0]!=0) res[0][6]=a19;
  a33=(a1*a33);
  a32=(a1*a32);
  a33=(a33+a32);
  a32=3.8776029961152626e-02;
  a11=tan(a11);
  a32=(a32*a11);
  a33=(a33+a32);
  a33=(a0*a33);
  if (res[0]!=0) res[0][7]=a33;
  a33=1.;
  if (res[0]!=0) res[0][8]=a33;
  if (res[0]!=0) res[0][9]=a33;
  a32=sin(a26);
  a10=cos(a10);
  a10=casadi_sq(a10);
  a8=(a8/a10);
  a10=(a3*a8);
  a35=(a32*a10);
  a35=(a7*a35);
  a35=(a1*a35);
  a9=(a9/a12);
  a12=(a3*a9);
  a37=(a15*a12);
  a37=(a5*a37);
  a37=(a1*a37);
  a35=(a35+a37);
  a20=(a20/a21);
  a21=(a19*a20);
  a37=(a24*a21);
  a37=(a4*a37);
  a35=(a35+a37);
  a35=(a0*a35);
  a35=(-a35);
  if (res[0]!=0) res[0][10]=a35;
  a26=cos(a26);
  a10=(a26*a10);
  a10=(a7*a10);
  a10=(a1*a10);
  a12=(a18*a12);
  a12=(a5*a12);
  a12=(a1*a12);
  a10=(a10+a12);
  a21=(a27*a21);
  a21=(a4*a21);
  a10=(a10+a21);
  a10=(a0*a10);
  if (res[0]!=0) res[0][11]=a10;
  if (res[0]!=0) res[0][12]=a33;
  a9=(a1*a9);
  a8=(a8+a9);
  a20=(a1*a20);
  a8=(a8+a20);
  a25=(a25/a17);
  a8=(a8+a25);
  a8=(a0*a8);
  if (res[0]!=0) res[0][13]=a8;
  a8=cos(a6);
  a25=3.8776029961152625e-01;
  a16=(a25*a16);
  a17=(a3*a16);
  a20=(a32*a17);
  a20=(a7*a20);
  a28=(a28-a20);
  a28=(a1*a28);
  a8=(a8+a28);
  a13=(a25*a13);
  a3=(a3*a13);
  a28=(a15*a3);
  a28=(a5*a28);
  a30=(a30-a28);
  a30=(a1*a30);
  a8=(a8+a30);
  a22=(a25*a22);
  a19=(a19*a22);
  a30=(a24*a19);
  a30=(a4*a30);
  a31=(a31-a30);
  a8=(a8+a31);
  a8=(a0*a8);
  if (res[0]!=0) res[0][14]=a8;
  a8=sin(a6);
  a17=(a26*a17);
  a17=(a7*a17);
  a29=(a29+a17);
  a29=(a1*a29);
  a8=(a8+a29);
  a3=(a18*a3);
  a3=(a5*a3);
  a14=(a14+a3);
  a14=(a1*a14);
  a8=(a8+a14);
  a19=(a27*a19);
  a19=(a4*a19);
  a23=(a23+a19);
  a8=(a8+a23);
  a8=(a0*a8);
  if (res[0]!=0) res[0][15]=a8;
  if (res[0]!=0) res[0][16]=a33;
  a13=(a1*a13);
  a16=(a16+a13);
  a22=(a1*a22);
  a16=(a16+a22);
  a25=(a25*a11);
  a16=(a16+a25);
  a16=(a0*a16);
  if (res[0]!=0) res[0][17]=a16;
  a16=sin(a6);
  a16=(a2*a16);
  a32=(a7*a32);
  a32=(a1*a32);
  a16=(a16+a32);
  a15=(a5*a15);
  a15=(a1*a15);
  a16=(a16+a15);
  a24=(a4*a24);
  a16=(a16+a24);
  a16=(a0*a16);
  a16=(-a16);
  if (res[0]!=0) res[0][18]=a16;
  a6=cos(a6);
  a2=(a2*a6);
  a7=(a7*a26);
  a7=(a1*a7);
  a2=(a2+a7);
  a5=(a5*a18);
  a1=(a1*a5);
  a2=(a2+a1);
  a4=(a4*a27);
  a2=(a2+a4);
  a0=(a0*a2);
  if (res[0]!=0) res[0][19]=a0;
  if (res[0]!=0) res[0][20]=a33;
  return 0;
}

int FORCESNLPsolver_ddynamics_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f3(arg, res, iw, w, mem);
}

int FORCESNLPsolver_ddynamics_0_alloc_mem(void) {
  return 0;
}

int FORCESNLPsolver_ddynamics_0_init_mem(int mem) {
  return 0;
}

void FORCESNLPsolver_ddynamics_0_free_mem(int mem) {
}

int FORCESNLPsolver_ddynamics_0_checkout(void) {
  return 0;
}

void FORCESNLPsolver_ddynamics_0_release(int mem) {
}

void FORCESNLPsolver_ddynamics_0_incref(void) {
}

void FORCESNLPsolver_ddynamics_0_decref(void) {
}

casadi_int FORCESNLPsolver_ddynamics_0_n_in(void) { return 2;}

casadi_int FORCESNLPsolver_ddynamics_0_n_out(void) { return 1;}

casadi_real FORCESNLPsolver_ddynamics_0_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* FORCESNLPsolver_ddynamics_0_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

const char* FORCESNLPsolver_ddynamics_0_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_ddynamics_0_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_ddynamics_0_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s5;
    default: return 0;
  }
}

int FORCESNLPsolver_ddynamics_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_objective_1:(i0[7],i1[2])->(o0) */
static int casadi_f4(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  return 0;
}

int FORCESNLPsolver_objective_1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f4(arg, res, iw, w, mem);
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

casadi_int FORCESNLPsolver_objective_1_n_out(void) { return 1;}

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
    default: return 0;
  }
}

int FORCESNLPsolver_objective_1_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_dobjective_1:(i0[7],i1[2])->(o0[1x7,0nz]) */
static int casadi_f5(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  return 0;
}

int FORCESNLPsolver_dobjective_1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f5(arg, res, iw, w, mem);
}

int FORCESNLPsolver_dobjective_1_alloc_mem(void) {
  return 0;
}

int FORCESNLPsolver_dobjective_1_init_mem(int mem) {
  return 0;
}

void FORCESNLPsolver_dobjective_1_free_mem(int mem) {
}

int FORCESNLPsolver_dobjective_1_checkout(void) {
  return 0;
}

void FORCESNLPsolver_dobjective_1_release(int mem) {
}

void FORCESNLPsolver_dobjective_1_incref(void) {
}

void FORCESNLPsolver_dobjective_1_decref(void) {
}

casadi_int FORCESNLPsolver_dobjective_1_n_in(void) { return 2;}

casadi_int FORCESNLPsolver_dobjective_1_n_out(void) { return 1;}

casadi_real FORCESNLPsolver_dobjective_1_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* FORCESNLPsolver_dobjective_1_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

const char* FORCESNLPsolver_dobjective_1_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_dobjective_1_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_dobjective_1_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

int FORCESNLPsolver_dobjective_1_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
