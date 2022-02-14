import numpy
import ctypes

name = "FORCESNLPsolver"
requires_callback = True
lib = "lib/libFORCESNLPsolver.so"
lib_static = "lib/libFORCESNLPsolver.a"
c_header = "include/FORCESNLPsolver.h"
nstages = 10

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, ( 70,   1),   70),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (100,   1),  100),
 ("reinitialize"        , ""      , "FORCESNLPsolver_int", ctypes.c_int   , numpy.int32  , (  0,   1),    1)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7)]

# Info Struct Fields
info = \
[("it", ctypes.c_int),
 ("res_eq", ctypes.c_double),
 ("rsnorm", ctypes.c_double),
 ("pobj", ctypes.c_double),
 ("solvetime", ctypes.c_double),
 ("fevalstime", ctypes.c_double),
 ("QPtime", ctypes.c_double)]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(7, 5, 9, 10, 4, 4, 9, 0), 
	(7, 5, 9, 10, 4, 4, 9, 0), 
	(7, 5, 9, 10, 4, 4, 9, 0), 
	(7, 5, 9, 10, 4, 4, 9, 0), 
	(7, 5, 9, 10, 4, 4, 9, 0), 
	(7, 5, 9, 10, 4, 4, 9, 0), 
	(7, 5, 9, 10, 4, 4, 9, 0), 
	(7, 5, 9, 10, 4, 4, 9, 0), 
	(7, 5, 9, 10, 4, 4, 9, 0), 
	(7, 5, 9, 10, 4, 4, 9, 0)
]