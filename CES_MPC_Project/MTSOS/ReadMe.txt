More information on MTSOS is available at http://www.stanford.edu/~boyd/software/MTSOS
Version: 1.12

Quick Start:
Running "make" will build test_MTSOS, a simple executable to make sure that everything is configured properly
Running "make test" will both build and run test_MTSOS
In MATLAB:
running setup_MTSOS will build and run the test_MTSOS mex file.

Summary of files:
barrier.c: a barrier function for 2 dof holonic vehicle with single acuator of limited magnitude
barrier_front.c: a barrier function for the front wheel drive friction circle model (used in testing)
barrier_mat.m: a barrier function for 2 dof holonic vehicle with single acuator of limited magnitude, written in MATLAB
barrier_matlab.c: a barrier function to redirect to barrier_mat.m
csparse: Tim Davis's sparse matrix library
dynamics.c: a dynamics function for 2 dof holonic vehicle with single acuator of limited magnitude
dynamics_front.c: a dynamics function for the front wheel drive friction circle model (used in testing)
dyamics_front_mat.m: dynamics for the front wheel drive friction circle model, written in MATLAB
dynamics_mat.m: a dynamics function for 2 dof holonic vehicle with single acuator of limited magnitude written in MATLAB
Makefile: makes the MTSOS executable test_MTSOS
MTSOS.c: the set of subroutines to find the control inputs to achieve the minimum time traversal of a specified path
MTSOS_mat.c: the file to create a MATLAB interface for MTSOS
setup_MTSOS: builds the MATLAB mexfile MTSOS_mat, and runs the testprogram test_MTSOS.m
test_MTSOS.c: A c function to test MTSOS, built my the Makefile
test_MTSOS.m: A function to test MTSOS_mat
