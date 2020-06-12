%assuming your Matlab installation is set up correctly to compile mex
%files, this will perform the necessary behavior, setting up
%MTSOS to run for the front wheel drive vehicle.

%to perform with other vehicles change the barrier_front and dyanmics_front
%files to point to files that generate the appropriate dynamic and barrier
%terms.
mex  -compatibleArrayDims -v MTSOS_mat.c MTSOS.c csparse.c barrier_front.c dynamics_front.c -lmwblas

%this line will create the speed optimization program to run with Matlab
%interfaces for the barrier and dynamcis function.  This setup is
%for the friction circle point mass model in 2 dimensions, and should
%provide an optimum time of 7.2609, the test program will claim failure
%with this instance though, as it is configured to verify the front wheel
%drive model.
%mex  -compatibleArrayDims -v MTSOS_mat.c MTSOS.c csparse.c barrier_matlab.c dynamics_matlab.c -lmwblas

test_MTSOS

%Although MTSOS has not been written for 64 bit systems, most users have
%had successs simply switching the -compatibleArrayDims for -largeArrayDims
%flags, although we have not tested the stability of the program to make
%sure this does not introduce errors.

