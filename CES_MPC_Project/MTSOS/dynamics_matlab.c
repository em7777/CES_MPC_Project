#include "MTSOS.h"
#include "mex.h"
#include "string.h"

/*This code redirects the call to dynamics to dynamics_mat.m*/
int so_dynamics(const double* const S_middle, const double* const S_prime, int S_length, int State_size, int U_size, const double* variables, int variables_length, double* R_dynamics, double* M_dynamics, double* C_dynamics, double* d_dynamics){
    mxArray *prhsDynamics[3], *plhsDynamics[4];
    double *data;
    char* dynamics = "dynamics_front_mat";//change to the name of your file.
    int block_length, status;
    
    //The Arrays must be copied into Matlab format;
    prhsDynamics[0] = mxCreateDoubleMatrix(State_size,S_length-1,mxREAL);
    data = mxGetPr(prhsDynamics[0]);
    memcpy(data,S_middle,(S_length-1)*State_size*sizeof(double));
    prhsDynamics[1] = mxCreateDoubleMatrix(State_size,S_length-1,mxREAL);
    data = mxGetPr(prhsDynamics[1]);
    memcpy(data,S_prime,(S_length-1)*State_size*sizeof(double));
    prhsDynamics[2] = mxCreateDoubleMatrix(variables_length,1,mxREAL);
    memcpy(mxGetPr(prhsDynamics[2]),variables,sizeof(double)*variables_length);
    //This will call the dynamic generator as mexfile or matlab function
    status = mexCallMATLAB(4, plhsDynamics, 3, prhsDynamics, dynamics);
    mxDestroyArray(prhsDynamics[0]);
    mxDestroyArray(prhsDynamics[1]);
    mxDestroyArray(prhsDynamics[2]);
    
    block_length = mxGetM(plhsDynamics[0])/(S_length-1);
    
    //copy the results into the necessary matrices to return.
    data = mxGetPr(plhsDynamics[0]);
    memcpy(R_dynamics,data,(S_length-1)*block_length*sizeof(double));
    
    data = mxGetPr(plhsDynamics[1]);
    memcpy(M_dynamics,data,(S_length-1)*State_size*State_size*sizeof(double));
    
    data = mxGetPr(plhsDynamics[2]);
    memcpy(C_dynamics,data,(S_length-1)*State_size*State_size*sizeof(double));
    
    data = mxGetPr(plhsDynamics[3]);
    memcpy(d_dynamics,data,(S_length-1)*State_size*sizeof(double));
    
    mxDestroyArray(plhsDynamics[0]);
    mxDestroyArray(plhsDynamics[1]);
    mxDestroyArray(plhsDynamics[2]);
    mxDestroyArray(plhsDynamics[3]);
    if(status == 0)
        return 1;
    else
        return 0;
}
