#include "MTSOS.h"
#include "mex.h"
#include "string.h"
/*This function redirects the barrier, to barrier_mat.m*/

int so_barrier(const double* S, const double* S_prime, const double* S_dprime, const double* b,  const double* a, const double* u, int S_length, int U_size, int State_size, int indicator, double kappa, const double* variables, int variables_length, double* H_barrier, double* G_barrier, double* F_barrier){
    mxArray *prhsBarrier[9], *plhsBarrier[3];
    double* temp;
    char* barrier = "barrier_mat";
    int i, status;
    
    prhsBarrier[0] = mxCreateDoubleMatrix(State_size, S_length, mxREAL);
    memcpy(mxGetPr(prhsBarrier[0]), S, (S_length-1)*State_size*sizeof(double));
    prhsBarrier[1] = mxCreateDoubleMatrix(State_size, S_length, mxREAL);
    memcpy(mxGetPr(prhsBarrier[1]), S_prime, (S_length-1)*State_size*sizeof(double));
    prhsBarrier[2] = mxCreateDoubleMatrix(State_size, S_length, mxREAL);
    memcpy(mxGetPr(prhsBarrier[2]), S_dprime, (S_length-1)*State_size*sizeof(double));
    prhsBarrier[3] = mxCreateDoubleMatrix(S_length-1, 1, mxREAL);
    memcpy(mxGetPr(prhsBarrier[3]), b, (S_length-1)*sizeof(double));
    prhsBarrier[4] = mxCreateDoubleMatrix(S_length-1, 1, mxREAL);
    memcpy(mxGetPr(prhsBarrier[4]), a, (S_length-1)*sizeof(double));
    prhsBarrier[5] = mxCreateDoubleMatrix(U_size, S_length-1, mxREAL);
    memcpy(mxGetPr(prhsBarrier[5]), u, U_size*(S_length-1)*sizeof(double));
    prhsBarrier[6] = mxCreateDoubleMatrix(1, 1, mxREAL);
    temp = mxGetPr(prhsBarrier[6]);
    temp[0] = indicator;
    prhsBarrier[7] = mxCreateDoubleMatrix(1, 1, mxREAL);
    temp = mxGetPr(prhsBarrier[7]);
    temp[0] = kappa;
    prhsBarrier[8] = mxCreateDoubleMatrix(variables_length, 1, mxREAL);
    memcpy(mxGetPr(prhsBarrier[8]), variables, variables_length*sizeof(double));
    status = mexCallMATLAB(3, plhsBarrier, 9, prhsBarrier, barrier);
    
    for(i = 0; i<9;i++)
        mxDestroyArray(prhsBarrier[i]);
    
    if(indicator <= 1)
        memcpy(H_barrier, mxGetPr(plhsBarrier[0]), (2+U_size)*(2+U_size)*(S_length-1)*sizeof(double));
    if(indicator == 0 || indicator == 2)
        memcpy(G_barrier, mxGetPr(plhsBarrier[1]), (2+U_size)*(S_length-1)*sizeof(double));
    if(indicator == 3)
        memcpy(F_barrier, mxGetPr(plhsBarrier[2]), (S_length-1)*sizeof(double));
    
    for(i=1; i<3; i++)
        mxDestroyArray(plhsBarrier[i]);
    
    if(status == 0)
        return 1;
    else
        return 0;
}
