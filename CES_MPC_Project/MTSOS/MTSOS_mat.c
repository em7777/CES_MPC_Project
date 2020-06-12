#include "mex.h"
#include <string.h>
#include <math.h>
#include "csparse.h"
#include "MTSOS.h"
#include <time.h>

void mexFunction(int nlhs, mxArray *plhs[],
    int nrhs, const mxArray *prhs[]) {
    //inputs S [0], flags [1], parameters [2]
    //outputs, b [0], u [1], v [2], fx [3], interations timers[4]
    mxArray* temp;
    double* data;
    algorithm_params a_params;
    problem_params p_params;
    algorithm_flags a_flags;
    optional_params o_params;
    double *p_b, *p_u, *p_v, *p_timers;
    double fx;
    int iterations, status;
    
    p_b = NULL;
    p_u = NULL;
    p_v = NULL;
    p_timers = NULL;
    
    if(nrhs != 3){
        mexErrMsgTxt("Wrong Number of Inputs");
    }
    if(nlhs != 6){
        mexErrMsgTxt("Wrong Number of Outputs");
    }
    
    temp = mxGetField(prhs[1],0,"timer");    
    if(temp != NULL){
            data = mxGetPr(temp);
            a_flags.timer = (int)data[0];
    }else
            a_flags.timer = 0;
    
    //determine kappa, for the barrier function;
    temp = mxGetField(prhs[2],0,"kappa");
    if(temp!=NULL){
        data = mxGetPr(temp);
        a_params.kappa = data[0];
    }else
        a_params.kappa = -1;
    
    //determine the backtracking line search parameter alpha if set
    temp = mxGetField(prhs[2],0,"alpha");
    if(temp!=NULL){
        data = mxGetPr(temp);
        a_params.alpha = data[0];
    }else{
        a_params.alpha = -1;
    }
    
    //determine the backtracking line searh parameter beta if set
    temp = mxGetField(prhs[2],0,"beta");
    if(temp==NULL)
        a_params.beta = -1;
    else{
        data = mxGetPr(temp);
        a_params.beta = data[0];
    }
    
    //determine the espilon, accuracy term
    temp = mxGetField(prhs[2],0,"epsilon");
    if(temp==NULL)
        a_params.epsilon = -1;
    else{
        data = mxGetPr(temp);
        a_params.epsilon = data[0];
    }
    
    temp = mxGetField(prhs[2],0,"MAX_ITERATIONS");
    if(temp == NULL)
        a_params.MAX_ITER = -1;
    else{
        data = mxGetPr(temp);
        a_params.MAX_ITER = (int)data[0];
    }
    
    temp = mxGetField(prhs[2],0,"variables");
    if(temp==NULL){
        o_params.variables = NULL;
        o_params.variables_length = 0;
    }else{
        o_params.variables = mxGetPr(temp);
        o_params.variables_length = (int)mxGetM(temp)*mxGetN(temp);//in this way a row vector and a column vector will both be accomodated.
    }
    
    temp = mxGetField(prhs[2], 0, "U_size");
    if(temp==NULL){
        mexErrMsgTxt("Number of control inputs as parameter.U_size not provided");
    } else{
        data = mxGetPr(temp);
        p_params.U_size = (int)data[0];
    }
    
    p_params.S_length = (int)mxGetN(prhs[0]);
    p_params.State_size = (int)mxGetM(prhs[0]);
    p_params.S = mxGetPr(prhs[0]);
    //mexPrintf("S_length = %d\nState_size = %d\n",p_params.S_length,p_params.State_size);
    
    temp = mxGetField(prhs[1],0,"kappa");
    if(temp==NULL)
        a_flags.kappa = 1;
    else{
        data = mxGetPr(temp);
        a_flags.kappa = (int)data[0];
    }

    temp = mxGetField(prhs[1],0,"display");
    if(temp == NULL)
        a_flags.display = 0;
    else{
        data = mxGetPr(temp);
        a_flags.display = (int)data[0];
    }
    
    temp = mxGetField(prhs[2],0,"initial_velocity");
    if(temp == NULL)
        p_params.initial_velocity = 0;
    else{
        data = mxGetPr(temp);
        p_params.initial_velocity = data[0];
    }
    
    //check if an initial b was provided
    temp = mxGetField(prhs[2], 0, "initial_b");
    if(temp == NULL)
        o_params.initial_b = NULL;
    else{
        if(mxGetM(temp)*mxGetN(temp)!=p_params.S_length){
            #ifdef MATLAB_MEX_FILE
                    mexErrMsgTxt("Initial b vector provided is the wrong length");
            #endif
            #ifndef MATLAB_MEX_FILE
                    fprintf(stderr, "Initial b vector provided is the wrong length\n");
            #endif
        }else{
            data = mxGetPr(temp);
            o_params.initial_b = data;
        }
    }
    
    //check if an initial u was provided    
    temp = mxGetField(prhs[2], 0, "initial_u");//If there is no initial u, then the control initizalizes to zero.
    if(temp==NULL)
        o_params.initial_u = NULL;
    else{
        if(mxGetM(temp)!=p_params.U_size || mxGetN(temp) != p_params.S_length-1){
            #ifdef MATLAB_MEX_FILE
                    mexErrMsgTxt("the initial u matrix provided is the wrong size");
            #endif
            #ifndef MATLAB_MEX_FILE
                    fprintf(stderr, "the initial u matrix provided is the wrong size\n");
            #endif
        }
        data = mxGetPr(temp);
        o_params.initial_u = data;
    }
    status = so_MTSOS(&p_params, &a_flags, &a_params, &o_params, &p_b, &p_u, &p_v, &fx, &p_timers, &iterations);
    
    //Set all of the values to be returned
    if(status == 1){
        plhs[0] = mxCreateDoubleMatrix(p_params.S_length, 1, mxREAL);
        memcpy(mxGetPr(plhs[0]), p_b, p_params.S_length*sizeof(double));
        mxFree(p_b);
        plhs[1] = mxCreateDoubleMatrix(p_params.U_size, p_params.S_length-1, mxREAL);
        memcpy(mxGetPr(plhs[1]), p_u, p_params.U_size*(p_params.S_length-1)*sizeof(double));
        mxFree(p_u);
        plhs[2] = mxCreateDoubleMatrix((p_params.S_length-1)*(p_params.State_size+1), 1, mxREAL);
        memcpy(mxGetPr(plhs[2]), p_v, (p_params.S_length-1)*(p_params.State_size+1)*sizeof(double));
        mxFree(p_v);
        plhs[3] = mxCreateDoubleMatrix(1, 1, mxREAL);
        data = mxGetPr(plhs[3]);
        data[0] = fx;
        plhs[4] = mxCreateDoubleMatrix(1, 1, mxREAL);
        data = mxGetPr(plhs[4]);
        data[0] = iterations;
        if(nlhs == 6){
            if(a_flags.timer==1){
                plhs[5] = mxCreateDoubleMatrix(19,1,mxREAL);
                memcpy(mxGetPr(plhs[5]),p_timers,19*sizeof(double));
                if(p_timers != NULL)
                    mxFree(p_timers);
            }else{
                plhs[5] = mxCreateDoubleMatrix(1,1,mxREAL);
                data = mxGetPr(plhs[5]);
                data[0] = -1;
            }
        }
    }else{
        plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
        data = mxGetPr(plhs[0]);
        data[0] = -1;
        if(p_b != NULL)
            mxFree(p_b);
        plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
        data = mxGetPr(plhs[1]);
        data[0] = -1;
        if(p_u != NULL)
            mxFree(p_u);
        plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL);
        data = mxGetPr(plhs[2]);
        data[0] = -1;
        if(p_v != NULL)
            mxFree(p_v);
        plhs[3] = mxCreateDoubleMatrix(1, 1, mxREAL);
        data = mxGetPr(plhs[3]);
        data[0] = -1;
        plhs[4] = mxCreateDoubleMatrix(1, 1, mxREAL);
        data = mxGetPr(plhs[4]);
        data[0] = -1;
        if(nlhs == 6){
            plhs[5] = mxCreateDoubleMatrix(1, 1, mxREAL);
            data = mxGetPr(plhs[5]);
            data[0] = -1;
        }
    }
}
