#include "MTSOS.h"
#include "mex.h"
/*dynamics for a friction circle car, or 2 degree of freedom spacecraft model*/


int so_dynamics(const double* const S_middle, const double* const S_prime, int S_length, int State_size, int U_size, double* variables, int variables_length, double* R_dynamics, double* M_dynamics, double* C_dynamics, double* d_dynamics){
    double m;
    int i,block_length;
    
    m = variables[0];
    block_length = U_size*State_size;
    for(i=0;i<(S_length-1);i++){
        R_dynamics[i*block_length]=1;
        R_dynamics[i*block_length+1]=0;
        R_dynamics[i*block_length+2]=0;
        R_dynamics[i*block_length+3]=1;
    }
    
    block_length = State_size*State_size;
    for(i=0;i<(S_length-1);i++){
        M_dynamics[i*block_length]=1;
        M_dynamics[i*block_length+1]=0;
        M_dynamics[i*block_length+2]=0;
        M_dynamics[i*block_length+3]=1;
    }
    
    for(i=0;i<(S_length-1)*State_size*State_size;i++)
        C_dynamics[i] = 0;
    
    for(i=0;i<(S_length-1)*State_size;i++)
        d_dynamics[i] = 0;
    
    return 1;
}
