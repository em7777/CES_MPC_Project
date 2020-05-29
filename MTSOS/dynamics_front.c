#include "MTSOS.h"
#include <math.h>

/*
 *Implementation of dynamics for the friction circle car (with front wheel drive)
 *although front wheel drive has no impact on the dynamics, only on the constraint set.
 *return 1 if everything goes smoothly, zero otherwise
 */
int so_dynamics(const double* const S_middle, const double* const S_prime, int S_length, int State_size, int U_size, const double* variables, int variables_length, double* R_dynamics, double* M_dynamics, double* C_dynamics, double* d_dynamics){
    double m, theta;
    int i,block_length;
    
    m = variables[0];
    block_length = U_size*State_size;
    for(i=0;i<(S_length-1);i++){
        theta = atan2(S_prime[State_size*i],S_prime[State_size*i+1]);
        R_dynamics[i*block_length]=cos(theta);
        R_dynamics[i*block_length+1]=sin(theta);
        R_dynamics[i*block_length+2]=-sin(theta);
        R_dynamics[i*block_length+3]=cos(theta);
    }
    
    block_length = State_size*State_size;
    for(i=0;i<(S_length-1);i++){
        M_dynamics[i*block_length]=m;
        M_dynamics[i*block_length+1]=0;
        M_dynamics[i*block_length+2]=0;
        M_dynamics[i*block_length+3]=m;
    }
    
    for(i=0;i<(S_length-1)*State_size*State_size;i++)
        C_dynamics[i] = 0;
    
    for(i=0;i<(S_length-1)*State_size;i++)
        d_dynamics[i] = 0;
    
    return 1;
}
