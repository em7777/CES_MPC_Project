#include <math.h>
#include "MTSOS.h"
#include "mex.h"
#include "blas.h"
/*a barrier for a two degree freedom space craft model with maximum permitted force*/

int so_barrier(const double* S, const double* S_prime, const double* S_dprime, const double* b,  const double* a, const double* u, int S_length, int U_size, int State_size, int indicator, double kappa, const double* variables, int variables_length, double* H_barrier, double* G_barrier, double* F_barrier){
    //F, G, and H are all passed in as NULL pointers;
    double *fx, *Df, *A;
    double alpha, C;
    int valid, block_length;
    char* uplo = "U";//used for the blas call to dsyr.
    ptrdiff_t one, u_size;
    int i;
    
    one = 1;
    u_size = U_size;
    //pull the terms needed that are stored in variables;
    C = variables[1];
    
    //calculate the value of f(x)-c for a constraint f(x) < c;
    fx = mxMalloc((S_length-1)*sizeof(double));
    
    valid = 1;//a parameter to ensure that the inequality constraint is always met.
    for(i=0;i<(S_length-1)&&valid == 1;i++){
        fx[i] = u[i*U_size]*u[i*U_size]+u[i*U_size+1]*u[i*U_size+1]-C;
        if(fx[i]>0)
            valid = 0;
    }
    
    //if the constraints are violated, then the NULL pointers are returned;
    if(valid != 0){
        if(indicator < 3){
            //calculate the gradient of your limiting function;
            Df = mxMalloc(U_size*(S_length-1)*sizeof(double));
            for(i=0;i<(S_length-1);i++){
                Df[i*U_size] = 2*u[i*U_size];
                Df[i*U_size+1] = 2*u[i*U_size+1];
            }
            if(indicator < 2){
                //Allocate memory to store an array of Hessians(the first (2+U_size)*(2+U_size) block refers to the first Hessian)
                block_length = (2+U_size)*(2+U_size);//the size of one Hessian block
                //zero the allocated memory (for speed up this could be incorporated into the allocation loop as it is with G.
                for(i=0;i<(2+U_size)*(2+U_size)*(S_length-1);i++)
                    H_barrier[i] = 0;
                //calculate the Hessian: kappa*(1/fx(i)^2*(Df(:,i)*Df(:,i)')+1/-fx(i)*[2 0; 0 2])
                //by using the blas syr command which calculates alpha*x*x'+A where alpha = kappa/fx(i)^2,
                //x = Df(:,i), and A = 1/-fx(i)*[2 0; 0 2].
                A = mxMalloc(4*sizeof(double));
                A[1] = 0;
                for(i=0;i<(S_length-1);i++){
                    //Df*Df';
                    alpha = 1.0/(fx[i]*fx[i]);
                    A[0] = -2.0/fx[i];
                    A[2] = 0;//this needs to be reset each time because it gets overwritten by dsyr.
                    A[3] = -2.0/fx[i];
                    /*a note on dsyr(uplo,N,alpha,x,incr,A,lda)
                     *uplo: dsyr assumes A is symmetric and only uses either the upper "U" or lower "U" triangular part.
                     *N: the length of x
                     *incr: x(i) can be found at x(i*incr);
                     *lda: A(i,j) can be found at A(i+lda*j);
                     */
                    dsyr(uplo, &U_size, &alpha, &Df[i*U_size], &one, A, &U_size);
                    H_barrier[i*block_length+(2+U_size)*2+2] = kappa*A[0];
                    H_barrier[i*block_length+(2+U_size)*2+3] = kappa*A[2];//note these are both 2, because the result is stored in the upper triangular portion.
                    H_barrier[i*block_length+(2+U_size)*3+2] = kappa*A[2];
                    H_barrier[i*block_length+(2+U_size)*3+3] = kappa*A[3];
                }
                mxFree(A);
            }
            if(indicator == 0 || indicator ==2){
                for(i=0;i<S_length-1;i++){
                    G_barrier[(2+U_size)*i] = 0;//zeroing the allocated memory
                    G_barrier[(2+U_size)*i+1] = 0;
                    G_barrier[(2+U_size)*i+2]=-kappa*Df[U_size*i]/fx[i];
                    G_barrier[(2+U_size)*i+3]=-kappa*Df[U_size*i+1]/fx[i];
                }
            }
            mxFree(Df);
        }else{
            for(i=0;i<S_length-1;i++)
                F_barrier[i] = -kappa*log(-fx[i]);
        }
    }
    mxFree(fx);
    return valid;
}
