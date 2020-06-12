function [H G F] = barrier(S, S_prime, S_dprime, b, a, u,indicator,kappa,variables)
%our default barrier function is going to be (u1^2+u2^2) < C;
%that is a friction circle car model.
%G is the gradient of the barrier function and H is the Hessian matrix of
%the barrier function.  Remember than in our formulation, these constraints
%are enforced point to point, so this applies to a specific point.  note
%that the variable order should be (b1, a1, u1, u2);
%if the indicator = 0, then just H and G are needed
%if indicator = 1, then just H is needed
%if indicator = 2, just G is needed;
%if indicator = 3, just F is needed;
C = variables(2);
fx = u(1,:).^2+u(2,:).^2-C;
temp = size(u);
U_size = temp(1);
S_length = temp(2);%really S_length-1
H = 0;
G = 0;
F = 0;
if(sum(fx > 0) >0)
    G = inf;
    F = inf;
elseif(indicator < 3)
    Df = [2*u(1,:);2*u(2,:)];
    if(indicator < 2)
        H = zeros(2+U_size,2+U_size,S_length);
        for i = 1:S_length
            H(3:4,3:4,i) = kappa*(1/fx(i)^2*(Df(:,i)*Df(:,i)')+1/-fx(i)*[2 0; 0 2]);
        end
    end
    if(indicator == 0 || indicator ==2)
        G = zeros(2+U_size,S_length);
        G(3:4,:) = -Df./[fx;fx]*kappa;
    end
elseif(indicator == 3)
    F = -kappa*log(-fx);
end
