function [R M C d] = dynamics_mat(S,S_prime,variables)
%dynamics for a friction circle car model, or 2 degree of freedom
%spacecraft.

m = variables(1);
s = size(S);
R = repmat([1; 0; 0; 1],s(2),1);
M = zeros(2,2,s(2));
M(1,1,:) = m*ones(s(2),1);
M(2,2,:) = m*ones(s(2),1);
C = zeros(2,2,s(2));
d = zeros(2,s(2));
end

