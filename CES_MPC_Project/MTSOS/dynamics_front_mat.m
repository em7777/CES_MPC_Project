function [R M C d] = dynamics_front_mat(S,S_prime,variables)
%The front wheel drive friction circle vehicle dynamics written in matlab.
%Used to run the simulation in MATLAB

m = variables(1);
s = size(S);
theta = atan2(S_prime(1,:),S_prime(2,:));
R = zeros(4*s(2),1);
for i = 1:s(2)
    R((i-1)*4+1:i*4) = [cos(theta(i)); sin(theta(i)); -sin(theta(i)); cos(theta(i))];
end
M = zeros(2,2,s(2));
M(1,1,:) = m*ones(s(2),1);
M(2,2,:) = m*ones(s(2),1);
C = zeros(2,2,s(2));
d = zeros(2,s(2));
end

