function [A, B] = CESJacobianFun(x, u)
% Jacobian of model equations 
%
% States:
%   x(1)  x inertial coordinate of center of mass
%   x(2)  y inertial coordinate of center of mass
%   x(3)  theta, thrust direction

%
% Inputs:
%   u(1), u(2), Velocity and Angular Velocity

% Parameters

% Variables
theta = x(3);
V = u(1);
omega = u(2);

% Linearize the state equations at the current condition
A = zeros(3,3);
A(1,3) = -V*sin(theta);
A(2,3) = V*cos(theta);

B = zeros(3,2);

B(1,1) = cos(theta);
B(1,2) = -V*sin(theta);
B(2,1) = sin(theta);
B(2,2) = V*cos(theta);
B(3,1) = 0;
B(3,2) = 1;
