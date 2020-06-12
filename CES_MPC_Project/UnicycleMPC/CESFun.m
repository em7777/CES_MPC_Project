function dxdt = CESFun(x, u)
% State equations of the unicycle model robot.
%
% States:
%   x(1)  x inertial coordinate of center of mass
%   x(2)  y inertial coordinate of center of mass
%   x(3)  theta, thrust direction

% Inputs:
%   u(1), u(2) velocity and omega
% Parameters

% Variables
theta = x(3);
V = u(1);
omega = u(2);

% State equations
dxdt = zeros(3,1);
dxdt(1) = V*cos(theta);
dxdt(2) = V*sin(theta);
dxdt(3) = omega;

