function [newP, vel] = CES(P,A,r)
R_min = 1; %min turning radius
m = 1; %mass
u_max = 1; %max control (= mu * mass * g)
initial_vel = .1;

%Speed Optimization%
parameters.kappa = 0;
parameters.epsilon = .1;
parameters.alpha = 0;
parameters.beta = 0;
parameters.initial_velocity = initial_vel;
parameters.variables = [m;u_max];
parameters.MAX_ITERATIONS = 1000;
parameters.U_size = 2;

flags.display = 0;
flags.kappa = 1;
flags.timer = 0;

data = P';
[b u v fx iterate timed] = MTSOS_mat(data,flags, parameters);

test_length = size(P,1);
dtheta = 1/(test_length-1);
S_prime = (data(:,2:end)-data(:,1:end-1))/dtheta;
S_middle = (data(:,2:end)+data(:,1:end-1))/2;
b_s = sqrt(b);
dt = 2*dtheta./(b_s(1:end-1)+b_s(2:end));
x = zeros(2,test_length);
vel = zeros(2,test_length);
x(:,1) = data(:,1);
v_init = S_prime(:,1)/norm(S_prime(:,1))*parameters.initial_velocity;
vel(:,1) = v_init;
v1 = v_init;
x1 = data(:,1);
vz1 = S_prime(:,1);

for j = 1:test_length-1
    [R M C d] = dynamics_front_mat(x1,vz1,parameters.variables);
    R = reshape(R,2,2);
    k1v = dt(j)*inv(M)*(R*u(:,j)-C*v1.^2-d);
    k1p = dt(j)*v1;
    [R M C d] = dynamics_front_mat(x1+k1p/2,vz1+k1v/2,parameters.variables);
    R = reshape(R,2,2);
    k2v = dt(j)*inv(M)*(R*u(:,j)-C*(vz1+k1v/2).^2-d);
    k2p = dt(j)*(v1+k1v/2);
    [R M C d] = dynamics_front_mat(x1+k2p/2,vz1+k2v/2,parameters.variables);
    R = reshape(R,2,2);
    k3v = dt(j)*inv(M)*(R*u(:,j)-C*(vz1+k2v/2).^2-d);
    k3p = dt(j)*(v1+k2v/2);
    [R M C d] = dynamics_front_mat(x1+k3p,vz1+k3v,parameters.variables);
    R = reshape(R,2,2);
    k4v = dt(j)*inv(M)*(R*u(:,j)-C*(v1+k3v).^2-d);
    k4p = dt(j)*(v1+k3v);
    x1 = x1+k1p/6+(k2p+k3p)/3+k4p/6;
    v1 = v1+k1v/6+(k2v+k3v)/3+k4v/6;
    vz1 = v1;
    x(:,j+1) = x1;
    vel(:,j+1) = v1;
end
vel = vel';

%Path Optimization
d = 0;
n = length(P);
for i = 2 : 1 : n
    d = d + norm(P(i,:) - P(i-1,:), 2);
end
d = d * 1./(n-1);

cvx_begin
    variables Q(n,2)
    Q(1,:) == P(1,:);
    Q(2,:) == P(1,:) + d .* vel(1,:) / norm(vel(1,:));
    Q(n,:) == P(n,:);
    Q(n-1,:) == P(n,:) - d .* vel(n-1,:) / norm(vel(n-1,:));
    sum = 0;
    
    for k = 3 : n-2
         norm(Q(k, :) - A(k, :), 2) <= r(k);
    end
    
    for k = 2 : n-1
        alpha_k = sqrt((u_max / m) ^2 - (u(1,k) / m) ^2);
        friction_constr = alpha_k * (d / norm(vel(k), 2)).^2;
        radius_constr = d.^2 ./ R_min;
        
        norm(2.* Q(k, :) - Q(k-1, :) - Q(k+1, :), 2) <= min(friction_constr, radius_constr);
        sum = sum + pow_pos(norm(2.* Q(k, :) - Q(k-1, :) - Q(k+1, :), 2),2);
    end
    minimize sum
cvx_end

newP = Q;
fx
end