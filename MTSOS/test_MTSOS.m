%clear all
close all

m = 1;
u_max = 1;

rand('state',0);
trials = 1;
test_length = 200;
upsample = 40;
L = ceil(test_length/upsample);
X = 0:L;
Y = rand(2,L+1);
out = (0:L*upsample)/upsample;
x = spline(X,Y(1,:),out);
y = spline(X,Y(2,:),out);
data = [x;y];
test_length = length(data);
%to make data consistent with c file.
data = round(data*10000)/10000;

parameters.kappa = 0;
parameters.epsilon = .01;
parameters.alpha = 0;
parameters.beta = 0;
parameters.initial_velocity = 0;
parameters.variables = [m;u_max];
parameters.MAX_ITERATIONS = 1000;
parameters.U_size = 2;

flags.display = 0;
flags.kappa = 1;%1 for dynamic, 0 for fixed
flags.timer = 0;
dynamics = str2func('dynamics_front_mat');
simulate = 1;
[b, u, v, fx, iterate, timed] = MTSOS_mat(data,flags, parameters);


fprintf('The optimal time to traverse is %2.4f\n',fx);
if(abs(fx-8.3287)<1e-3)
    fprintf('MTSOS is working!\n')
else
    fprintf('The optimal traverse time should have been 8.3287\n');
    fprintf('There appear to have been issues with your mex build\n');
    fprintf('See http://www.stanford.edu/~boyd/MTSOS/install.html for more information\n');
end
    
if(simulate)
    fprintf('running simulation of control inputs\n');
    dtheta = 1/(test_length-1);
    S_prime = (data(:,2:end)-data(:,1:end-1))/dtheta;
    %test to see what happens.;
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
    vz1 = S_prime(:,1);%since zero speed doesn't encode orientation
    %note this is a 4th order RTK scheme where as the paper used a 6th
    %order
    for j = 1:test_length-1
        [R M C d] = dynamics(x1,vz1,parameters.variables);
        R = reshape(R,2,2);
        k1v = dt(j)*inv(M)*(R*u(:,j)-C*v1.^2-d);
        k1p = dt(j)*v1;
        [R M C d] = dynamics(x1+k1p/2,vz1+k1v/2,parameters.variables);
        R = reshape(R,2,2);
        k2v = dt(j)*inv(M)*(R*u(:,j)-C*(vz1+k1v/2).^2-d);
        k2p = dt(j)*(v1+k1v/2);
        [R M C d] = dynamics(x1+k2p/2,vz1+k2v/2,parameters.variables);
        R = reshape(R,2,2);
        k3v = dt(j)*inv(M)*(R*u(:,j)-C*(vz1+k2v/2).^2-d);
        k3p = dt(j)*(v1+k2v/2);
        [R M C d] = dynamics(x1+k3p,vz1+k3v,parameters.variables);
        R = reshape(R,2,2);
        k4v = dt(j)*inv(M)*(R*u(:,j)-C*(v1+k3v).^2-d);
        k4p = dt(j)*(v1+k3v);
        x1 = x1+k1p/6+(k2p+k3p)/3+k4p/6;
        v1 = v1+k1v/6+(k2v+k3v)/3+k4v/6;
        vz1 = v1;
        x(:,j+1) = x1;
        vel(:,j+1) = v1;
    end
   
    
    %plotting of a trajectory;
    figure
    plot(data(1,:),data(2,:),'-b+');
    hold on
    plot(x(1,:),x(2,:),'-rx')
    legend('original path','simulated');
end