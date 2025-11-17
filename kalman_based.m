close all
clc
clear all
%% Initialzation 
dock=[0 0 0]'; % dock potition

kpx=0.5;
kpy=0.2;
kpz=0.3;
kptheta=1;
kppsi=1.5;

auvtrue0=[-20 -30 -50 pi/4 pi/3]'; % auv initial position true [x y z psi theta]
auvhat0=auvtrue0+[2; 3; 5; 0.2; 0.4].*rand(5,1); % auv initial noisy

%% Noise covariance
Qw_true=diag([0.01 0.01 0.01 0.001 0.001]);  % affects plant

% EKF process noise 
Q=diag([1e-4 1e-4 1e-4 1e-5 1e-5]);

% Measurement noise: y = [x; y; z; psi; theta]
R=diag([0.005 0.005 0.005 0.0002 0.0002]);


%% ODE setup

dt=0.05;
T=60;
N=T/dt;
time=(0:N-1)*dt;

%% storage for variables

X_true_hist =zeros(5,N);
X_hat_hist=zeros(5,N);
Y_meas_hist=zeros(5,N);
P_diag=zeros(5,N);  % std devs for each state
psi_d_hist=zeros(1,N);
theta_d_hist=zeros(1,N);
vel_true_hist=zeros(5, N);   % store [u; v; w; psi_dot; theta_dot]
vel_hat_hist=zeros(5, N);

%% initialize for filter

X_true=auvtrue0;
X_hat=auvhat0;
P=eye(5);

%% main loop
for i=1:N
    %true states
    [f_true, vel_true]=controllor(X_true,dock,kpx,kpy,kpz,kppsi,kptheta);
    vel_true_hist(:,i)=vel_true;
    
    w=mvnrnd(zeros(5,1),Qw_true)';

    X_true=X_true+dt*(f_true+w);

    % measurements with noise
    v=mvnrnd(zeros(5,1),R)';
    Y_meas=X_true+v;

    % EKF prediction
    [f_hat, vel_hat]=controllor(X_hat,dock,kpx,kpy,kpz,kppsi,kptheta);
    vel_hat_hist(:,i)=vel_hat;
    
    X_pred=X_hat+dt*(f_hat);

    % jacobian
    A=numerical_jacobian(@(x) controllor(x,dock,kpx,kpy,kpz,kppsi,kptheta),X_hat);
    F=eye(5)+dt*A;

    % covariance predicton
    P_pred=F*P*F'+Q;

    % update
    H=eye(5);
    y_pred=X_pred;

    S=H*P_pred*H'+R;
    Kk=P_pred*H'/S;

    X_hat=X_pred+Kk*(Y_meas-y_pred);
    P=(eye(5)-Kk*H)*P_pred;

    % log data
    P_diag(:,i)=sqrt(diag(P));

    X_true_hist(:,i)=X_true;
    X_hat_hist(:,i)=X_hat;
    Y_meas_hist(:,i)=Y_meas;
    
    % also log desired LOS angles for plotting (from TRUE state)
    ex = dock(1)-X_true(1);
    ey = dock(2)-X_true(2);
    ez = dock(3)-X_true(3);
    psi_d_hist(i)=atan2(ey, ex);
    dist_xy= sqrt(ex^2 + ey^2);
    theta_d_hist(i)=atan2(-ez, dist_xy);

end

%% Additional processing
x=X_true_hist(1,:)';
y=X_true_hist(2,:)';
z=X_true_hist(3,:)';
psi=X_true_hist(4,:)';
theta=X_true_hist(5,:)';

dist_true=sqrt((x-dock(1)).^2+(x-dock(2)).^2+(x-dock(3)).^2);
dist_filter=sqrt((X_hat_hist(1,:)-dock(1)).^2+(X_hat_hist(2,:)-dock(2)).^2+(X_hat_hist(3,:)-dock(3)).^2);

% Estimation errors
est_err=X_true_hist-X_hat_hist;
sigma3=3*P_diag;

%% Plots

%3D true
figure(1);
plot3(x, y, z, 'b-', 'LineWidth', 2); hold on;
plot3(dock(1), dock(2), dock(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot3(auvtrue0(1), auvtrue0(2), auvtrue0(3), 'k*', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('AUV Trajectory (true)', 'Dock', 'Initial position');
title('3D docking simulation');

%3D filtered
figure(2);
plot3(x, y, z, 'b-', 'LineWidth', 2); hold on;
plot3(dock(1), dock(2), dock(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot3(auvtrue0(1), auvtrue0(2), auvtrue0(3), 'k*', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('AUV Trajectory (filtered)', 'Dock', 'Initial position');
title('3D docking simulation');

% distance between AUV and Dock
figure(3);
plot(time, dist_true, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Distance to Dock [m]');
title('True Distance vs Time');
grid on;

% distance between AUV and Dock
figure(4);
plot(time, dist_filter, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Distance to Dock [m]');
title('Filtered Distance vs Time');
grid on;

% Z error
figure(5);
plot(time, dock(3) - z, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('z error [m]');
title('Vertical Error (z_d - z)');
grid on;

% speed plots
figure(6);
plot(time, vel_true_hist(1,:), 'LineWidth', 2);
hold on;
plot(time, vel_hat_hist(1,:), '--', 'LineWidth', 2);
xlabel("Time [s]");
ylabel("Surge u [m/s]");
legend("True","Estimated");
title("Surge Velocity");
grid on;

% speed plots
figure(7);
plot(time, vel_true_hist(2,:), 'LineWidth', 2);
hold on;
plot(time, vel_hat_hist(2,:), '--', 'LineWidth', 2);
xlabel("Time [s]");
ylabel("Sway v [m/s]");
legend("True","Estimated");
title("Sway Velocity");
grid on;

% speed plots
figure(8);
plot(time, vel_true_hist(3,:), 'LineWidth', 2);
hold on;
plot(time, vel_hat_hist(3,:), '--', 'LineWidth', 2);
xlabel("Time [s]");
ylabel("Heave w [m/s]");
legend("True","Estimated");
title("Heave Velocity");
grid on;

% estimation plots
state_names = {'x','y','z','\psi','\theta'};
for i = 1:5
    figure('Name',['3-sigma check for ' state_names{i}]);
    plot(time, est_err(i,:), 'k','LineWidth',1.5); hold on;
    plot(time,  sigma3(i,:), 'r--','LineWidth',1.2);
    plot(time, -sigma3(i,:), 'r--','LineWidth',1.2);
    xlabel('Time (s)');
    ylabel([state_names{i} ' error']);
    title(['Estimation error vs 3\sigma bound for ' state_names{i}]);
    legend('error','+3\sigma','-3\sigma');
    grid on;
end

%% Controllor
function [dx, vel] = controllor(x, dock, kpx,kpy,kpz,kppsi,kptheta)
    % x = [X; Y; Z; psi; theta]

    X=x(1);
    Y=x(2);
    Z=x(3);
    psi=x(4);
    theta=x(5);

    % Position errors
    ex=dock(1)-X;
    ey=dock(2)-Y;
    ez=dock(3)-Z;

    % LOS-based desired angles
    psi_d=atan2(ey, ex);
    dist_xy=sqrt(ex^2 + ey^2);
    theta_d=atan2(-ez, dist_xy);

    % Body-frame velocity commands 
    u=kpx*(cos(psi)*ex+sin(psi)*ey);    
    v=kpy*(-sin(psi)*ex+cos(psi)*ey);   
    w=kpz*ez;                      

    % Kinematics
    dx = zeros(5,1);
    dx(1)=u*cos(psi)*cos(theta)-v*sin(psi)+w*cos(psi)*sin(theta);  
    dx(2)=u*sin(psi)*cos(theta)+v*cos(psi)+w*sin(psi)*sin(theta); 
    dx(3)=-u*sin(theta)+w*cos(theta);                                
    dx(4)=kppsi*pi_to_pi(psi_d-psi);                             
    dx(5)=kptheta*pi_to_pi(theta_d-theta); 

    vel=[u;v;w;kppsi*pi_to_pi(psi_d-psi);kptheta*pi_to_pi(theta_d-theta)];
end

%% Jacobian function
function A = numerical_jacobian(fun, x)
    n=length(x);
    fx=fun(x);
    A=zeros(n,n);
    epsJ=1e-5;
    for i=1:n
        dx=zeros(n,1);
        dx(i)=epsJ;
        f_plus=fun(x + dx);
        f_minus=fun(x - dx);
        A(:,i)=(f_plus - f_minus)/(2*epsJ);
    end
end
%% additional func
function x=pi_to_pi(x)
        x=mod(x,2*pi);
        if x>=pi
            x=x-2*pi;
        end
        
    end