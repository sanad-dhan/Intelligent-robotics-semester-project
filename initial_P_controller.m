close all
clear all
clc

%% AUV initialization

dock=[5 10 0]'; % docker position

Kp_x=0.5;  % surge
Kp_y=0.2;  % sway
Kp_z=0.3;  % heave
Kp_psi=0.8;  % yaw
Kp_theta=1; % pitch

x0= [0 -25 -30 pi/3 pi/6]'; % AUV initials [x,y,z,psi,theta] psi=head, theta=pitch



%% ODE solver
tStart=0;
tEnd=30;
dt=0.1; %  step size
tRange = tStart:dt:tEnd;
[tsol, Ysol]=ode45(@(t,y) controllor(t,y,Kp_psi,Kp_theta,Kp_x,Kp_y,Kp_z,dock),tRange,x0);

%% state extract
x=Ysol(:,1);
y=Ysol(:,2);
z=Ysol(:,3);
psi=Ysol(:,4);
theta=Ysol(:,5);

%% Additional info for plots
N=length(tsol);
psi_d=zeros(N,1); theta_d=zeros(N,1);
for i=1:N
    ex=dock(1)-x(i); 
    ey=dock(2)-y(i);
    ez=dock(3)-z(i);
    psi_d(i)=atan2(ey, ex);
    dist_xy=sqrt(ex^2 + ey^2);
    theta_d(i)=atan2(-ez, dist_xy);
end

%% plots 
figure;
plot3(x, y, z, 'b-', 'LineWidth', 2); hold on;
plot3(dock(1), dock(2), dock(3), 'o', 'MarkerSize', 10, 'LineWidth', 2);
plot3(x0(1),x0(2),x0(3),'MarkerSize',5,'LineWidth',2, 'Marker', '*')
grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('AUV Trajectory', 'Dock', 'Initial position of AUV')
title("3D docking simulation")


% Distance vs Time
dist = sqrt((dock(1)-x).^2 + (dock(2)-y).^2 + (dock(3)-z).^2);
figure;
plot(tsol, dist, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Distance to Dock [m]');
title('Distance vs Time (P Control Only)');
grid on;

% heading
figure;
plot(tsol, pi_to_pi(psi_d - psi), 'LineWidth', 2);
xlabel('Time [s]');
ylabel('\psi_d - \psi [rad]');
title('Yaw Tracking Error');
grid on;

% pitch
figure;
plot(tsol, theta_d - theta, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('\theta_d - \theta [rad]');
title('Pitch Tracking Error');
grid on;
%% function to solve

function dx = controllor(t,x,Kp_psi,Kp_theta,Kp_x,Kp_y,Kp_z,dock)
% Desired angles

ex=dock(1)-x(1);
ey=dock(2)-x(2);
ez=dock(3)-x(3);
psi_d=atan2(ey,ex);
theta_d=atan2(-ez,sqrt(ey^2+ex^2));

% Current states
theta=x(5);
psi=x(4);

u=Kp_x*(cos(psi)*ex+sin(psi)*ey);
v=Kp_y*(-sin(psi)*ex+cos(psi)*ey);
w=Kp_z*ez;

% Kinematics (full model)
dx = zeros(5,1);
dx(1)=u*cos(psi)*cos(theta) - v*sin(psi) + w*cos(psi)*sin(theta);
dx(2)=u*sin(psi)*cos(theta) + v*cos(psi) + w*sin(psi)*sin(theta);
dx(3)=-u*sin(theta) + w*cos(theta);
dx(4)=Kp_psi*pi_to_pi(psi_d-psi);  
dx(5)=Kp_theta*pi_to_pi(theta_d-theta);

end





%% additional func
function x=pi_to_pi(x)
        x=mod(x,2*pi);
        if x>=pi
            x=x-2*pi;
        end
        
    end