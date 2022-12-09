% --------------------------------------------------%
% Assignment 1 - MTRN9400 T3 2021 - Mohammad Deghat %
% --------------------------------------------------%
function main
clc; clear
figure(1); clf

global qdes l1 l2

% --- Desired end-effector position -----------
qdes = deg2rad([135; 90]);

tspan = [0,3];    % timespan
st = 5;           % video recording step (the larger, the faster)

% ---- Initial values -----------
q1_0 = 0;   q2_0 = 0;
qd1_0 = 0;  qd2_0 = 0;

% ---- ODE solver -----------
Fo = [q1_0; q2_0; qd1_0; qd2_0];       % ODE input vector

options = odeset('RelTol',1e-12);       % ODE solver error
[t,F] = ode45(@System,tspan,Fo,options);

Q1 = F(:,1);    Q2 = F(:,2);            % ODE outputs
Qd1 = F(:,3);   Qd2 = F(:,4);

% ---- Desired location of the end-effector (calculated using Forward Kinematic)
x_des = l1*cos(qdes(1))+l2*cos(qdes(1)+qdes(2));
y_des = l1*sin(qdes(1))+l2*sin(qdes(1)+qdes(2));

% ---- Plotting the joint error vectors -----------
plot_error_signals(t, Q1, Q2, Qd1, Qd2)

%--- Plotting & creating a video of the robot's trajectory
% figure(2);
% plot_robot_traj(st, t, Q1, Q2, x_des, y_des) 

return 





