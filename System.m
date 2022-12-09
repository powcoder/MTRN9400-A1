% MTRN9400 T3 2021 Assignment 1 ------------------------------
% This file includes the robot parameters, the controller 
% and the closed-loop system model
% ------------------------------------------------------------
function [F]= System(t,Fo)
global qdes l1 l2

%%/\/\/\/\/ Robot parameters /\/\/\/\/\/\ 
m1 = 1.91;              % Mass of the link #1 assembly (kg)
m2 = 0.97326;           % Mass of the link #2 assembly (kg)
I1 = 39170.18 * 1e-6;   % Moment of inertia of the link #1 assembly at its centre of mass (kg.m^2)
I2 = 8082.84 * 1e-6;    % Moment of inertia of the link #2 assembly at its centre of mass (kg.m^2)
lc1 = 179.0 * 1e-3;     % distance from the link #1 assembly centre of mass to its axis of rotation (m)
l1 = 2*lc1;   
lc2 = (398.4 - 343.4) * 1e-3; % distance from the link #2 assembly centre of mass to its axis of rotation (m)
l2 = 4*lc2;   
g = 9.80665;            % Earth's gravity

q1 = Fo(1);
q2 = Fo(2);
qd1 = Fo(3);
qd2 = Fo(4);

x1 = [q1;q2];           
x2 = [qd1;qd2];         

M(1,1) = m1*lc1^2 + m2*(l1^2+lc2^2+2*l1*lc2*cos(q2))+I1+I2;
M(1,2) = m2*(lc2^2+l1*lc2*cos(q2))+I2;
M(2,1) = M(1,2);
M(2,2) = m2*lc2^2+I2;

C(1,1) = -m2*l1*lc2*qd2*sin(q2);
C(1,2) = -m2*l1*lc2*(qd1+qd2)*sin(q2);
C(2,1) = m2*l1*lc2*qd1*sin(q2);
C(2,2) = 0;

G(1,1) = (m1*g*lc1+m2*g*l1)*cos(q1) + m2*g*lc2*cos(q1+q2);
G(2,1) = m2*g*lc2*cos(q1+q2);

%%/\/\/\/\/ Controller /\/\/\/\/\/\
% --- Control gains -----------
Kp = 5*[1 0; 0 1];
Kd = 1*[1 0; 0 1];

%/\/\/\/\/  Control law  /\/\/\/\/\/\
% You should modify the following controller
tau = - Kp*(x1-qdes) - Kd*x2;       % PD Controller

% ....
% ....






%%/\/\/\/\/ Closed-loop system /\/\/\/\/\/\
dx1 = x2;
dx2 = M^(-1)*(tau - C*x2 - G);       

F = [dx1; dx2];
return