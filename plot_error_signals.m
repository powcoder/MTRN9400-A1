% MTRN9400 T3 2021 Assignment 1 ---------------------------------------
% This function is called within the main.m file and generates a video 
% file that demonstrates how the robot moves.
% ---------------------------------------------------------------------
function plot_error_signals(t, Q1, Q2, Qd1, Qd2)

global qdes

subplot(1,2,1);
hold on; grid on
plot(t,Q1-qdes(1))
plot(t,Q2-qdes(2))
xlabel('Time $$ (sec)$$','Interpreter','Latex');
title('Joint position error $$ (rad)$$','Interpreter','Latex');
h = legend('$$q_1(t)-q_{1d}$$','$$q_2(t)-q_{2d}$$');
set(h,'Interpreter','latex')

subplot(1,2,2);
hold on; grid on
plot(t,Qd1)
plot(t,Qd2)
xlabel('Time $$ (sec)$$','Interpreter','Latex');
title('Joint velocity $$ (rad/s)$$','Interpreter','Latex');
h = legend('$$\dot{q}_1(t)$$','$$\dot{q}_2(t)$$');
set(h,'Interpreter','latex')

return