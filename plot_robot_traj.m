% MTRN9400 T3 2021 Assignment 1 ---------------------------------------
% This function is called within the main.m file and polts the position
% velocity error signals.
% ---------------------------------------------------------------------
function [F]= plot_robot_traj(st, t, Q1, Q2, x_des, y_des)

video = VideoWriter('Control','MPEG-4');
video.Quality = 75;
open(video);

global l1 l2

x1=0; y1=0;     % Coordinates of joint 1
format bank
for j=1:length(Q1)
    q1 = Q1(j);
    q2 = Q2(j);
    
    % Plotting the first link
    x2 = l1*cos(q1);
    y2 = l1*sin(q1);
    x3 = x2+l2*cos(q1+q2); 
    y3 = y2+l2*sin(q1+q2);
    xa = [x1, x2];
    ya = [y1, y2];
    plot(xa, ya, 'b-', 'LineWidth', 10);
    hold on
    
    % Plotting the second link    
    xb = [x2, x3];
    yb = [y2, y3];
    plot(xb, yb, 'r-', 'LineWidth', 10);
    xlim([-0.6, 0.6]);
    ylim([-0.6, 0.6]);
    axis square
    
    % Plotting the positio of the end-effector
    ee(:,j) = [x3;y3]; 
    plot(ee(1,:),ee(2,:),'k')
    
    t_disp = fix(t(j)*1000)/1000;
    xlabel(['t = ' num2str(t_disp) ' sec'])
    
    % Plotting the desired location of the end-effector
    plot(x_des, y_des ,'*')
    if mod(j,st)==0
        currFrame = getframe(gcf);
        writeVideo(video,currFrame);
    end
    hold off
end
close(video);
return