clear all
close all
tic
convoy_leader_trajectory

lead = robot_class;
follower = robot_class;
radius = 5;
% X, Y, theta, sigma, R, alpha_T
xT0 = double(subs(xT,0));
yT0 = double(subs(yT,0));
alpha_t0 = double(subs(alpha_t,0));
xp0 = -0; yp0 = -1;

dist0 = sqrt((xp0-xT0)^2+(yp0-yT0)^2);
los_angles = atan2((yT0-yp0),(xT0-xp0));


f1 = [-1,-1];
% f2 = [-1,-1];
% f3 = [-1,-1];
% f4 = [-1,-1];
% f5 = [-1,-1];
theta1 = 45*pi/180;
% theta2 = 90*pi/180;
% theta3 = 90*pi/180;
% theta4 = 90*pi/180;
% theta5 = 90*pi/180;
% dist2 = norm(f2-f1);
% dist3 = norm(f3-f2);
% dist4 = norm(f4-f3);
% dist5 = norm(f5-f4);
follower(1).state = [f1,theta1,los_angles,dist0,alpha_t0];
% follower(2).state = [f2,theta2,los_angles,dist2,theta1];
% follower(3).state = [f3,theta3,los_angles,dist3,theta2];
% follower(4).state = [f4,theta4,los_angles,dist4,theta3];
% follower(5).state = [f5,theta5,los_angles,dist5,theta4];


figure,hold on;
title('Sinusoidal trajectory using ppn with LOS noise');
xlabel('X');ylabel('Y');
axis([-20 30 -10 100]); 
for i=1:5
    h(i) = animatedline;
end
h(1).Color = 'black';h(2).Color = 'blue';h(3).Color = 'red';h(4).Color = 'black';h(5).Color = 'blue';
 g = animatedline; g.Color = 'red';

for i = 1:1600
    lead.u_omega = omega_func(i);
    lead.u_vel = vel_func(i);
    
    sensor(follower(1),lead);
    follower(1).guidance();
    follower(1).integrator();
    addpoints(h(1),follower(1).trajectory(i,1),follower(1).trajectory(i,2));
    addpoints(g,double(subs(xT,0.005*i)),double(subs(yT,0.005*i)));
%     for j= 2:5
%         sensor(follower(j),follower(j-1));
%         follower(j).guidance();
%         follower(j).integrator();
%         addpoints(h(j),follower(j).trajectory(i,1),follower(j).trajectory(i,2));
%     
%     end
    drawnow 
% 
%         pause(0.005)
%     addpoints(h2,follower(2).trajectory(i,1),follower(2).trajectory(i,2));
%     addpoints(h3,follower(3).trajectory(i,1),follower(3).trajectory(i,2));
%     addpoints(h4,follower(4).trajectory(i,1),follower(4).trajectory(i,2));
%     addpoints(h5,follower(5).trajectory(i,1),follower(5).trajectory(i,2));
%     
%     
end

legend('Leader','Follower1');

%% Plots
% plot(follower.trajectory(:,1),follower.trajectory(:,2),'.')
%  hold on
% % plot(follower.trajectory(:,1),follower.trajectory(:,1));
% plot(5+5*sin(lead.u_omega*[0:0.005:90]), 5+5*cos(lead.u_omega*[0:0.005:90]))
% axis equal
% toc