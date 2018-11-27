clear all
close all
tic
lead = robot_class;
follower = robot_class;
radius = 5;
lead.u_vel = 0.5;
lead.u_omega = lead.u_vel/radius;
% X, Y, theta, sigma, R, alpha_T
dist = 0.414*radius;
follower(1).state = [0,0,45*pi/180,45*pi/180,dist,-45*pi/180];
% follower(2).state = [-1,-1,45*pi/180,45*pi/180,dist,45*pi/180];
% follower(3).state = [-2,-2,45*pi/180,45*pi/180,dist,45*pi/180];

for i = 1:9000
    sensor(follower(1),lead);
    follower(1).guidance();
    follower(1).integrator();
%     sensor(follower(2),follower(1));
%     follower(2).guidance();
%     follower(2).integrator();
%     sensor(follower(3),follower(2));
%     follower(3).guidance();
%     follower(3).integrator();
    
end

%% Plots
% plot(follower.trajectory(:,1),follower.trajectory(:,2),'.')
%  hold on
% % plot(follower.trajectory(:,1),follower.trajectory(:,1));
% plot(5+5*sin(lead.u_omega*[0:0.01:90]), 5+5*cos(lead.u_omega*[0:0.01:90]))
% axis equal
% toc

figure(1);
hold on;

h1 = animatedline;
h2 = animatedline;
h3 = animatedline;
g = animatedline;
%// Set x and y limits of the plot
axis([-5 15 -5 15]); 
%// Plot point by point
for k = 1:length(follower(1).trajectory(:,1))
%     pause(0.01)
    addpoints(h1,follower(1).trajectory(k,1),follower(1).trajectory(k,2));
%     addpoints(h2,follower(2).trajectory(k,1),follower(2).trajectory(k,2));
%     addpoints(h3,follower(3).trajectory(k,1),follower(3).trajectory(k,2));
    addpoints(g,(5+5*sin(-3*pi/4-lead.u_omega*0.01*k)),(5+5*cos(-3*pi/4-lead.u_omega*0.01*k)));
%     plot([x(k,1) Xt(k,1)], [x(k,2) x(k,2)]);
    drawnow     
end