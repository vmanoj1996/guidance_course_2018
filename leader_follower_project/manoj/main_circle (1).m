clear all
close all
tic
% One leader and 5 followers
lead = robot_class;
follower(5) = robot_class;

%% Initial condition
% X, Y, theta, sigma, R, alpha_T
% sigma is los angle
% theta is orientation of the current bot
% alpha t is the orientation of the target bot

radius = 2;
leader_pos = [5,5];
leader_heading = 0;
los_angles = 45*pi/180;
f1 = [4.8125, 4.8125];
f2 = [4.625 , 4.625];
f3 = [4.4275, 4.4275];
f4 = [4.1875, 4.1875];
f5 = [4.0, 4.0];
theta1 = 45*pi/180;
theta2 = 45*pi/180;
theta3 = 45*pi/180;
theta4 = 45*pi/180;
theta5 = 45*pi/180;
dist1 = norm(f1-leader_pos);
dist2 = norm(f2-f1);
dist3 = norm(f3-f2);
dist4 = norm(f4-f3);
dist5 = norm(f5-f4);
follower(1).state = [leader_pos,theta1,los_angles,dist1,leader_heading]
follower(2).state = [f1,theta2,los_angles,dist2,theta1]
follower(3).state = [f2,theta3,los_angles,dist3,theta2]
follower(4).state = [f4,theta4,los_angles,dist4,theta3]
follower(5).state = [f5,theta5,los_angles,dist5,theta4]
follower

%% Solver
for i = 1:9000
    lead.u_vel = 0.5;
	lead.u_omega = lead.u_vel/radius;
    sensor(follower(1),lead);
    
    follower(1).guidance_pp();
    follower(1).integrator();
    for j= 2:5
        sensor(follower(j),follower(j-1));
        follower(j).guidance_pp();
        follower(j).integrator();
    end
end

%% Plots
for i=1:3   
plot(follower(i).trajectory(:,1),follower(i).trajectory(:,2),'.')
 hold on
end
% plot(follower.trajectory(:,1),follower.trajectory(:,1));
% plot(5+5*sin(lead.u_omega*[0:0.01:90]), 5+5*cos(lead.u_omega*[0:0.01:90]))
axis equal
toc

% figure(1);
% hold on;
% 
% h = animatedline;
% g = animatedline;
% %// Set x and y limits of the plot
% axis([0 10 0 10]); 
% %// Plot point by point
% for k = 1:length(follower.trajectory(:,1))
% %     pause(0.01)
%     addpoints(h,follower.trajectory(k,1),follower.trajectory(k,2));
%     addpoints(g,(5+5*sin(-3*pi/4-lead.u_omega*0.01*k)),(5+5*cos(-3*pi/4-lead.u_omega*0.01*k)));
% %     plot([x(k,1) Xt(k,1)], [x(k,2) x(k,2)]);
%     drawnow     
% end