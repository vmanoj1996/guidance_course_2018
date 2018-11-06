% Assignment 2 . TPN problem
clear all
close all
global VR0 VP0 VT0
%SI unless specified
VP0 = 400;
alpha_T0 = 60/180*pi;
theta_0 = 30/180*pi;
alpha_P0 = 85/180*pi;
R0 = 7000;
XT0 = R0*cos(theta_0);
YT0 = R0*sin(theta_0);
XP0 = 0;
YP0 = 0;
VT0 = 0.6*VP0; 
VR0 = VT0*cos(alpha_T0-theta_0) - VP0*cos(alpha_P0-theta_0);

T_interval = [0, 400];
X0 = [R0, theta_0, alpha_P0, VP0, alpha_T0, XT0, YT0, XP0, YP0]';
opt = odeset('MaxStep', 0.05, 'events', @stopCondition );
[t,y] = ode45(@tpn_ode , T_interval ,  X0 ,opt);

%% Range
plot(t, y(:,1) )
title('Range vs time') ;
xlabel('time(s)')      ;
ylabel('Range(m)') ; 
file =  'Range'   ;
savefig(file)
print(file,'-dpng')
%% MISSILE AND TARGET TRAJECTORY
figure
XT = y(:, 6);
YT = y(:, 7);
XP = y(:, 8);
YP = y(:, 9);
plot(XT,YT,'r'); hold on
plot(XT(end), YT(end), 'X') 
plot(XT(1), YT(1), 'o') 

plot(XP,YP,'b'); hold on
plot(XP(end), YP(end), 'X') 
plot(XP(1), YP(1), 'o') 
hold off
legend('Target', 'Pursuer')
xlabel('X m ')      ;
ylabel('Y m') ; 
title('Trajectory');
file =  'Trajectory'   ;
savefig(file)
print(file,'-dpng')
%% VR vs Vtheta 
figure
for i = 1:length(t)
     der = tpn_ode(t(i), y(i,:)');
    VR(i) = der(1) ;
    Vtheta(i) = der(2)*y(i,1)  ;
    latax(i) = der(3)*y(i,4);
    hold on;
end
plot(  Vtheta,  VR ); hold on;
plot(Vtheta(end), VR(end),'x');
plot(Vtheta(1), VR(1),'o');
title('Vr vs Vtheta');
xlabel('Vtheta');
ylabel('Vr');
file =  'Vr_vtheta'   ;
savefig(file)
print(file,'-dpng')
%% Latax history
figure
plot(t,latax); 
% axis([0 50 -500 500])
title('Latax');
xlabel('time s');
ylabel('latax m/s^2');
file =  'Latax'   ;
savefig(file)
print(file,'-dpng')

    