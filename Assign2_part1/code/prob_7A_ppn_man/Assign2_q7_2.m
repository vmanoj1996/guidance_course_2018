%%
clear all
% INPUTS
Xp0 = [0 0]; %m
theta0 = 30*pi/180; %rad
Xt0 = 7000*[cos(theta0) sin(theta0)]; %m
vp = 400; %m/s
nu = 1/0.6;
vt = vp/nu; %m/s
alpha_p0 = 85*pi/180; %rad
alpha_t = 60*pi/180; %rad
N = 4;
k = N - 1;
phi0 = alpha_p0 - N*theta0; %rad
vt_vec = vt*[cos(alpha_t) sin(alpha_t)]; %m/s
N_vp = N*vp;

%termination criteria
%t_end may be specified to avoid infinite loop in case Rf condition is not met
Rf = 0.5; %range criterion (m)
t_end = 60; %time criterion (s)

%terminate program if t_end is NaN
if isnan(t_end)
    disp('Please specify value for t_end');
    return;
end

tf = NaN;
impact_ang = NaN; %impact angle
tmiss = NaN;
Rmiss = NaN;
flag = true; %set flag to true if Rmiss, tmiss are to be found

tstep = 0.001; %time step

%Evaluation and plotting of trajectories and missile latax
t = 0;
ap = 0;
at = 30;
Xp = Xp0;
Xt = Xt0;
R_vec = Xt - Xp;
R = norm(R_vec);
theta = theta0;
alpha_p = alpha_p0;

delta_t = alpha_t - theta;
delta_p = alpha_p - theta;
v_r = vt*cos(delta_t) - vp*cos(delta_p);
v_theta = vt*sin(delta_t) - vp*sin(delta_p);

%initialising arrays
t_sol = t;
ap_sol = ap;
Xp_sol = Xp;
Xt_sol = Xt;
R_sol = R;
theta_sol = theta;
v_r_sol = v_r;
v_theta_sol = v_theta;

%% 

while true
    if R < Rf
        tf = t;
        impact_ang = abs(alpha_p - alpha_t);
        disp('Interception occurs');
        break;
    end
    
    if t > t_end
        break;
    end
    
    vp_vec = vp*[cos(alpha_p) sin(alpha_p)];
    vt_vec = vt*[cos(alpha_t) sin(alpha_t)];
    %evaluation of parameters for next time step
    t = t + tstep;
    Xp = Xp + vp_vec*tstep;
    Xt = Xt + vt_vec*tstep;
    
    R_vec = Xt - Xp;
    R = norm(R_vec);
    
    if flag
        if R > R_sol(end)
            tmiss = t;
            Rmiss = R;
            disp('Interception does not occur');
            flag = false;
            break; %comment break to calculate data after missile misses target
        end
    end
    
    theta = atan2(R_vec(2), R_vec(1));
    dtheta_t = (theta - theta_sol(end))/tstep;
    
    alpha_t = (at/vt)*t;
    alpha_p = N*theta + phi0;
    ap = N_vp*dtheta_t;
    
    delta_t = alpha_t - theta;
    delta_p = k*theta + phi0;
    v_r = vt*cos(delta_t) - vp*cos(delta_p);
    v_theta = vt*sin(delta_t) - vp*sin(delta_p);
    
    %updating arrays
    t_sol = [t_sol t];
    ap_sol = [ap_sol ap];
    Xp_sol = [Xp_sol; Xp];
    Xt_sol = [Xt_sol; Xt];
    R_sol = [R_sol R];
    theta_sol = [theta_sol theta];
    v_r_sol = [v_r_sol v_r];
    v_theta_sol = [v_theta_sol v_theta];
end

%LOS angle taking vT0 as reference X axis
theta_rel = pi + theta_sol - alpha_t;

%% plot pursuer and target trajectories
subplot(2, 2, [1 3]);
hold on;
plot(Xp_sol(:, 1), Xp_sol(:, 2), 'b', 'linewidth', 1.5);
plot(Xt_sol(:, 1), Xt_sol(:, 2), 'r', 'linewidth', 1.5);
xlabel('x (m)');
ylabel('y (m)');
title('Subplot 1: Pursuer and target trajectories');
legend('pursuer', 'target', 'Location', 'northwest');
set(gca,'fontsize',12.5);

% plot ap_sol
subplot(2, 2, 2);
plot(t_sol, ap_sol, 'linewidth', 1.5);
xlabel('time t (s)');
ylabel('a_p (m/s^2)');
title('Subplot 2: Missile latax');
set(gca,'fontsize',12.5);

% plot trajectory in (v_theta, v_r) space
subplot(2, 2, 4);
plot(v_theta_sol, v_r_sol, 'linewidth', 1.5);
xlabel('v_{\theta} (m/s)');
ylabel('v_r (m/s)');
title('Subplot 3: Trajectory in (v_{\theta}, v_r) space');
set(gca,'fontsize',12.5);

%% plot pursuer relative trajectory in polar plane
polarplot(theta_rel, R_sol, 'linewidth', 1.5);
title('Relative trajectory in polar plane');
% set(gca,'fontsize',12.5);

% polar plot sectors
theta_vr_arr = [0.8482 1.8850 2.6075 3.9584 4.9951 5.7491];

theta_vtheta_arr = [0.2827 1.0681 2.4190 3.4243 4.1783 5.5920];

% theta_vr_arr = [1.2566 2.3876 3.1102 4.4296 5.4978 6.2204];
% theta_vtheta_arr = [0.7226 1.5080 2.7960 3.8642 4.5867 5.9690];
theta_vr_arr = pi + theta_vr_arr - alpha_t;
theta_vtheta_arr = pi + theta_vtheta_arr - alpha_t;
size_roots = size(theta_vtheta_arr);
sign_r = '<';
sign_theta = '<';

hold on;
theta_one = [1 1];

rho = [0 8500];
text_rho = 7000;

for i = 1:size_roots(2) - 1
    polarplot(theta_vr_arr(i)*theta_one, rho, '--r');
    text(theta_vr_arr(i), 8700, 'v_r = 0');
    polarplot(theta_vtheta_arr(i)*theta_one, rho, '--b');
    text(theta_vtheta_arr(i), 8700, 'v_{\theta} = 0');
    
%     text_theta = 0.5*(theta_vr_arr(i) + theta_vtheta_arr(i));
%     text(text_theta, text_rho, strcat('v_r', sign_r, '0, v_{\theta}', sign_theta, '0'));
%     sign_theta = flip_sign(sign_theta);
%     
%     text_theta = 0.5*(theta_vr_arr(i + 1) + theta_vtheta_arr(i));
%     text(text_theta, text_rho, strcat('v_r', sign_r, '0, v_{\theta}', sign_theta, '0'));
%     sign_r = flip_sign(sign_r);
    
    text_theta = 0.5*(theta_vr_arr(i) + theta_vtheta_arr(i));
    text(text_theta, text_rho, strcat('v_r', sign_r, '0, v_{\theta}', sign_theta, '0'));
    sign_r = flip_sign(sign_r);
    
    text_theta = 0.5*(theta_vr_arr(i) + theta_vtheta_arr(i + 1));
    text(text_theta, text_rho, strcat('v_r', sign_r, '0, v_{\theta}', sign_theta, '0'));
    sign_theta = flip_sign(sign_theta);
end

polarplot(theta_vr_arr(size_roots(2))*theta_one, rho, '--r');
text(theta_vr_arr(size_roots(2)), 8700, 'v_r = 0');
polarplot(theta_vtheta_arr(size_roots(2))*theta_one, rho, '--b');
text(theta_vtheta_arr(size_roots(2)), 8700, 'v_{\theta} = 0');

% text_theta = 0.5*(theta_vr_arr(size_roots(2)) + theta_vtheta_arr(size_roots(2)));
% text(text_theta, text_rho, strcat('v_r', sign_r, '0, v_{\theta}', sign_theta, '0'));
% sign_theta = flip_sign(sign_theta);
% 
% text_theta = 0.5*(theta_vr_arr(1) + theta_vtheta_arr(size_roots(2)) - 2*pi);
% text(text_theta, text_rho, strcat('v_r', sign_r, '0, v_{\theta}', sign_theta, '0'));

text_theta = 0.5*(theta_vr_arr(size_roots(2)) + theta_vtheta_arr(size_roots(2)));
text(text_theta, text_rho, strcat('v_r', sign_r, '0, v_{\theta}', sign_theta, '0'));
sign_r = flip_sign(sign_r);

text_theta = 0.5*(theta_vr_arr(size_roots(2)) + theta_vtheta_arr(1) - 2*pi);
text(text_theta, text_rho, strcat('v_r', sign_r, '0, v_{\theta}', sign_theta, '0'));

%% plot v_r vs theta and v_theta vs theta
%t = 20;
alpha_t = (at/vt)*t;
theta_arr = 0:pi/100:4*pi;
delta_t_arr = alpha_t - theta_arr;
delta_p_arr = k*theta_arr + phi0;
v_r_arr = vt*cos(delta_t_arr) - vp*cos(delta_p_arr);
v_theta_arr = vt*sin(delta_t_arr) - vp*sin(delta_p_arr);

size_theta_arr = size(theta_arr);
theta_axis = zeros(size_theta_arr(2));

subplot(2, 1, 1);
plot(theta_arr*180/pi, v_r_arr, 'linewidth', 1.5);
hold on;
plot(theta_arr*180/pi, theta_axis, '--r');
ylabel('v_r (m/s)');
xlabel('\theta (deg)');
title('Subplot 1: v_r vs \theta');
set(gca,'fontsize',12.5);

subplot(2, 1, 2);
plot(theta_arr*180/pi, v_theta_arr, 'linewidth', 1.5);
hold on;
plot(theta_arr*180/pi, theta_axis, '--r');
ylabel('v_{\theta} (m/s)');
xlabel('\theta (deg)');
title('Subplot 2: v_{\theta} vs \theta');
set(gca,'fontsize',12.5);
