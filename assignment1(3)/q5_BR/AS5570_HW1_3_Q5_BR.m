% INPUTS
Xp0 = [0 0]; %m
Xt0 = [10000 1000]; %m
vt = 300; %m/s
nu = 1/0.6;
vp = nu*vt; %m/s
alpha_p0 = 0*pi/180; %rad
at = 3*9.81; %m/s^2
alpha_t0 = 60*pi/180; %rad
K = 10;

%termination criteria
%t_end may be specified to avoid infinite loop in case Rf condition is not met
Rf = NaN; %range criterion (m)
t_end = 60; %time criterion (s)

%terminate program if t_end is NaN
if isnan(t_end)
    disp('Please specify value for t_end');
    return;
end

tf = NaN;
tmiss = NaN;
Rmiss = NaN;
flag = true;

tstep = 0.001; %time step

%Evaluation and plotting of missile latax, trajectories and range
t = 0;
ap = 0;
Xp = Xp0;
Xt = Xt0;
R_vec = Xt - Xp;
R = norm(R_vec);
theta = atan2(R_vec(2), R_vec(1));
alpha_p = alpha_p0;
alpha_t = alpha_t0;

%initialising arrays
t_sol = t;
ap_sol = ap;
Xp_sol = Xp;
Xt_sol = Xt;
R_sol = R;

while true
    if R < Rf
        tf = t;
        disp('Interception occurs');
        break;
    end
    
    if t > t_end
        break;
    end
    
    vp_vec = vp*[cos(alpha_p) sin(alpha_p)];
    vt_vec = vt*[cos(alpha_t) sin(alpha_t)];
    
    %evaluation of parameters
    t = t + tstep;
    Xp = Xp + vp_vec*tstep;
    Rp = norm(Xp);
    theta_p = atan2(Xp(2), Xp(1));
    
    Xt = Xt + vt_vec*tstep;
    Rt = norm(Xt);
    theta_t = atan2(Xt(2), Xt(1));
    
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
    alpha_p = alpha_p + (ap/vp)*tstep;
    alpha_t = alpha_t + (at/vt)*tstep;
    ap = K*Rp*(theta_t - theta_p);
    
    %updating arrays
    t_sol = [t_sol t];
    ap_sol = [ap_sol ap];
    Xp_sol = [Xp_sol; Xp];
    Xt_sol = [Xt_sol; Xt];
    R_sol = [R_sol R];
end
%% plot ap_sol
subplot(2, 2, 1);
plot(t_sol, ap_sol);
xlabel('time t (s)');
ylabel('a_p (m/s^2)');
title('Subplot 1: Missile latax');
set(gca,'fontsize',12.5);

%% plotting trajectories
subplot(2, 2, 2);
hold on;
plot(Xp_sol(:, 1), Xp_sol(:, 2), 'b');
plot(Xt_sol(:, 1), Xt_sol(:, 2), 'r');
xlabel('x (m)');
ylabel('y (m)');
title('Subplot 2: Trajectories');
legend('pursuer', 'target', 'Location', 'northwest');
set(gca,'fontsize',12.5);

%% plot Rp*(theta_t - theta_p)
subplot(2, 2, 3);
plot(t_sol, ap_sol/K);
xlabel('time t (s)');
ylabel('R_p (\theta_t - \theta_p) (m)');
title('Subplot 3: R_p (\theta_t - \theta_p)');
set(gca,'fontsize',12.5);

%% plot R_sol
subplot(2, 2, 4);
plot(t_sol, R_sol);
hold on;
scatter(tmiss, Rmiss, 'filled');
text(tmiss - 10, Rmiss - 500, strcat('tmiss = ', num2str(tmiss), ', Rmiss = ', num2str(Rmiss)));
xlabel('time t (s)');
ylabel('R (m)');
title('Subplot 4: Separation between missile and target');
axis([0 60 -1500 12000]);
set(gca,'fontsize',12.5);