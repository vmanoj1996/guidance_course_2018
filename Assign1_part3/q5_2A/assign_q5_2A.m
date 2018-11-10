%% Initialises target varibales
dr = [8 12]*1000;
alt = [0.5 0.8]*1000;
tar_pos = [10,1]*1000;
a_t = 3*9.81; %2nd part 3*9.81;
alpha_t = pi/3;%(alt(1,1)- alt(1,2))/(dr(1,1) - dr(1,2)); %60
V_t = 300;  %meter/sec
theta_t_prev = atan2(tar_pos(1,2),tar_pos(1,1));
theta_t_dot_prev = 0;

%% pursuer variables
pur_pos = [0,0];
Rp_prev = sqrt(pur_pos(1,1)^2+pur_pos(1,2)^2);
alpha_p = 0;%tar_pos(1,2)/tar_pos(1,1);
theta_p=0;
V_p = V_t/0.6;
K = 5; %1,5,10

dt = .1;
t_end =45;
traj_t=[tar_pos];
traj_p=[pur_pos];
R_rel = [];
latax = [];

%% Calculation
for time=0:dt:t_end%collision == false
    theta_t = atan2(tar_pos(1,2),tar_pos(1,1));
    theta_t_dot = (theta_t - theta_t_prev)/dt;
    theta_t_ddot = (theta_t_dot - theta_t_dot_prev)/dt;
    alpha_t = (a_t/V_t)*dt +alpha_t;
    tar_pos = [V_t*dt*cos(alpha_t), V_t*dt*sin(alpha_t)] + tar_pos;
    theta_t_prev = theta_t;
    theta_t_dot_prev = theta_t_dot;
    
    Rp = sqrt(pur_pos(1,1)^2+pur_pos(1,2)^2);
    Rp_dot = (Rp-Rp_prev)/dt;
    a_p = K*Rp*(theta_t-theta_p)+Rp*theta_t_ddot+ 2*Rp_dot*theta_t_dot;
    alpha_p = a_p/V_p*dt + alpha_p;
    pur_pos = [V_p*dt*cos(alpha_p), V_p*dt*sin(alpha_p)] + pur_pos;
    Rp_prev = Rp;
    theta_p = atan2(pur_pos(1,2),pur_pos(1,1));

    latax = [latax a_p];
    traj_t = [traj_t; tar_pos];
    traj_p = [traj_p; pur_pos];
    R_rel = [R_rel sqrt((tar_pos(1,1)-pur_pos(1,1))^2+(tar_pos(1,2)-pur_pos(1,2))^2)];
    
    %time = time + dt;
    if tar_pos == pur_pos
            disp('Hit');
    end
end

[Rmiss, index] = min(R_rel);
Tf = index*dt;

subplot(2,2,1);
plot(0:dt:t_end,latax);
title('Subplot 1: Missile Latax')

subplot(2,2,2);
plot(0:dt:t_end,R_rel,index*dt,Rmiss,'r*');
text(index*dt, Rmiss, sprintf('Rmiss = %6.3f', Rmiss));
text(index*dt, Rmiss+50, sprintf('Tf = %6.3f', Tf));
ylim([-100 1100])
title('Subplot 2: Separation between missile and target')

subplot(2,2,[3,4]);
plot(traj_p(:,1),traj_p(:,2),'b',traj_t(:,1),traj_t(:,2),'g');
legend('Missile', 'target');
%legend('Rmiss');
title('Subplot 3: Trajectory')
