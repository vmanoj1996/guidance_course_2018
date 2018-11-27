%% Inputs
dur = 50;
n = 3; %no of robots (including convoy leader)

%leader trajectory: circle
% XP0 = [6 -1; 7 -2];
% vP0 = [5 5];
% alpha_P0 = [125 125]*pi/180;

%leader trajectory: sinusoid
XP0 = [0 -1; 0 -2; 0 -3];
vP0 = [5 5 5];
alpha_P0 = [90 90 90]*pi/180;

%leader trajectory: spiral
% XP0 = [0 -1; 0 -2];
% vP0 = [1 1];
% alpha_P0 = [90 90]*pi/180;

follower = cell(n - 1, 1);

leader_func = @convoy_leader_trajectory;

z0 = [XP0(1, 1); XP0(1, 2); vP0(1)*cos(alpha_P0(1)); vP0(1)*sin(alpha_P0(1))];

options = odeset('RelTol',1e-10);
[t, z] = ode45(@dpp_convoy_state_time_derivative, [0 dur], z0, options);

follower{1}{1} = t;
follower{1}{2} = z;
leader_func = @local_leader_trajectory;

for i = 2:n-1
    local_leader = follower{i - 1};
    ax_leader = (diff(local_leader{2}(:, 3))./diff(local_leader{1}))';
    ay_leader = (diff(local_leader{2}(:, 4))./diff(local_leader{1}))';
    a_time = (local_leader{1}(1:(end-1), 1) + local_leader{1}(2:end, 1))'/2;
    
    z0 = [XP0(i, 1); XP0(i, 2); vP0(i)*cos(alpha_P0(i)); vP0(i)*sin(alpha_P0(i))];
    [t, z] = ode45(@dpp_convoy_state_time_derivative, [0 dur], z0, options);
    
    follower{i}{1} = t;
    follower{i}{2} = z;
end

arr_size = size(follower{1}{1});
xT = zeros(arr_size(1), 1);
yT = zeros(arr_size(1), 1);
vxT = zeros(arr_size(1), 1);
vyT = zeros(arr_size(1), 1);
t = 0:dur/(arr_size(1) - 1):dur;

for i = 1:arr_size(1)
    [xT(i), yT(i), vxT(i), vyT(i)] = convoy_leader_trajectory(t(i));
end

%% Plotting trajectories
hold on;
plot(xT(:, 1), yT(:, 1), 'DisplayName', 'Leader');

for i = 1:n-1
    plot(follower{i}{2}(:, 1), follower{i}{2}(:, 2), 'DisplayName', strcat('Follower', num2str(i)));
end

xlabel('x');
ylabel('y');
title('Trajectories');
legend;