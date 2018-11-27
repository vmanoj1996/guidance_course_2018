function [xT, yT, vxT, vyT, axT, ayT] = local_leader_trajectory(t)
local_leader_data = evalin('base', 'local_leader');

zT_t = interp1(local_leader_data{1}', local_leader_data{2}, t, 'pchip');
xT = zT_t(1);
yT = zT_t(2);
vxT = zT_t(3);
vyT = zT_t(4);

axT_data = evalin('base', 'ax_leader');
ayT_data = evalin('base', 'ay_leader');
a_time_data = evalin('base', 'a_time');

axT = interp1(a_time_data, axT_data, t, 'pchip');
ayT = interp1(a_time_data, ayT_data, t, 'pchip');
end