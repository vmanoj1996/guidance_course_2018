function dz_dt = dpp_convoy_state_time_derivative(t, z)
delta = 5*pi/180;

[xT, yT, vxT, vyT, axT, ayT] = evalin('base', strcat('leader_func(', num2str(t), ')'));

f1 = ((xT - z(1))*(vyT - z(4)) - (yT - z(2))*(vxT - z(3)))/((xT - z(1))^2 + (yT - z(2))^2);

vT = (vxT^2 + vyT^2)^0.5;
dvT_dt = (vxT*axT + vyT*ayT)/vT;
alpha_T = atan2(vyT, vxT);
theta = atan2(yT - z(2), xT - z(1));
dalpha_T_dt = (vxT*ayT - vyT*axT)/(vT^2);
dtheta_dt = f1;

dvP_dt = (dvT_dt*cos(alpha_T - theta) - vT*(dalpha_T_dt - dtheta_dt)*sin(alpha_T - theta))/cos(delta);
vP = ((z(3))^2 + (z(4))^2)^0.5;

dz_dt = [z(3); z(4); z(3)*dvP_dt/vP - z(4)*f1; z(4)*dvP_dt/vP + z(3)*f1];
end