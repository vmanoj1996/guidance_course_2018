function [Xt, alpha_to, Xtinit] = leader_trajectory(t)

w = 0.2; vt = 1; % cirlce centered at 10,10 and radius 5
% position: [5 5] at t=0
alpha_to = pi/4;
Xtinit(1)=5; Xtinit(2)=5; Xtinit(3)=vt*cos(alpha_to); Xtinit(4)=vt*sin(alpha_to);

Xt(1) = 10+5*sqrt(2)*cos(w*t-3*pi/4);
Xt(2) = 10+5*sqrt(2)*sin(w*t-3*pi/4);

alpha_t = pi/2 - w*t;

Xt(3) = vt*cos(alpha_t);
Xt(4) = vt*sin(alpha_t);
end