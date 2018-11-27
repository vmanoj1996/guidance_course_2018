function [xT, yT, vxT, vyT, axT, ayT] = convoy_leader_trajectory(t)
% XT0 = [5 5];
% vT = 1;
% alpha_T = 60*pi/180;
% 
% xT = XT0(1) + vT*cos(alpha_T)*t;
% yT = XT0(2) + vT*sin(alpha_T)*t;
% vxT = vT*cos(alpha_T);
% vyT = vT*sin(alpha_T);
% axT = 0;
% ayT = 0;

%leader trajectory: circle
% xT = 5*cos(t);
% yT = 5*sin(t);
% vxT = -5*sin(t);
% vyT = 5*cos(t);
% axT = -5*cos(t);
% ayT = -5*sin(t);

%leader trajectory: sinusoid
xT = 5*sin(t);
yT = t;
vxT = 5*cos(t);
vyT = 1;
axT = -5*sin(t);
ayT = 0;

%leader trajectory: spiral
% xT = t*cos(t);
% yT = t*sin(t);
% vxT = cos(t) - t*sin(t);
% vyT = sin(t) + t*cos(t);
% axT = -2*sin(t) + t*cos(t);
% ayT = 2*cos(t) - t*sin(t);
end