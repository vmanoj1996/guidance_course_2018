syms t;

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
% % w=0.5;
% % xT = cos(w*t-pi/2)+5;
% % yT = sin(w*t-pi/2)+6;
% % vxT = diff(xT);
% % vyT = diff(yT);

%leader trajectory: sinusoid
% xT = 5*sin(t);
% yT = t;
% vxT = 5*cos(t);
% vyT = 1;
% axT = -5*sin(t);
% ayT = 0;
%leader trajectory: spiral
xT = -1.5*t*cos(2t);
yT = 1.5*t*sin(2t);
vxT = diff(xT);
vyT = diff(yT);

alpha_t=atan2(vyT,vxT);
omega=matlabFunction(diff(alpha_t));
vel=matlabFunction(sqrt(vxT^2+vyT^2));


omega_func = @(k) omega(0.01*k);
vel_func = @(k) vel(0.01*k);

% hold on;

% for k1 = 1:9000
%     x(k1)=omega(0.01*k1);
% end
% plot(x,'.');
% function lead_init = leader_initial_state()
%     lead_init = [;double(subs(yT,0));double(subs(alpha_t,0))];
% end