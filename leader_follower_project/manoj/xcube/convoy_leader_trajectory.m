syms t real;

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
% w=0.5;
% xT = cos(w*t-pi/2)+5;
% yT = sin(w*t-pi/2)+6;
% vxT = diff(xT);
% vyT = diff(yT);


%leader trajectory: cube root if x
xT=0.1*(((t-7))^3+7^3);
yT=0.1*(30*(t-5)+150);
vxT = diff(xT);
vyT = diff(yT);

%leader trajectory: sinusoid
% xT = 5*sin(t);
% yT = t;
% vxT = 5*cos(t);
% vyT = 1;
% axT = -5*sin(t);
% ayT = 0;
%leader trajectory: spiral
% xT = t*cos(t);
% yT = t*sin(t);
% vxT = cos(t) - t*sin(t);
% vyT = sin(t) + t*cos(t);
% axT = -2*sin(t) + t*cos(t);
% ayT = 2*cos(t) - t*sin(t);

alpha_t=atan2(vyT,vxT);
omega=matlabFunction(diff(alpha_t));
vel=matlabFunction(sqrt(vxT^2+vyT^2));


omega_func = @(k) omega(0.01*k);
vel_func = @(k) vel(0.01*k);

% hold on;

% for k1 = 1:900
%     xd(k1)=double(subs(xT,0.01*k1));
%     yd(k1)=double(subs(yT,0.01*k1));
% end
%  plot(xd,yd);
% function lead_init = leader_initial_state()
%     lead_init = [;double(subs(yT,0));double(subs(alpha_t,0))];
% end