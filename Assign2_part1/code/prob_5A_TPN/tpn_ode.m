function X_dot = tpn_ode( t, X )
%All units in Si unless specified
%TPN_ODE Ode governing tpn with maneuvering target is given here
% X= [R, theta, alpha_P, VP, alpha_T]
global VR0 VP0 VT0

c = -3*VR0;
AT = -0;
VT = VT0;

R               = X(1);
theta        = X(2);
alpha_P    = X(3);
VP             = X(4);
alpha_T    =  X(5);
XT  = X(6);
YT = X(7);
XP = X(8);
YP = X(9);
R_dot                 = VT*cos(alpha_T - theta) - VP*cos(alpha_P - theta);
theta_dot          = VT/R*sin(alpha_T-theta) - VP/R*sin(alpha_P - theta);
alpha_P_dot     = c*theta_dot/VP*cos(alpha_P-theta);
VP_dot              = c*theta_dot*sin(alpha_P-theta);
alpha_T_dot     = AT/VT;
XT_dot = VT*cos(alpha_T);
YT_dot = VT*sin(alpha_T);
XP_dot = VP*cos(alpha_P);
YP_dot = VP*sin(alpha_P);

traj_dot = [XT_dot, YT_dot, XP_dot, YP_dot];
X_dot = [R_dot , theta_dot, alpha_P_dot, VP_dot, alpha_T_dot, traj_dot]';

end

