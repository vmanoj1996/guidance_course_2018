% Question 3a on page 108-109 NPTEL lectures
% All units in SI unless specified
% Pure pursuit!! with Maneuvering target 
% MATLAB 2017A ACAD VERSION

%% INIT
R0           = 3000;
theta0       = pi/6;
alpha_T_0      = 170/180*pi;
VT           = 300;
R_tolerance  = 0.5; %meter 
max_time_step= 0.001;
tSim         = 20 ; 
XT0 = R0*cos(theta0);
XP0 = 0;
YT0 = R0*sin(theta0);
YP0 = 0;
alphaP0 = theta0 
%% SOLUTION
for nu = [.8, 1, 1.5]
    %% FORMULATION AND ODE INTEGRATION
    VP       = @(nu)    nu*VT;
    getAlpha_T = @(theta) alpha_T_0 + CT*(theta-theta0);
    V_R      = @(theta,alphaP) VT*cos( getAlpha_T(theta) - theta) - VP(nu)*cos(alphaP-theta);
    V_theta  = @(theta,alphaP) VT*sin( getAlpha_T(theta) - theta) - VP(nu)*sin(alphaP-theta);
    alphaPdot = @(y) (V_theta(y(2),y(7))/y(1));
    
    derivative = @(t,y) [V_R(y(2),y(7)) , V_theta(y(2),y(7))/y(1), VP(nu)*cos( y(7) ) , VP(nu)*sin( y(7) ) , VT*cos(getAlpha_T(y(2))) , VT*sin(getAlpha_T(y(2))), alphaPdot(y)]';
    %where derivative = [VR thetadot Vx_p, Vy_p, Vx_t, Vy_t,]
    options = odeset('MaxStep',max_time_step );
    [TOUT,YOUT] = ode45(derivative, [0,tSim], [R0, theta0, XP0, YP0, XT0, YT0,alphaP0]' ,options );
    
    %TAKE ONLY R>=0 terms
    YOUT = YOUT(YOUT(:,1)>=0,:);
    TOUT = TOUT(YOUT(:,1)>=0);
    
    theta = YOUT(:,2);
    alpha_P = YOUT(:,7);
    V_theta_ = V_theta(YOUT(:,2), YOUT(:,7));
    V_R_ = V_R( YOUT(:,2), YOUT(:,7));
    Acc_p = 0;
    for i = 1:length(YOUT(:,1))
        Acc_p(i) = VP(nu)*alphaPdot(YOUT(i,:)');
    end
    XP = YOUT(:,3);
    YP = YOUT(:,4);
    XT = YOUT(:,5);
    YT = YOUT(:,6);
    
    %% PLOT RANGE
    figure(1);
    plot(TOUT, YOUT(:,1)); 
    title('Range vs time');
    xlabel('time(s)')   ; ylabel('Range(m)');       hold on;
                txt     = ['nu= ' num2str(nu)];
                index   = fix(length(TOUT)/1.5-25);
                text(TOUT(index), YOUT(index,1),txt);
                plot(TOUT(end), YOUT(end,1), 'X')
                savefig('images/Range_time')
                print('images/Range_time','-dpng')
    %% PLOT LOS ANGLE
    figure(2);
    plot(TOUT, YOUT(:,2)*180/pi); 
    title('Los angle vs time'); 
    xlabel('time(s)')     ; ylabel('\theta(deg)'); hold on;
                txt     = ['nu= ' num2str(nu)];
                index   = fix(length(TOUT)/1.5-25);
                text(TOUT(index), YOUT(index,2)*180/pi,txt);
                plot(TOUT(end), YOUT(end,2)*180/pi, 'X')
                savefig('images/LOS_angle_time')
                print('images/LOS_angle_time','-dpng')
    %% PLOT V_THETA
    figure(3);
    plot(TOUT, V_theta_); 
    title('V_\theta vs time'); 
    xlabel('time(s)')     ; ylabel('V_\theta(m/s)'); hold on;
                txt     = ['nu= ' num2str(nu)];
                index   = fix(length(TOUT)/1.5-25);
                text(TOUT(index), V_theta_(index),txt);
                plot(TOUT(end), V_theta_(end), 'X')
                savefig('images/V_theta_time')
                print('images/V_theta_time','-dpng')
    %% PLOT RANGE RATE
    figure(4);
    plot( TOUT, V_R_ ); 
    title('Range Rate vs time'); 
    xlabel('time(s)')     ; ylabel('Range Rate(m/s)'); hold on;
                txt     = ['nu= ' num2str(nu)];
                index   = fix(length(TOUT)/2);
                text(TOUT(index), V_R_(index),txt);
                plot(TOUT(end), V_R_(end), 'X')
                savefig('images/range_rate_time')
                print('images/range_rate_time','-dpng')
    %% PLOT ACCELERATION_PURSUER
    figure(5);
    axis([0 tSim 0 600])
    plot( TOUT, Acc_p ); 
    title('Pursuer Acceleration Required by PP'); 
    xlabel('time(s)')     ; ylabel('Acc(m/s^2)'); hold on;
                txt     = ['nu= ' num2str(nu)];
                index   = fix(length(TOUT)/2);
                text(TOUT(index), Acc_p(index),txt);
                plot(TOUT(end), Acc_p(end), 'X')  
                savefig('images/Acceleration_pursuer')
                print('images/Acceleration_pursuer','-dpng')
    %% MISSILE AND TARGET TRAJECTORY
    figure(6)
    plot(XT,YT,'r'); hold on;
    plot(XT(end), YT(end), 'X')
    plot(XP,YP,'b');
    plot(XP(end), YP(end), 'X')
    title('Missile and Target Trajectories')
                txt     = ['nu= ' num2str(nu)];
                index   = fix(length(TOUT)/2);
                text(XP(index), YP(index),txt);
                savefig('images/trajectory')
                print('images/trajectory','-dpng')
    %% Plot VR vs Vtheta
    figure(7)
    plot( V_theta_, V_R_ ); hold on
    plot( V_theta_(end), V_R_(end), 'X' );
    title('V_R V_\theta Phase plot')
    xlabel('V_\theta in m/s');
    ylabel('V_R in m/s')
    axis equal
                txt     = ['nu= ' num2str(nu)];
                index   = fix(length(TOUT)/2);
                text(V_theta_(index), V_R_(index),txt);
                savefig('images/VR_V_theta')
                print('images/VR_V_theta','-dpng')
    %% TMiss & Rmiss or TF
    if YOUT(end,1)<R_tolerance 
        txt = ['Collision detected for nu= ', num2str(nu) , ' at TF=', num2str(TOUT(end)) ,'secs' ];
        disp(txt);
    else
        RMiss = min(YOUT(:,1));
        TMiss = TOUT(RMiss == YOUT(:,1));
        txt = ['no collision for nu=', num2str(nu), ' RMiss= ', num2str(RMiss), ' TMiss= ', num2str(TMiss)];
        disp(txt);
    end
end

