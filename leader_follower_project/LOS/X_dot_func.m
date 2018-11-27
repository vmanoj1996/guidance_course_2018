function X_dot = X_dot_func(t,X)
   
    %defining variables [vt vp alphat xpinit rt]
    [Xt,alpha_t,Xtinit] = leader_trajectory(t);
    Vt = sqrt(Xt(3)^2+Xt(4)^2);
    Rt = sqrt(Xt(1)^2+Xt(2)^2);
    
%     Xinit = follower_Xinit();
    Vp = sqrt(X(3)^2+X(4)^2);
    
    alpha_p=X(5);
    w=0.2;
    theta = atan2(Xt(2)-X(2),Xt(1)-X(1));
        
    % equations as function of state variables X=[x y vx vy]
    
    ap = (2*Vt*Vp*sin(alpha_t-alpha_p))/(Rt*cos(alpha_p-theta));
%     disp(ap);
    % state differential
    X_dot = [X(3);X(4);-X(4)*ap/Vp;X(3)*ap/Vp;ap/Vp];
    
%     X_dot(1) = X(3);
%     X_dot(2) = X(4);
%     X_dot(3) = -ap*sin(theta);
%     X_dot(4) = -ap*cos(theta);
    
end

