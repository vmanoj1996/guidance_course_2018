function X_dot = X_dot_func(t,X)
   
    %defining variables [yt xt vty vtx alpha_to theta_o aplha_po Vto Vpo]
    [Xt,alpha_to,Xtinit] = leader_trajectory(t);
    Vto = sqrt(Xtinit(3)^2+Xtinit(4)^2);
    
    Xinit = follower_Xinit();
    alpha_po = atan2(Xinit(4),Xinit(3));
    theta_o = atan2(Xinit(2)-Xtinit(2),Xinit(1)-Xtinit(1));
    Vpo = sqrt(X(3)^2+Xinit(4)^2);
    
    % equations as function of state variables X=[x y vx vy]
    N =1;
    theta = atan2(Xt(2)-X(2),Xt(1)-X(1));
    theta_dot = ((Xt(4)-X(4))*(Xt(1)-X(1))-(Xt(3)-X(3))*(Xt(2)-X(2)))/((Xt(1)-X(1))^2+(Xt(2)-X(2))^2);
    Vro = Vto*cos(alpha_to-theta_o)-Vpo*cos(alpha_po-theta_o);
    ap = -N*Vro*theta_dot;
    disp(theta_dot);
    alpha_p=atan2(X(4),X(3));
%     if ap> -1
%         ap=-1;
%     end
    
    % state differential
    X_dot = [X(3);X(4);ap*sin(alpha_p);-ap*cos(alpha_p)];
    
%     X_dot(1) = X(3);
%     X_dot(2) = X(4);
%     X_dot(3) = -ap*sin(theta);
%     X_dot(4) = -ap*cos(theta);    
    
end

