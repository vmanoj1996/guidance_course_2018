
clear all;
tspan =0:0.1:35;
Xinit = follower_Xinit();
[t,x] = ode45(@X_dot_func, tspan, Xinit);

%% plotting the path
i=1;
for t = 0:0.1:35
    [Xt(i,:), alpha_to, Xtinit] = leader_trajectory(t);
    if Xt(i,1) == x(i,1)
        disp('HIT!!');
    end
    i=i+1;
end

figure(1);
hold on;

h = animatedline;
g = animatedline;
%// Set x and y limits of the plot
axis([0 20 0 20]);
%// Plot point by point
for k = 1:length(x)
    pause(0.01)
    addpoints(h,x(k,1),x(k,2));
    addpoints(g,Xt(k,1),Xt(k,2));
%     plot([x(k,1) Xt(k,1)], [x(k,2) x(k,2)])
    %// MATLAB pauses for 0.001 sec before moving on to execue the next 
    %%// instruction and thus creating animation effect ,x(k,1),x(k,2)
    drawnow     
end