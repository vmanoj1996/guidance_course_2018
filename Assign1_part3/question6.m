% /home/manoj/gits/projects/courseWork/aerospace courses/Guidance/assignment2
theta_f = 0:0.004:13.75*pi/180;
alpha_PF  = theta_f + asin(1/1.5*sin(theta_f));
plot(theta_f , alpha_PF)