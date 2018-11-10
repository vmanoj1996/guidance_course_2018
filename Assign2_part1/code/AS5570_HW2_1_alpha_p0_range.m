yaxis = -1500:1500;
size_yaxis = size(yaxis);
xeq0 = zeros(1, size_yaxis(2));
plot(xeq0, yaxis, '-g');

xaxis = -360:360;
size_xaxis = size(xaxis);
yeq584 = 584.758*ones(1, size_xaxis(2));
yeq344 = 344.758*ones(1, size_xaxis(2));

delta0 = -360:0.01:360;
rad_del0 = delta0*pi/180;

%% I and III
y = 894.427*cos(rad_del0) + 400*sin(rad_del0);
hold on;
plot(delta0, y);
plot(xaxis, yeq584, '--c');

%% II and IV
y = 894.427*cos(rad_del0) - 400*sin(rad_del0);
hold on;
plot(delta0, y);
plot(xaxis, yeq344, '--r');