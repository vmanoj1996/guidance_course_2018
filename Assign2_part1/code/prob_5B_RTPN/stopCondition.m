function [endCondition,isterminal,direction] = stopCondition(t,y)
R = y(1);
Tolerence = 0.5;%0.5 meter
endCondition = Tolerence + R;
isterminal = 1;
direction = 0;
end

