function Xinit = follower_Xinit()
    % starts at [2 2] with 2m/s speed at alpha_po=pi/3
    Xinit(1) =7;
    Xinit(2)=7;
    Xinit(3) = 2*cos(-pi/4);
    Xinit(4) = 2*sin(-pi/4);
end