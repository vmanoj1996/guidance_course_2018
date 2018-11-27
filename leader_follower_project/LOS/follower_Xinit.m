function Xinit = follower_Xinit()
    % starts at [2 2] with 2m/s speed at alpha_po=pi/3
    Xinit(1) =2;
    Xinit(2)=2;
    Xinit(3) = 2*cos(pi/3);
    Xinit(4) = 2*sin(pi/3);
    Xinit(5) = pi/3
end