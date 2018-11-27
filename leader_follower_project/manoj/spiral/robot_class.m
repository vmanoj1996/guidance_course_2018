%The robot is modelled and put into a seperate calss with all its physical
%properties. This makes dealing with a lot easier.
% X - X coordinate
% Y - y coordinate
% theta - orientation angle wrt x axis
% SI units unless specified!!!!

classdef robot_class < handle
    properties
        totalTime = 90;
        deltaT     = 0.010;
%         TotalSteps = 3000; % HARDCODED
        trajectory = zeros(9000 , 6); %x,y,theta,sigma
        currentTimeStep = 1;
        state = [0,0,0,0,1,0]'; %Current Bot State  X, Y, theta, sigma, R, alpha_T
        
        %Sensed variables
        lead_vel   = 0;
        lead_omega = 0;
        
        %Guidance commands to be given currently
        u_vel   = 0;
        u_omega = 0;
        
        %Guidance Law Settings for PN guidance
        
    end
    methods
        function derivat = bot_ode(obj, ~, state)
            theta = state(3);
            r = state(5);
            sigma = state(4);
            alpha_t = state(6);
            derivat = [ obj.u_vel*cos(theta);
                        obj.u_vel*sin(theta);
                        obj.u_omega;
                        (1/r)*(obj.lead_vel*sin(alpha_t - sigma) - obj.u_vel*sin(theta-sigma));
                        obj.lead_vel*cos(alpha_t - sigma) - obj.u_vel*cos(theta - sigma);
                        obj.lead_omega];
        end
        
        
        function guidance(obj)
            %VR = 0 
            M = 0.05;
            K = 1.05;
            theta = obj.state(3);
            r = obj.state(5);
            sigma = obj.state(4);
            alpha_t = obj.state(6);
            obj.u_omega = K*(1/r)*(obj.lead_vel*sin(alpha_t - sigma) - obj.u_vel*sin(theta-sigma));
            obj.u_vel = obj.lead_vel*cos(theta - sigma)/cos(theta-sigma);
        end
        
        function sensor(follower, lead)
            follower.lead_vel   = lead.u_vel;
            follower.lead_omega = lead.u_omega;
        end
        function integrator(obj)
            [~,y] = ode45(@obj.bot_ode, [0 obj.deltaT], obj.state );
            obj.state = y(end,:)';
            obj.trajectory(obj.currentTimeStep,:) = obj.state';obj.currentTimeStep = obj.currentTimeStep + 1;
            
        end
    end
end

        