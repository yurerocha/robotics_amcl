classdef OdomModel
    properties
        lastOdomPose
        noise
    end
    methods
        function obj = OdomModel()
            obj.lastOdomPose = [0.0 0.0 0.0];
            obj.noise = [0.2 0.2 0.2 0.2];
        end % Constructor

        function estimatedPose = estimatePose(obj, ut, xtPrev)
            xbar = obj.lastOdomPose(1);
            ybar = obj.lastOdomPose(2);
            thetabar = obj.lastOdomPose(3);
            % -- Unpack values ---------------
            % Odometry motion information (ut)
            xbarPrime = ut(1);
            ybarPrime = ut(2);
            thetabarPrime = ut(3);
            % Pose at t-1
            x = xtPrev(1);
            y = xtPrev(2);
            theta = xtPrev(3);
            % Noise coefficients
            a1 = obj.noise(1);
            a2 = obj.noise(2);
            a3 = obj.noise(3);
            a4 = obj.noise(4);
            % Calculate the relative motion based on what our odometry told us
            deltaRot1 = atan2(ybarPrime - ybar, xbarPrime - xbar) - thetabar;
            deltaTrans = sqrt((xbar - xbarPrime)^2 + (ybar - ybarPrime)^2);
            deltaRot2 = thetabarPrime - thetabar - deltaRot1;
            
            % Calculate what the relative motion would be if we were to end up at
            % the hypothesized successor pose, xtPrev
            deltahatRot1 = deltaRot1 + obj.prob(deltaRot1, a1*deltaRot1 +  a2*deltaTrans);
            deltahatTrans = deltaTrans + obj.prob(deltaTrans, a3*deltaTrans + a4*(deltaRot1 + deltaRot2));
            deltahatRot2 = deltaRot2 + obj.prob(deltaRot2, a1*deltaRot2 + a2*deltaTrans);

            xPrime = x + deltahatTrans*cos(theta + deltahatRot1);
            yPrime = y + deltahatTrans*sin(theta + deltahatRot1);
            thetaPrime = theta + deltahatRot1 + deltahatRot2;

            obj.lastOdomPose = [xbarPrime ybarPrime thetabarPrime];
            estimatedPose = [xPrime yPrime thetaPrime];
        end

        function p = prob(obj, a, s) 
            % Zero-mean, Gaussian distribution with variance s
            p = normpdf(a, 0, sqrt(s));
        end
    end
end