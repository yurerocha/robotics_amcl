classdef LikelihoodFieldModel
    properties
        sensorLimits
        % Sensor pose relative to the robot's carcaça.
        sensorPose
        z
        sigma
    end
    methods
        function obj = LikelihoodFieldModel()
            obj.sensorLimits = [0.45 8.0];
            obj.z = [0.2 0.2 0.2]; % zHit, zRand, zMax
            obj.sigma = 0.2;
        end % Constructor

        function w = computeWeight(obj, zt, xt, map)
            % Localização, em relação ao quadro de coordenadas global, do sistema
            % de coordenadas local do robô
            x = xt(1);
            y = xt(2);
            theta = xt(3);
            % A localização relativa do sensor no robô é fixa e as coordenadas são
            % xSens, ySens
            % Com orientação angular do feixe do sensor em relação à direção
            % frontal do robô thetaSens
            xSens = obj.sensorPose(1);
            ySens = obj.sensorPose(2);
            thetaSens = obj.sensorPose(3);
            w = 1;
            for k = 1:length(zt)
                if zt(k) > obj.sensorLimits(1) && zt(k) < obj.sensorLimits(2)
                    xzt = x + xSens * cos(theta) - ySens * sin(theta) + zt(k) * cos(theta + thetaSens);
                    yzt = y + ySens * cos(theta) - xSens * sin(theta) + zt(k) * sin(theta + thetaSens);
                    d = 100000;
                    % for i = 1:length(map)
                    %     xPrime = map(i, 1);
                    %     yPrime = map(i, 2);
                    %     d = min(d, (xzt - xPrime)^2 + (yzt - yPrime)^2);
                    % end
                    % for i = 0:m
                    %     for j = 0:n
                    %         [i, j]
                    %         if map.getOccupancy([i j])
                    %             d = min(d, (xzt - i)^2 + (yzt - j)^2);
                    %         end
                    %     end
                    % end
                    for i = 1:length(map)
                        x = map(i, 1);
                        y = map(i, 2);
                        d = min(d, (xzt - x)^2 + (yzt - y)^2);
                    end
                    % To obtain the map:
                    % map = []
                    % for i = 150:620
                    %     for j = 325:560
                    %         if grid.getOccupancy([i j]) == 1
                    %             [i j]
                    %             map = cat(1, map, [i j])
                    %         end
                    %     end
                    % end
                    w = w * (obj.z(1) * obj.prob(d^2, obj.sigma^2) + obj.z(2) / obj.z(3));
                end
            end
            % w
        end

        function p = prob(obj, a, s) 
            % Zero-mean, Gaussian distribution with variance s
            p = normpdf(a, 0, sqrt(s));
        end
    end
end