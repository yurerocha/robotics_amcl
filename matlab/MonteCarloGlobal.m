classdef MonteCarloGlobal < handle
    properties
        particles
        nParticles % M
        motionModel
        sensorModel
        samplingCounter
        samplingInterval
        prevPose
        prevEstimatedPose
    end
    properties (Access = private)
        eps = 0.01
    end
    methods
        function obj = MonteCarloGlobal(grid, nParticles)
            m = grid.GridSize(1);
            n = grid.GridSize(2);

            obj.nParticles = nParticles;
            obj.particles = [];
            
            for i=1:nParticles
                x = randi([1 m], 1);
                y = randi([1 n], 1);
                % Random double between 0 and 2pi
                theta = randi([0 99], 1) * 2 * pi / 100.0;
                obj.particles= cat(1, obj.particles, [x y theta]);
            end

            obj.motionModel = OdomModel;
            obj.sensorModel = LikelihoodFieldModel;
            obj.samplingInterval = 5;
            obj.samplingCounter = 0;
            obj.prevPose = [0 0 0];
            obj.prevEstimatedPose = [0 0 0]
        end

        function [isUpdated, estimatedPose] = MCL(obj, map, pose, scan)
            % [obj.prevPose, pose]
            % ~obj.isEqualPose(obj.prevPose, pose)
            if ~obj.isEqualPose(obj.prevPose, pose)
                % Xt - {xt1, xt2, ..., xtM} set of M particles
                % Xtprev = particles - set of particles at t-1

                Xtbar = [];
                eta = 0.0;
                maxWt = 1;
                for m=1:obj.nParticles
                    xt = obj.motionModel.estimatePose(pose, obj.particles(m, :));
                    wt = obj.sensorModel.computeWeight(scan.Ranges, xt, map);
                    maxWt = max(maxWt, wt);
                    % [xt wt]
                    eta = eta + wt;
                    Xtbar = cat(1, Xtbar, [xt wt]);
                end

                % Normalization of the particles' weights.
                for m=1:obj.nParticles
                    Xtbar(m, 4) = Xtbar(m, 4) / eta;
                end
                maxWt = maxWt / eta;

                % estimatedPose = obj.estimatePose(Xt);

                % Sampling of the particles.
                [Xt, estimatedPose] = obj.sampling(Xtbar, maxWt);
                % Xtbar
                % Xt

                isUpdated = true;
                obj.particles = Xt;
                obj.nParticles = length(Xt);
                obj.particles
                obj.nParticles
                % obj.particles
                obj.prevPose = pose;
                obj.prevEstimatedPose = estimatedPose;
            else
                isUpdated = false;
                estimatedPose = obj.prevEstimatedPose;
            end
        end
    end
    methods (Access = private)
        % Draw xt[m] from Xtbar with probability proportional to wt[m].
        % Also return the estimated pose, i.e. the particle with the highest weight.
        function [Xt, estimatedPose] = sampling(obj, Xtbar, maxWt)
            obj.samplingCounter = obj.samplingCounter + 1;
            Xt = [];

            maxWeight = 0.0;
            maxWeighti = 1;
            if length(Xtbar) > 2 && obj.samplingCounter == obj.samplingInterval
                obj.samplingCounter = 0;
                for m=1:obj.nParticles
                    p = randi([0 100], 1) * maxWt / 100.0; % Get a random number p.
                    % Comment rationale
                    if Xtbar(m, 4) > p
                        [Xtbar(m, 4), p]
                        Xt = cat(1, Xt, [Xtbar(m, 1) Xtbar(m, 2) Xtbar(m, 3)]);

                        % Compute the estimated pose.
                        if Xtbar(m, 4) > maxWeight
                            maxWeight = Xtbar(m, 4);
                            maxWeighti = m;
                        end
                    end
                end
            else
                for m=1:obj.nParticles
                    Xt = cat(1, Xt, [Xtbar(m, 1) Xtbar(m, 2) Xtbar(m, 3)]);

                    % Compute the estimated pose.
                    if Xtbar(m, 4) > maxWeight
                        maxWeight = Xtbar(m, 4);
                        maxWeighti = m;
                    end
                end 
            end

            res = Xtbar(maxWeighti, :);
            estimatedPose = [res(1) res(2) res(3)];
        end

        function isEq = isEqualPose(obj, p1, p2)
            isEq = true;
            if ~obj.isEqualVal(p1(1), p2(1))
                isEq = false;
            elseif ~obj.isEqualVal(p1(2), p2(2))
                isEq = false;
            elseif ~obj.isEqualVal(p1(3), p2(3))
                isEq = false;
            end
        end

        function isEq = isEqualVal(obj, a, b)
            isEq = abs(a - b) < obj.eps;
        end

        % Select the particle with the highest weight.
        % function estimatedPose = estimatePose(obj, Xt)
        %     maxw = 0.0;
        %     maxwi = 0;
        %     for i = 1:length(Xt)
        %         if Xt(i, 4) > maxw
        %             maxw = Xt(i, 4);
        %             maxwi = i;
        %         end
        %     end
        %     estimatedPose = Xt(maxwi, :);
        % end
    end
end