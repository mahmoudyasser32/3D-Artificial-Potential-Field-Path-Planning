classdef APFPathPlanner
    properties
        env                 % UAVEnvironment object
        Katt = 0.08         % Attractive gain
        Krep = 1.0          % Repulsive gain (increased for safety)
        goal                % 3x1 target position
        start               % 3x1 initial position
        stepLimit = 500     % Maximum number of steps
        threshold = 0.1     % Distance threshold to goal
        maxStep = 1.0       % Max movement step size
        safetyMargin = 1.5; 
    end

    methods
        function obj = APFPathPlanner(env, start, goal)
            obj.env = env;
            obj.start = start(:);
            obj.goal = goal(:);
        end

        function path = plan(obj)
            pos = obj.start;
            path = pos';
        
            for i = 1:obj.stepLimit
                % Attractive force
                Fatt = obj.Katt * (obj.goal - pos);
        
                % Repulsive force
                Frep = obj.computeRepulsion(pos);
        
                % If inside obstacle, apply hard push outward
                if obj.isInObstacle(pos)
                    correction = obj.computePushOut(pos);
                    Frep = Frep + correction;
                end
        
                % Total force
                Ftotal = Fatt + Frep;
        
                % Normalize step
                stepVec = Ftotal / (norm(Ftotal) + eps);
                step = min(obj.maxStep, norm(Ftotal));
                new_pos = pos + stepVec * step;
        
                % Store and plot
                path(end+1,:) = new_pos';
                plot3(new_pos(1), new_pos(2), new_pos(3), 'k.');
                drawnow;
        
                % Stop if close enough to goal
                if norm(obj.goal - new_pos) < obj.threshold
                    disp('Goal reached!');
                    break;
                end
        
                pos = new_pos;
            end
            path = smoothdata(path, 1, 'gaussian', 5);
        end


        function Frep = computeRepulsion(obj, pos)
            Frep = zeros(3,1);
            for i = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{i};
                switch obs.type
                    case 'cylinder'
                        center = obs.pos(1:2)';
                        height = obs.pos(3);
                        rou = norm(pos(1:2) - center);
                        zeta = 4.5 * (obs.radius + obj.safetyMargin);
                        if rou < zeta && pos(3) < height
                            d_rou = [(pos(1)-center(1)); (pos(2)-center(2)); 0] / (rou + eps);
                            Frep = Frep + obj.Krep * ((1/rou - 1/zeta) / (rou^2 + eps)) * d_rou;
                        end

                    case 'sphere'
                        center = obs.pos(:);
                        rou = norm(pos - center);
                        zeta = 4.5 * (obs.radius + obj.safetyMargin);
                        if rou < zeta
                            d_rou = (pos - center) / (rou + eps);
                            Frep = Frep + obj.Krep * ((1/rou - 1/zeta) / (rou^2 + eps)) * d_rou;
                        end

                    case 'wall'
                        wallPos = obs.pos(:);           % Ensure column vector
                        wallDims = obs.dims(:);         % Ensure column vector
                        wallMin = wallPos;
                        wallMax = wallPos + wallDims;
                        wallCenter = (wallMin + wallMax) / 2;
                    
                        rou = norm(pos - wallCenter);
                        zeta = 4.5 * (max(wallDims) + obj.safetyMargin);
                        if rou < zeta
                            d_rou = (pos - wallCenter) / (rou + eps);
                            Frep = Frep + obj.Krep * ((1/rou - 1/zeta) / (rou^2 + eps)) * d_rou;
                        end
                end
            end
        end

        function correction = computePushOut(obj, pos)
            correction = zeros(3,1);
            push_strength = 10.0;
        
            for i = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{i};
                switch obs.type
                    case 'cylinder'
                        center = obs.pos(1:2)';
                        height = obs.pos(3);
                        if pos(3) <= height
                            dir = [pos(1:2) - center; 0];
                            correction = correction + push_strength * dir / (norm(dir) + eps);
                        end
                    case 'sphere'
                        center = obs.pos(:);
                        dir = pos - center;
                        correction = correction + push_strength * dir / (norm(dir) + eps);
                    case 'wall'
                        wallMin = obs.pos(:);
                        wallMax = obs.pos(:) + obs.dims(:);
                        % Push away from center of wall if inside
                        if all(pos >= wallMin) && all(pos <= wallMax)
                            center = (wallMin + wallMax) / 2;
                            dir = pos - center;
                            correction = correction + push_strength * dir / (norm(dir) + eps);
                        end
                end
            end
        end

        function inside = isInObstacle(obj, pos)
            inside = false;
            for i = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{i};
                switch obs.type
                    case 'cylinder'
                        center = obs.pos(1:2)';
                        height = obs.pos(3);
                        dist2D = norm(pos(1:2) - center);
                        if dist2D <= (obs.radius + obj.safetyMargin) && pos(3) <= height
                            inside = true; return;
                        end
                    case 'sphere'
                        if norm(pos - obs.pos(:)) <= (obs.radius + obj.safetyMargin)
                            inside = true; return;
                        end
                    case 'wall'
                        wallPos = obs.pos(:);
                        wallDims = obs.dims(:);
                        wallMin = wallPos;
                        wallMax = wallPos + wallDims;
                    
                        % If UAV inside wall volume:
                        if all(pos >= wallMin) && all(pos <= wallMax)
                            % Calculate distances to each face of the box
                            distances = [ ...
                                abs(pos(1) - wallMin(1)), ... % left
                                abs(pos(1) - wallMax(1)), ... % right
                                abs(pos(2) - wallMin(2)), ... % front
                                abs(pos(2) - wallMax(2)), ... % back
                                abs(pos(3) - wallMin(3)), ... % bottom
                                abs(pos(3) - wallMax(3))  ... % top
                            ];
                    
                            [~, minIdx] = min(distances);
                    
                            % Push direction: unit vector based on closest face
                            switch minIdx
                                case 1, dir = [-1; 0; 0];  % left
                                case 2, dir = [1; 0; 0];   % right
                                case 3, dir = [0; -1; 0];  % front
                                case 4, dir = [0; 1; 0];   % back
                                case 5, dir = [0; 0; -1];  % bottom
                                case 6, dir = [0; 0; 1];   % top
                            end
                    
                            correction = correction + push_strength * dir;
                        end
                end
            end
        end
    end
end
