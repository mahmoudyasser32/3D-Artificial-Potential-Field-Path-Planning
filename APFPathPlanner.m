classdef APFPathPlanner
    properties
        env                 % UAVEnvironment object
        Katt = 0.08         % Attractive gain
        Krep = 0.1          % Repulsive gain (increased for safety)
        goal                % 3x1 target position
        start               % 3x1 initial position
        stepLimit = 500     % Maximum number of steps
        threshold = 0.1     % Distance threshold to goal
        maxStep = 0.1       % Max movement step size
        safetyMargin = 0.5; 
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
                        rou_center = norm(pos(1:2) - center);
                        delta = rou_center - (obs.radius + obj.safetyMargin);
                        delta = max(delta, eps);  % avoid div by zero
                        sigma = 0.5 * (obs.radius + obj.safetyMargin);
                        decay = exp(-delta^2 / (2 * sigma^2));
                        if pos(3) <= height + obj.safetyMargin
                            dir = [(pos(1:2) - center); 0] / (rou_center + eps);
                            Frep = Frep + obj.Krep * decay * dir;
                        end
        
                    case 'sphere'
                        center = obs.pos(:);
                        rou = norm(pos - center);
                        delta = rou - (obs.radius + obj.safetyMargin);
                        delta = max(delta, eps);
                        sigma = 0.5 * (obs.radius + obj.safetyMargin);
                        decay = exp(-delta^2 / (2 * sigma^2));
                        d_rou = (pos - center) / (rou + eps);
                        Frep = Frep + obj.Krep * decay * d_rou;
        
                    case 'wall'
                        wallMin = obs.pos(:);
                        wallMax = wallMin + obs.dims(:);
                        closestPoint = max(wallMin, min(pos, wallMax));  % projection
                        delta = norm(pos - closestPoint);
                        delta = max(delta, eps);
                        sigma = 0.05 * (norm(obs.dims(:)) + obj.safetyMargin);  % smoother
                        decay = exp(-delta^2 / (2 * sigma^2));
                        d_rou = (pos - closestPoint) / (delta + eps);
                        Frep = Frep + obj.Krep * decay * d_rou;
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
                            inside = true;
                            return;
                        end
        
                    case 'sphere'
                        if norm(pos - obs.pos(:)) <= (obs.radius + obj.safetyMargin)
                            inside = true;
                            return;
                        end
        
                    case 'wall'
                        wallMin = obs.pos(:);
                        wallMax = wallMin + obs.dims(:);
                        if all(pos >= wallMin) && all(pos <= wallMax)
                            inside = true;
                            return;
                        end
                end
            end
        end
  
        function plotRepulsionField(obj, gridRes)
            [xmin, xmax, ymin, ymax, zmin, zmax] = obj.getEnvironmentBounds();
        
            [X, Y, Z] = meshgrid(xmin:gridRes:xmax, ...
                                 ymin:gridRes:ymax, ...
                                 zmin:gridRes:zmax);
        
            numPoints = numel(X);
            U = zeros(numPoints, 1);
            V = zeros(numPoints, 1);
            W = zeros(numPoints, 1);
            magnitudes = zeros(numPoints, 1);
        
            for i = 1:numPoints
                pos = [X(i); Y(i); Z(i)];
                Frep = obj.computeRepulsion(pos);
                mag = norm(Frep);
                magnitudes(i) = mag;
        
                if mag > 0
                    Frep = Frep / mag;
                end
        
                U(i) = Frep(1);
                V(i) = Frep(2);
                W(i) = Frep(3);
            end
        
            % Normalize magnitudes
            magNorm = (magnitudes - min(magnitudes)) / (max(magnitudes) - min(magnitudes) + eps);
        
            % Map to jet colormap
            cmap = jet(256);  % or parula, hot, etc.
            colorIdx = round(1 + magNorm * 255);
            colorIdx = min(max(colorIdx, 1), 256);  % Ensure index in [1, 256]
            C = cmap(colorIdx, :);
        
            % Plot vector field
            figure;
            hold on;
            for i = 1:numPoints
                quiver3(X(i), Y(i), Z(i), U(i), V(i), W(i), ...
                        'Color', C(i,:), 'AutoScale', 'on', 'MaxHeadSize', 1);
            end
        
            plot3(obj.start(1), obj.start(2), obj.start(3), 'go', 'MarkerSize', 8, 'LineWidth', 2);
            plot3(obj.goal(1), obj.goal(2), obj.goal(3), 'rx', 'MarkerSize', 8, 'LineWidth', 2);
        
            colormap(jet);
            caxis([min(magnitudes), max(magnitudes)]);
            colorbar;
            title('Repulsive Force Field (Color: Low→High)');
            xlabel('X'); ylabel('Y'); zlabel('Z');
            axis equal;
            grid on;
            view(3);
        end
  
        function [xmin, xmax, ymin, ymax, zmin, zmax] = getEnvironmentBounds(obj)
            % Automatically infer bounding box from obstacles
            allPoints = [];
            for i = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{i};
                switch obs.type
                    case 'sphere'
                        c = obs.pos(:);
                        r = obs.radius;
                        allPoints = [allPoints, c - r, c + r];
                    case 'cylinder'
                        c = obs.pos(:);
                        r = obs.radius;
                        allPoints = [allPoints, [c(1)-r; c(2)-r; 0], [c(1)+r; c(2)+r; c(3)]];
                    case 'wall'
                        p = obs.pos(:);
                        d = obs.dims(:);
                        allPoints = [allPoints, p, p + d];
                end
            end
            % Include start and goal
            allPoints = [allPoints, obj.start(:), obj.goal(:)];
        
            % Determine min and max
            xmin = min(allPoints(1, :)) - 2;
            xmax = max(allPoints(1, :)) + 2;
            ymin = min(allPoints(2, :)) - 2;
            ymax = max(allPoints(2, :)) + 2;
            zmin = min(allPoints(3, :)) - 2;
            zmax = max(allPoints(3, :)) + 2;
        end

        function plotRepulsionHeatmapAtZ(obj, zLevel, gridRes)
            % Define bounding area at specific horizontal Z-level
            [xmin, xmax, ymin, ymax, ~, ~] = obj.getEnvironmentBounds();
            [X, Y] = meshgrid(xmin:gridRes:xmax, ymin:gridRes:ymax);
            Z = zLevel * ones(size(X));
        
            magMap = zeros(size(X));
        
            % Evaluate repulsion force magnitude at each grid point
            for i = 1:numel(X)
                pos = [X(i); Y(i); Z(i)];
                Frep = obj.computeRepulsion(pos);
                magMap(i) = norm(Frep);
            end
        
            % Normalize for colormap scaling
            maxVal = max(magMap(:));
            minVal = min(magMap(:));
        
            % Plot heatmap
            figure;
            magMap = smoothdata(magMap, 1, 'gaussian', 3);
            surf(X, Y, Z, magMap, 'EdgeColor', 'none');  % Color by magnitude
            colormap(jet);
            colorbar;
            clim([minVal, maxVal]);
        
            hold on;
            
            plot3(obj.start(1), obj.start(2), zLevel, 'go', 'MarkerSize', 8, 'LineWidth', 2);
            plot3(obj.goal(1), obj.goal(2), zLevel, 'rx', 'MarkerSize', 8, 'LineWidth', 2);
        
            title(['Repulsive Force Heatmap at Z = ' num2str(zLevel)]);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            xlim([-10 30])
            axis tight;
            view(2);  % Top-down view
            grid on;
        end

    end
end
