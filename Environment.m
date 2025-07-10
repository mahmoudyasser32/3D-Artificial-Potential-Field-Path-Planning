classdef Environment
    properties
        obstacles = {}; % list of obstacle structs with fields: type, pos, size, color
        figureHandle
    end

    methods
        function obj = Environment()
            obj.figureHandle = figure;
            hold on;
            view(3);
            axis vis3d;
            grid on;
            xlabel('x'); ylabel('y'); zlabel('z');
            xlim([0 30]); ylim([0 30]); zlim([0 10]);
        end

        function obj = addCylinder(obj, pos, radius, color)
            obj.obstacles{end+1} = struct('type', 'cylinder', 'pos', pos, 'radius', radius, 'color', color);
            obj.drawCylinder(pos, radius, color);
        end

        function obj = addWall(obj, basePos, dims, color)
            if nargin < 4
                color = [0.6 0.6 0.6]; % default grey
            end
            wall.pos = basePos(:);   % [x; y; z]
            wall.dims = dims(:);     % [width; depth; height]
            wall.type = 'wall';
            wall.color = color;
            obj.obstacles{end+1} = wall;
            obj.drawWall(wall);
        end

        function obj = addSphere(obj, pos, radius, color)
            [X, Y, Z] = sphere(30);
            X = X * radius + pos(1);
            Y = Y * radius + pos(2);
            Z = Z * radius + pos(3);
            surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'none');
            obj.obstacles{end+1} = struct('type', 'sphere', 'pos', pos, 'radius', radius, 'color', color);
        end

        function drawCylinder(~, pos, radius, color)
            nSides = 50;
            [X,Y,Z] = cylinder(radius, nSides);
            Z = Z * pos(3);
            X = X + pos(1);
            Y = Y + pos(2);
            surf(X,Y,Z,'FaceColor',color,'EdgeColor','none');
        end

        function drawWall(~, wall)
            pos = wall.pos(:);
            dims = wall.dims(:);
            color = wall.color;
        
            % Get the 8 corners of the box
            [X, Y, Z] = ndgrid([0, 1], [0, 1], [0, 1]);
            corners = [X(:), Y(:), Z(:)] .* dims' + pos';
        
            % Define box faces by corner indices
            faces = [
                1 3 4 2;  % bottom
                5 6 8 7;  % top
                1 2 6 5;  % front
                2 4 8 6;  % right
                4 3 7 8;  % back
                3 1 5 7   % left
            ];
        
            % Plot the box
            for i = 1:size(faces,1)
                f = faces(i,:);
                patch('Vertices', corners, 'Faces', f, ...
                      'FaceColor', color, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            end
        end

    end
end