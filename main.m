clc, clear, close all
env = Environment();
start = [10 1 0];
goal = [5 22 3];

% Add buildings
Cpos = [8,8,5;
        15,10,5; 
        10,18,5; 
        ];

radius = [1;0.5;2;];
env = env.addWall([0 0 0], [20 0 5], [1 0 0]);  % width, depth, height
env = env.addWall([0 0 0], [0 25 5], [1 0 0]);  % width, depth, 
env = env.addWall([20 0 0], [0 25 5], [1 0 0]);  % width, depth, height
env = env.addWall([0 25 0], [20 0 5], [1 0 0]);  % width, depth, height
% % env = env.addWall([0 0 5], [20 25 0], [1 0 0]);  % width, depth, height
% 
for i = 1:length(radius)
    env = env.addCylinder(Cpos(i,:), radius(i), [0.25, 0.58, 0.96]);
end

% Mark start and goal
plot3(start(1), start(2), start(3), '*', 'Color', 'cyan', 'MarkerSize', 10);
text(start(1), start(2), start(3)+2, 'Start');
plot3(goal(1), goal(2), goal(3), 's', 'MarkerFaceColor', 'green', 'MarkerSize', 10);
text(goal(1), goal(2), goal(3)+2, 'Goal');

% Run planner
planner = APFPathPlanner(env, start, goal);
path = planner.plan();


% Plot the path as a 3D line
plot3(path(:,1), path(:,2), path(:,3), 'b-', 'LineWidth', 2);

% Mark start and goal
hold on;
plot3(path(1,1), path(1,2), path(1,3), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
plot3(path(end,1), path(end,2), path(end,3), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal');

%planner.plotRepulsionField(2); % Use grid resolution = 2 units
planner.plotRepulsionHeatmapAtZ(2,0.1); % Use grid resolution = 2 units