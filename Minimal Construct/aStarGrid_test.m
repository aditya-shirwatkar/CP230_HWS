clear
clc

% Define the start and goal positions
start = [0, 0];
goal = [10, 10];

% Define the polygonal obstacles as a cell array of vertices
obstacles = {
        [3, 1; 4, 1; 4, 2; 3, 2], ...
        [2,6; 4, 4; 7, 4; 9,6; 7, 8; 4, 8], ...
        % [1, 1; 1, 2; 2, 2; 2, 1], ...
        %[6, 6; 7, 6; 7, 9; 6, 9]
    };

% Find the path using A*
path = aStarGrid(start, goal, obstacles, {});

% Plot the results
figure;
hold on;
plot(start(1), start(2), 'r*')
plot(goal(1), goal(2), 'g*')
for i = 1:numel(obstacles)
    patch(obstacles{i}(:,1), obstacles{i}(:,2), 'k');
end

if ~isempty(path)
    plot(path(:,1), path(:,2), 'r', 'LineWidth', 2);
end
xlim([-1, 11]);
ylim([-1, 11]);
axis equal;

