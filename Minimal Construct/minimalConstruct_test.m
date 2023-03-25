clear
clc
warning('off')

% Define the start and goal positions
start = [0, 0];
goal = [10, 10];

% Define the polygonal obstacles as a cell array of vertices
obs = {
        [-1, 2; 2, 2; 2, 7; -1, 7], ...
        %[4, 4; 7, 4; 7, 8; 4, 8], ...
        [1.5, 1; 1, 2; 3, 4; 4, 3]
    };

path = minimalConstruct(obs, start, goal);

% Plot the results
figure;
hold on;
plot(start(1), start(2), 'r*')
plot(goal(1), goal(2), 'g*')
plot([start(1), goal(1)], [start(2), goal(2)], 'b--', 'LineWidth', 2)
for i = 1:numel(obs)
    patch(obs{i}(:,1), obs{i}(:,2), 'k');
end

if ~isempty(path)
    plot(path(:,1), path(:,2), 'r', 'LineWidth', 2);
end
xlim([-1, 11]);
ylim([-1, 11]);
axis equal;