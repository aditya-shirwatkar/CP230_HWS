clear
clc
warning('off')

% Define the start and goal positions
start = [0, 0];
goal = [10,10];

test_u = [0, 0];
test_v = [7, 4];

vi = [7,8];
vj = [4,1];

obs1 = [2.5,3;5.5,1;6,1.5;3,3.5] + [6,0];
obs2 = obs1 + [1,3];
obs3 = [0.9, 1.1; 1.9, 0.9; 2.1, 1.9; 1.1, 2.1] + [-2,0];
obs4 = obs3 + [1,3];
obs5 = obs4 + [1,3];
obs6 = obs5 + [2,1];
obs10 = [3,5; 8, 0; 7,5; 6, 10; 5, 6;4, 8; ];
obs11 = [2,0; 4,0; 4,2; 2,2; ];

set = 4;

% Define the polygonal obstacles as a cell array of vertices
obstacles_set1 = {

        [8, 8; 12.5,8; 12.5,9; 8,9;],...
        [1, 1.5; 2, 1.5; 2, 2.5; 1, 2.5;], ...
        [3, 0.5; 4, 0.5; 4, 1.5; 3, 1.5], ...
        [3,5; 5,4; 7.5,5; 6, 10; 5, 6;4, 8;], ...
        [6.5, 9.25; 7.5,9.25; 7.5,10.8; 6.5,10.8;], ...
    };

obstacles_set2 = {

        [8, 8; 12.5,8; 12.5,9; 8,9;],...
        [1, 1.5; 2, 1.5; 2, 2.5; 1, 2.5;], ...
        [3, 0.5; 4, 0.5; 4, 1.5; 3, 1.5], ...
        [3,5; 8,0; 7.5,5; 6, 10; 5, 6;4, 8;], ...
        [6.5, 9.25; 7.5,9.25; 7.5,10.8; 6.5,10.8;], ...
    };

obstacles_set3 = {

        [8, 8; 12.5,8; 12.5,9; 8,9;],...
        [1, 0.5; 2, 0.5; 2, 1.5; 1, 1.5;], ...
        [3, 0.5; 4, 0.5; 4, 1.5; 3, 1.5], ...
        [3,5; 5,4; 7.5,5; 6, 10; 5, 6;4, 8;], ...
        [6.5, 9.25; 7.5,9.25; 7.5,10.8; 6.5,10.8;], ...
    };

obstacles_set4 = {

        [8, 8; 12.5,8; 12.5,9; 8,9;],...
        [1, 0.5; 2, 0.5; 2, 1.5; 1, 1.5;], ...
        [3, 0.5; 4, 0.5; 4, 3.5; 3, 3.5], ...
        [3,5; 5,4; 7.5,5; 6, 10; 5, 6;4, 8;], ...
        [6.5, 9.25; 7.5,9.25; 7.5,10.8; 6.5,10.8;], ...
    };

if (set == 1)
    obs = obstacles_set1;
elseif (set == 2)
    obs = obstacles_set2;
elseif (set == 3)
    obs = obstacles_set3;
elseif (set == 4)
    obs = obstacles_set4;
end

[path,G] = minimalConstruct(obs, start, goal);

num_edges = size(G.Edges);


% Plot the results
figure;
hold on;
for i=1:num_edges(1)
    x_y_1 = str2num(G.Edges{i,1}{1});
    x_y_2 = str2num(G.Edges{i,1}{2});
    plot([x_y_1(1) x_y_2(1)],[x_y_1(2) x_y_2(2)],'-o');
end
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