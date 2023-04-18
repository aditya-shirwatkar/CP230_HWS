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
% Define the polygonal obstacles as a cell array of vertices
obs = {

        [8, 8; 12.5,8; 12.5,9; 8,9;],...
        [1, 1.5; 2, 1.5; 2, 2.5; 1, 2.5;], ...
        [3, 0.5; 4, 0.5; 4, 1.5; 3, 1.5], ...
        [3,5; 8, 0; 7.5,5; 6, 10; 5, 6;4, 8;], ...
        [6.5, 9.25; 7.5,9.25; 7.5,10.8; 6.5,10.8;], ...
        %[2,4.5; 4, 4; 7, 4; 9,6; 7, 8; 5, 6;4,8;], ...
        %[2.5,3;5.5,1;6,1.5;3,3.5], ...

        %obs1, ...
        %obs2,...
        %obs3,...
        %obs4, ...
        %obs5,...
        %obs6,...
        %[2,6; 4, 4; 7, 4; 9,6;],...
        %obs10,...
        %obs11,...
    };

% Tang_flag = isTangential(obs{1}, vi, vj);

%p = lineIntersectionTest(obs, test_u, test_v);
%disp(p)

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