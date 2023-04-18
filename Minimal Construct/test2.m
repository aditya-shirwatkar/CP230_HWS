clear
clc
warning('off')

u = [4,8];

v = [8,9];

% Define the polygonal obstacles as a cell array of vertices
obs = {
        %[3, 1; 4, 1; 4, 2; 3, 2], ...
        %[2,6; 4, 4; 7, 4; 9,6; 7, 8; 4, 8],...
        [3,5; 8, 0; 7,5; 6, 10; 5, 6;4, 8;], ...
        %[6, 6; 7, 6; 7, 9; 6, 9]
    };

poly = lineIntersectionTest(obs, u, v);
poly