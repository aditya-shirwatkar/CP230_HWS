clear
clc
warning('off')


u = [4,5.4];

v = [10,3];

% obs = {
%     [0,1; 5,1; 5,5; 0,5; 0,1];
%     };


% % Get count of points inside polygon
% [points_inpoly, boundary] = inpolygon(p(:,1), p(:,2), obs{1}(:,1), obs{1}(:,2));


% Define the polygonal obstacles as a cell array of vertices
% obs = {
%         %[3, 1; 4, 1; 4, 2; 3, 2], ...
%         %[2,6; 4, 4; 7, 4; 9,6; 7, 8; 4, 8],...
%         [3,5; 8, 0; 7,5; 6, 10; 5, 6;4, 8;], ...
%         %[6, 6; 7, 6; 7, 9; 6, 9]
%     };

% obs = {
%         [3, 0.5; 4, 0.5; 4, 1.5; 3, 1.5], ...
%         [8, 8; 12.5,8; 12.5,9; 8,9;],...
%         [3,5; 8, 0; 7,5; 6, 10; 5, 6;4, 8;], ...
%         [1, 1.5; 1, 2.5; 2, 2.5; 2, 1.5], ...
% 
% };

% obs = {
%     [2,2; 2.5,2; 2.5,7.5; 4.5,7.5; 4.5,10; 4,10; 4,8; 2,8], ...
%     % [4,5.4; 10,5.4; 10,3; 12,3; 12,6; 4,6], ...
%     % [6,7; 12,7; 12,9; 14,9; 14,6; 15,6; 15,10; 6,10], ...
%     % [8,0; 9,0; 9,4; 8,4], ...
%     % [14,0; 15,0; 15,4; 14,4], ...
% };

obs = {
    [2,2; 2.5,2; 2.5,7.5; 4.5,7.5; 4.5,10; 4,10; 4,8; 2,8], ...
    [4,5.4; 10,5.4; 10,3; 12,3; 12,6; 4,6], ...
    [8,0; 9,0; 9,4; 8,4], ...
    [14,0; 15,0; 15,4; 14,4], ...
    [6,7; 12,7; 12,9; 14,9; 14,6; 15,6; 15,10; 6,10], ...
};

poly = lineIntersectionTest(obs, u, v);
poly{1}