clear
clc
warning('off')

vi = [3,1];

vj = [0,0];

% Define the polygonal obstacles as a cell array of vertices
obs = {
        %[3, 1; 4, 1; 4, 2; 3, 2], ...
        [2,6; 4, 4; 7, 4; 9,6; 7, 8; 4, 8]
        %[6, 6; 7, 6; 7, 9; 6, 9]
    };

polygon = [3, 1; 4, 1; 4, 2; 3, 2];
isTangential(polygon, vj, vi)

function flag = isTangential(polygon, vj, vi)
    %isTangential - Description
    %
    % Syntax: flag = isTangential(polygon, vi, vj)
    %
    % polygon is a Mx2 matrix of vertices
    % vj is a 1x2 vector of the vertex to check outside
    % vi is a 1x2 vector of the vertex to check on the polygon

    % An edge (vi, vj) is tangential in point vj if the adjacent 
    % corners of the polygon vi−1 and vi+1 lie on the same side of the edge
    
    % Find index of vi
    viIndex = find(ismember(polygon, vi, 'rows'));

    % Find vi-1
    if viIndex == 1
        v1 = polygon(end, :);
    else
        v1 = polygon(viIndex-1, :);
    end

    % Find vi+1
    if viIndex == length(polygon)
        v2 = polygon(1, :);
    else
        v2 = polygon(viIndex+1, :);
    end

    % Check if both edges (vi-1, vi) and (vi, vi+1) are on the same side of (vi, vj)
    a = vi - vj;
    b = v1 - vj;
    c = v2 - vj;
    if (sign(a(1)*b(2) - a(2)*b(1)) == sign(a(1)*c(2) - a(2)*c(1))) || (a(1)*b(2) == a(2)*b(1)) || (a(1)*c(2) == a(2)*c(1))
        flag = true;
    else
        flag = false;
    end
end
