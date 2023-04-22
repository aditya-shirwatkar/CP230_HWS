function isAdjacent = checkAdjacent(points, point1, point2)
% Checks if two given points are adjacent in a list of points
% Returns true if the two points are adjacent, false otherwise

% Find the index of the two points in the list
idx1 = find(points(:,1)==point1(1) & points(:,2)==point1(2));
idx2 = find(points(:,1)==point2(1) & points(:,2)==point2(2));

if isempty(idx1) || isempty(idx2)
    isAdjacent = false;
    return
end

% Check if the points are adjacent
if abs(idx1-idx2) == 1 || (idx1 == 1 && idx2 == size(points,1)) || (idx2 == 1 && idx1 == size(points,1))
    isAdjacent = true;
else
    isAdjacent = false;
end
end