function is_on_same_side = are_points_on_same_side_of_line(points, P1, P2)
% Check if all points in 'points' are on the same side of the line formed by 'P1' and 'P2'

% Compute the vector v that goes from P1 to P2
v = P2 - P1;

[num_points,~] = size(points);

for i=1:num_points 
    % Compute the cross product between v and each vector u that goes from P1 to each point in 'points'
    x_i = (points(i,:) - P1);
    cross_prods(i,:) = cross([v(1) v(2) 0.0], [x_i(1) x_i(2) 0.0]);
end
    % Check the sign of the cross product
    is_on_same_side = all(sign(cross_prods) == sign(cross_prods(1)) | abs(cross_prods) < eps);
end
