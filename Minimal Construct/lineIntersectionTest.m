function poly = lineIntersectionTest(obstacles, u, v)
    %lineIntersectionTest - Description
    %
    % Syntax: poly = lineIntersectionTest(obstacles, u, v)
    % obstacles is a cell of N polygons, where each polygon is a Mx2 matrix of vertices
    % u is a 1x2 vector of the start point
    % v is a 1x2 vector of the goal point
    
    % First calculate intersection points using polyxpoly
    % Then see if the points are inside or on the edges of the polygons using inpolygon
    % Ignore the boundary ones
    poly = {};
    min_dist = realmax('double');
    for i = 1:length(obstacles)
        obstacle_closed_x = [obstacles{i}(:,1); obstacles{i}(1,1)];
        obstacle_closed_y = [obstacles{i}(:,2); obstacles{i}(1,2)];
        [xi, yi] = polyxpoly(obstacle_closed_x, obstacle_closed_y, [u(1), v(1)], [u(2), v(2)]);
        %[xi, yi] = polyxpoly(obstacles{i}(:,1), obstacles{i}(:,2), [u(1), v(1)], [u(2), v(2)]);
        p = [xi, yi];

        % only unique points in p
        p = unique(p, 'rows');
        
        % size of p
        [m, ~] = size(p);

        if ((m > 1) && ~isempty(p)) % && ~ismember(u, p, 'rows'))
            % distance of each point in p to the initial point u
            p_dist_from_u = sqrt(sum((p - u).^2, 2));

            % find the point in p that is closest to u
            [min_dist_temp, ~] = min(p_dist_from_u);
        else
            min_dist_temp = realmax('double');
        end
        
        if ((min_dist_temp < min_dist) && ~isempty(p) && (~ismember(u, p, 'rows') || ~ismember(v, p, 'rows')))
            % update min_dist
            min_dist = min_dist_temp;          
            
            % Get count of points inside polygon
            [~, boundary] = inpolygon(p(:,1), p(:,2), obstacles{i}(:,1), obstacles{i}(:,2));

            % Add polygon to poly if boundary are more than 1
            if sum(boundary) > 1
                %poly{end+1} = obstacles{i};
                poly{1} = obstacles{i};
            end
        end
    end
    
end