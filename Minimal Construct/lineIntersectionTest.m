function [poly, polygon_intersecting] = lineIntersectionTest(obstacles, u, v)
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
        if isempty(u) || isempty(v)
            disp('Bug')
        end
        obstacle_closed_x = [obstacles{i}(:,1); obstacles{i}(1,1)];
        obstacle_closed_y = [obstacles{i}(:,2); obstacles{i}(1,2)];
        [xi, yi] = polyxpoly(obstacle_closed_x, obstacle_closed_y, [u(1), v(1)], [u(2), v(2)]);
        %[xi, yi] = polyxpoly(obstacles{i}(:,1), obstacles{i}(:,2), [u(1), v(1)], [u(2), v(2)]);
        p = [xi, yi];

        % only unique points in p
        p = unique(p, 'rows');
        [m, ~] = size(p);

        if m > 1
            % define a variable mid_pt_of_p_is_inside with length of p-1
            inside = zeros(length(p)-1, 1);
            
            % run a loop for each point in p
            for j = 1:length(p)-1
                % write the mid point of the jth and the j+1 th point in p
                mid_point = (p(j,:) + p(j+1,:))/2;
                [inside(j), ~] = inpolygon(mid_point(1), mid_point(2), obstacles{i}(:,1), obstacles{i}(:,2));
            end 

            % do bitwiseor of inside
            polygon_intersecting(i) = any(inside);
        else
            polygon_intersecting(i) = false;
        end
        % all_p_in_obs_vertex = all(ismember(p, obstacles{i}, 'rows'));
        % is_on_same_side = all(are_points_on_same_side_of_line(obstacles{i}, u, v));
        % % size of p
        % [m, ~] = size(p);
        % polygon_intersecting = ~isempty(p) && (~all_p_in_obs_vertex || (all_p_in_obs_vertex && ~is_on_same_side));

        % if ((m > 1) && ~isempty(p)) % && ~ismember(u, p, 'rows'))
        if(polygon_intersecting(i))
            % distance of each point in p to the initial point u
            p_dist_from_u = sqrt(sum((p - u).^2, 2));

            % find the point in p that is closest to u
            [min_dist_temp, ~] = min(p_dist_from_u);
        else
            min_dist_temp = realmax('double');
        end
        
        % if ((m>1) && (min_dist_temp < min_dist) && ~isempty(p)) && ~checkAdjacent(obstacles{i}, u, v) % and check if u and v are not adjacent nodes in polygon && (~ismember(u, p, 'rows') || ~ismember(v, p, 'rows')))
        if(polygon_intersecting(i) && (min_dist_temp < min_dist))
            % update min_dist
            min_dist = min_dist_temp;
            poly{1} = obstacles{i};
            % Get count of points inside polygon
            %[~, boundary] = inpolygon(p(:,1), p(:,2), obstacles{i}(:,1), obstacles{i}(:,2));

            % Add polygon to poly if boundary are more than 1
            %if sum(boundary) > 1
            %    %poly{end+1} = obstacles{i};
            %    poly{1} = obstacles{i};
            %end
        end
    end
    
% end
%     intersecting_polygons = {}; % Initialize an empty list for intersecting polygons.
%     k=1;
%     for i = 1:length(polygons)
%         polygon = polygons{i};
%         n = length(polygon);
%         for j = 1:n
%             % Check for intersection between line segment u and v and edge (p,q)
%             p = polygon(j,:);
%             if j == n
%                 q = polygon(1,:);
%             else
%                 q = polygon(j+1,:);
%             end
%             [is_intersecting, ~, ~] = find_line_segment_intersection(u, v, p, q);
%             if is_intersecting
%                 intersecting_polygons{k} = polygon; % Add polygon to list of intersecting polygons
%                 k = k + 1;
%                 break; % No need to check other edges of this polygon
%             end
%         end
%     end
%     % return the first intersecting polygon
%     intersecting_polygons_1 = intersecting_polygons{1};
    
% end
    
%     function [is_intersecting, x, y] = find_line_segment_intersection(a, b, c, d)
%         % Finds the intersection point of line segment (a,b) and line segment (c,d).
        
%         [x_int, y_int] = polyxpoly([a(1), b(1)], [a(2), b(2)], [c(1), d(1)], [c(2), d(2)], 'unique');
        
%         if ~isempty(x_int)
%             is_intersecting = true;
%             x = x_int(1);
%             y = y_int(1);
%         else
%             is_intersecting = false;
%             x = NaN;
%             y = NaN;
%         end
        
%     end
        

% function intersecting_polygons_1 = lineIntersectionTest(polygons, u, v)
%     % Creates a list of polygons intersecting line segment u and v.

%         obstacle = obstacles{i};
%         obstacle = [obstacle; obstacle(1,:)]; % Close the polygon
%         % Iterate all edges of obstacle
%         for j = 1:size(obstacle,1)-1
%             edgeStart = obstacle(i,:);
%             edgeEnd = obstacle(i+1,:);

%             dx = edgeEnd(1)-edgeStart(1);
%             dy = edgeEnd(2)-edgeStart(2);
%             p11 = ((u(1)-edgeStart(1))*dy-(u(2)-edgeStart(2))*dx);
%             p12 = ((v(1)-edgeStart(1))*dy-(v(2)-edgeStart(2))*dx);
    
%             if (abs(p11) < 1e-6) && (abs(p12) < 1e-6) % Collinear
%                 % The line is on the edge
%                 continue;
%             end

%             % Our special case: lines touching only in the end points
%             % are not considered to be intersecting.
%             if (abs(u(1) - edgeStart(1)) < 1e-6) && (abs(u(2) - edgeStart(2)) < 1e-6) || ...
%                (abs(u(1) - edgeEnd(1)) < 1e-6) && (abs(u(2) - edgeEnd(2)) < 1e-6) || ...
%                (abs(v(1) - edgeStart(1)) < 1e-6) && (abs(v(2) - edgeStart(2)) < 1e-6) || ...
%                (abs(v(1) - edgeEnd(1)) < 1e-6) && (abs(v(2) - edgeEnd(2)) < 1e-6)
%                 continue;
%             end

%             dx = v(1)-u(1);
%             dy = v(2)-u(2);
%             p21 = ((edgeStart(1)-u(1))*dy-(edgeStart(2)-u(2))*dx);
%             p22 = ((edgeEnd(1)-u(1))*dy-(edgeEnd(2)-u(2))*dx);

%             if (p11*p12 <= 0) || (p21*p22 <= 0)
%                 % intersection
%                 poly = {obstacle};
%                 return;
%             end
%         end

%     end
    
% end

