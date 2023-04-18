function [path,G] = minimalConstruct(obstacles, start, goal)
    %myFun - Description
    %
    % Syntax: path = minimalConstruct(obstacles, start, goal)
    % obstacles is a cell of N polygons, where each polygon is a Mx2 matrix of vertices
    % start is a 1x2 vector of the start point
    % goal is a 1x2 vector of the goal point

    
    flagMax = false;
    q = PriorityQueue(flagMax);
    G = graph;
    G.Nodes = addvars(G.Nodes, {}, 'NewVariableNames', 'Parent'); % Add Parent column to Nodes table

    closedSet = [];
    closedPoly = double.empty(0,2);

    G = addnode(G, {num2str(start), num2str(goal)}); % Add start and goal nodes to graph
    G = setParent(G, start, start); % Add parent of start node to empty string

    G = addNeighbor(G, start, goal); % Add edge between start and goal nodes
    G = setParent(G, goal, start); % Set parent of goal node to start node

    closedSet = [closedSet; start]; % Add start node to closed set

    cost_so_far = dictionary;
    cost_so_far(num2str(start)) = 0;

    % Add start node to priority queue
    q.insert(cost_so_far(num2str(start)) + heuristic_cost_estimate(start, goal), goal);

    while ~q.isEmpty()
        [~, v] = q.pop(); % Pop node with lowest cost from priority queue

        u = getParent(G, v); % Get parent of v
        p = lineIntersectionTest(obstacles, u, v); % Check if line segment between u and v intersects any obstacles

        if isempty(p)
            % Perform A* search
            
            % If reached goal, return path
            if v == goal
                path = v;
                G.Nodes
                G.Edges
                while ~isequal(path(1,:), start)
                    path = [u; path];
                    u = getParent(G, u);
                end
                path = [start; path];
                hold on
                plot(G);
                hold off
                return
            end

            % Add v to closed set
            closedSet = [closedSet; v];

            % For all neighbors of v
            %for vic = G.neighbors(num2str(v)) % NOTE: vi is a cell of string
            for l = 1:length(G.neighbors(num2str(v)))
                vic = G.neighbors(num2str(v));
                vi = vic{l}; % convert to string

                % Check if neighbor is in closed set
                if ~ismember(str2num(vi), closedSet, 'rows')
                    % Check if neighbor is in priority queue or cost_so_far(dictionary) is greater than cost of current path
                    if ~q.contains(str2num(vi)) || (cost_so_far(vi) > cost_so_far(num2str(v)) + norm(str2num(vi) - v))
                        % Set v as parent of vi
                        G = setParent(G, str2num(vi), v);

                        % Add neighbor to cost_so_far
                        cost_so_far(vi) = cost_so_far(num2str(v)) + norm(str2num(vi) - v);
                        priority_vi = cost_so_far(vi) + heuristic_cost_estimate(str2num(vi), goal);
                        
                        % Add neighbor to priority queue
                        q.insert(priority_vi, str2num(vi));
                    end
                end
            end
        else
            % v is no longer a neighbor of u
            G = removeNeighbor(G, num2str(u), num2str(v));
    
            % Remove parent of v
            G = removeParent(G, v);

            % Find parent of v
            [G, q, closedSet, cost_so_far] = findParent(G, v, q, closedSet, cost_so_far, goal);
            
            %celldisp(p)
            % check if the vertices of interecting polygons are in closed set
            %for i = 1:length(p)
                p_i = p{1};

                % Check if polygon pi is in closed set
                if any(~ismember(p_i, closedPoly, 'rows'))
                    % Connect Obstacle
                    [G, q, closedSet, cost_so_far] = connectObstacle(p_i, G, q, closedSet, cost_so_far, goal);
                            
                    % Add polygon p_i to closed set
                    closedPoly = [closedPoly; p_i];                    
                end
            %end
        end
    end
    G.Nodes
    G.Edges
    hold on
    plot(G);
    hold off

    path = [];
    disp('Search Failed! No Path Found!')
end

function G = addNeighbor(G, Node1, Node2)
    %addNeighbor - Description
    %
    % Syntax: G = addNeighbor(G, Node1, Node2)
    %
    % Long description
    G = addedge(G, num2str(Node1), num2str(Node2));
end

function G = removeNeighbor(G, Node1, Node2)
    %removeNeighbor - Description
    %
    % Syntax: G = removeNeighbor(G, Node1, Node2)

    G = rmedge(G, num2str(Node1), num2str(Node2));
end

function G = setParent(G, Node1, Node2)
    %setParent - Description
    %
    % Syntax: G = setParent(G, Node1, Node2)
    %
    % Parent of Node1 is Node2
    
    % Edit entry of Parent column in Node1
    G.Nodes.Parent{findnode(G, num2str(Node1))} = num2str(Node2);

end

function G = removeParent(G, Node)
    %removeParent - Description
    %
    % Syntax: G = removeParent(G, Node)
    %
    % Remove parent of Node
    G.Nodes.Parent{findnode(G, num2str(Node))} = '[]';
end

function parent = getParent(G, Node)
    %getParent - Description
    %
    % Syntax: parent = getParent(G, Node)
    parent = str2num(G.Nodes.Parent{findnode(G, num2str(Node))});
end

%function poly = lineIntersectionTest(obstacles, u, v)
%    %lineIntersectionTest - Description
%    %
%    % Syntax: poly = lineIntersectionTest(obstacles, u, v)
%    % obstacles is a cell of N polygons, where each polygon is a Mx2 matrix of vertices
%    % u is a 1x2 vector of the start point
%    % v is a 1x2 vector of the goal point
%    
%    % First calculate intersection points using polyxpoly
%    % Then see if the points are inside or on the edges of the polygons using inpolygon
%    % Ignore the boundary ones
%    poly = {};
%    for i = 1:length(obstacles)
%        [xi, yi] = polyxpoly(obstacles{i}(:,1), obstacles{i}(:,2), [u(1), v(1)], [u(2), v(2)]);
%        p = [xi, yi];
%        if ~isempty(p)
%            % Get count of points inside polygon
%            [~, boundary] = inpolygon(p(:,1), p(:,2), obstacles{i}(:,1), obstacles{i}(:,2));
%
%            % Add polygon to poly if boundary are more than 1
%            if sum(boundary) > 1
%                poly{end+1} = obstacles{i};
%            end
%        end
%    end
%    
%end

function [G, q, closedSet, cost_so_far] = findParent(G, v, q, closedSet, cost_so_far, goal)
    %findParent - Description
    %
    % Syntax: [G, q, closedSet, cost_so_far] = findParent(G, v, q, closedSet, cost_so_far, goal)
    % G is the graph
    % v is the node to find the parent of
    % q is the priority queue
    % closedSet is a Nx2 matrix of nodes that have been visited
    % cost_so_far is a dictionary of the cost of each node

    minPathCost = inf;
    newParent = [];
    % for all neighbors of v
    if ~isempty(G.neighbors(num2str(v)))
        %for vic = G.neighbors(num2str(v)) % NOTE: vi is a cell of string
        for k=1:length(G.neighbors(num2str(v))) % NOTE: vi is a cell of string
            vic = G.neighbors(num2str(v));
            vi = vic{k}; % convert to string
            % check if neighbor is in closed set
            if ismember(str2num(vi), closedSet, 'rows')
                if ((cost_so_far(vi) + norm(str2num(vi) - v)) <= minPathCost)
                    minPathCost = cost_so_far(vi) + norm(str2num(vi) - v);
                    newParent = str2num(vi);
                    % cost_so_far(num2str(newParent)) = cost_so_far(vi) + norm(str2num(vi) - v);
                end
            end
        end
    end

    if ~isempty(newParent)
        % Set parent of v to newParent
        G = setParent(G, v, newParent);
        cost_so_far(num2str(v)) = cost_so_far(num2str(newParent)) + norm(v - newParent);
        priority = cost_so_far(num2str(v)) + heuristic_cost_estimate(v, goal);
        q.insert(priority, v);
    end
end

function cost = heuristic_cost_estimate(v, goal)
    %heuristic_cost_estimate - Description
    %
    % Syntax: cost = heuristic_cost_estimate(v, goal)
    % v is a 1x2 vector of the start point
    % goal is a 1x2 vector of the goal point
    %
    % return the heuristic cost estimate of the distance between v and goal
    cost = norm(v - goal);
end

function [G, q, closedSet, cost_so_far] = connectObstacle(polygon, G, q, closedSet, cost_so_far, goal)
    %connectObstacle - Description
    %
    % Syntax: [G, q, closedSet, cost_so_far] = connectObstacle(polygon, G)
    %
    % polygon is a Mx2 matrix of vertices
    % G is the graph
 
    % For all vertices in polygon
    for i = 1:length(polygon)
        vi = polygon(i, :);

        % Check if vertex vi is convex
        if isConvex(polygon, vi)
            % Add vi to graph as a node
            G = addnode(G, num2str(vi));

            % For all nodes in the graph known so far
            for j = 1:height(G.Nodes)
                vj = str2num(G.Nodes.Name{j});

                if ~isequal(vi, vj)
                    % Check if vj is tangential to vi
                    if isTangential(polygon, vj, vi)
                        % add vi as a neighbor of vj
                        G = addNeighbor(G, vj, vi);
                    end
                end
            end
            % Find parent of vi
            [G, q, closedSet, cost_so_far] = findParent(G, vi, q, closedSet, cost_so_far, goal);

        end
    end
end

function flag = isConvex(polygon, vi)
    %isConvex - Description
    %
    % Syntax: flag = isConvex(polygon, vi)
    % polygon is a Mx2 matrix of vertices
    % vi is a 1x2 vector of the vertex to check
    % 
    % Check if vertex vi is convex by seeing cross product between vi-1 vi and vi vi+1 is positive

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

    % Check if sign of 2D cross product is positive
    %a = vi - v1;
    a = v1-vi;
    b = v2 - vi;
    %if a(1)*b(2) - a(2)*b(1) > 0
    if b(1)*a(2) - b(2)*a(1) > 0
        flag = true;
    else
        flag = false;
    end
end

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