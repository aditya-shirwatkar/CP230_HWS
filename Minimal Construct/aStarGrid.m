function path = aStarGrid(start, goal, obstacles)
    % start and goal are 2-element row vectors
    % obstacles is a cell array of polygons, where each polygon is an N-by-2 matrix of vertex coordinates
    
    % Initialize variables
    flagMaxPriorityQueue = false;
    open_set = PriorityQueue(flagMaxPriorityQueue);
    open_set.insert(0, start);
    
    came_from = dictionary;
    came_from(num2str(start)) = num2str(start);
    
    g_score = dictionary;
    g_score(num2str(start)) = 0;
    
    while ~open_set.isEmpty()
        % open_set
    
        % Get the node in the open set with the lowest f_score
        [~, current] = open_set.pop();
        
        % Check if we've reached the goal
        if isequal(current, goal)
            path = reconstruct_path(came_from, start, goal);
            return
        end
        
        % Generate neighboring nodes
        neighbors = generate_neighbors(current, obstacles);
    
        for i = 1:size(neighbors,1)
            neighbor = neighbors(i,:);
            new_cost = g_score(num2str(current)) + distance(current, neighbor);
            
            if ~g_score.isKey(num2str(neighbor)) || new_cost < g_score(num2str(neighbor))
                g_score(num2str(neighbor)) = new_cost;
                priority = new_cost + distance(neighbor, goal);
                open_set.insert(priority, neighbor);
                came_from(num2str(neighbor)) = num2str(current);
            end
        end
    end
    
    disp('No path found')
    path = [];
end

function d = distance(node1, node2)
    % Euclidean distance
    d = norm(node1 - node2);
end

function neighbors = generate_neighbors(node, obstacles)
    % Generate neighbors in 8-connected grid
    [x,y] = meshgrid(-0.25:0.25:0.25);
    deltas = 0.25*[x(:) y(:)];
    deltas(5,:) = [];  % remove the (0,0) delta
    neighbors = bsxfun(@plus, node, deltas);
    
    % Remove neighbors that are inside obstacles
    for i = 1:numel(obstacles)
        obstacle = obstacles{i};
        mask = inpolygon(neighbors(:,1), neighbors(:,2), obstacle(:,1), obstacle(:,2));
        neighbors(mask,:) = [];
    end
end

function path = reconstruct_path(came_from, start, goal)
    current = num2str(goal);
    path = [];
    while ~isequal(current, num2str(start))
        path(end+1,:) = str2num(current);
        current = came_from(current);
    end
    path(end+1,:) = start;
    path = flip(path);
end