function new_obs = updateMap(obs, boundary_obs, door_obs)
    %updateMap - Description
    %
    % Syntax: new_obs = updateMap(obs, boundary_obs, door_obs)
    %

    % Merge door and wall obstacles randomly with a probability of 0.5

    eps = 0.05;
    new_obs = obs;

    % % For door 1
    % % if rand(1) < eps    
        % poly1 = polyshape(obs{1}(:,1), obs{1}(:,2));
        % poly2 = polyshape(door_obs{1}(:,1), door_obs{1}(:,2));
        % polyout = union(poly1,poly2);
        % % Flip the order of the vertices
        % [x,y] = poly2ccw(polyout.Vertices(:,1), polyout.Vertices(:,2));
        % obs{1} = [x,y];
        % new_obs = obs;
    % % end

    % % For door 4
    %     poly1 = polyshape(obs{2}(:,1), obs{2}(:,2));
    %     poly11 = polyshape(obs{3}(:,1), obs{3}(:,2));
    %     % poly111 = polyshape(obs{4}(:,1), obs{4}(:,2));
    %     poly2 = polyshape(door_obs{4}(:,1), door_obs{4}(:,2));
    %     % poly22 = polyshape(door_obs{5}(:,1), door_obs{5}(:,2));
    %     polyout = union(poly1, poly11);
    %     % polyout = union(polyout, poly111);
    %     polyout = union(polyout, poly2);
    %     % polyout = union(polyout, poly22);
    % 
    %     % Flip the order of the vertices
    %     [x,y] = poly2ccw(polyout.Vertices(:,1), polyout.Vertices(:,2));
    %     obs{2} = [x,y];
    %     obs(:,[3]) = [];
    %     new_obs = obs;

    % % For door 1,2,4
    %     poly1 = polyshape(obs{1}(:,1), obs{1}(:,2));
    %     poly11 = polyshape(obs{2}(:,1), obs{2}(:,2));
    %     poly111 = polyshape(obs{3}(:,1), obs{3}(:,2));
    %     poly2 = polyshape(door_obs{1}(:,1), door_obs{1}(:,2));
    %     poly22 = polyshape(door_obs{2}(:,1), door_obs{2}(:,2));
    %     poly222 = polyshape(door_obs{4}(:,1), door_obs{4}(:,2));
    %     polyout = union(poly1, poly2);
    %     polyout = union(polyout, poly22);
    %     polyout = union(polyout, poly11);
    %     polyout = union(polyout, poly222);
    %     polyout = union(polyout, poly111);
    % 
    %     % Flip the order of the vertices
    %     [x,y] = poly2ccw(polyout.Vertices(:,1), polyout.Vertices(:,2));
    %     obs{1} = [x,y];
    %     obs(:,[2,3]) = [];
    %     new_obs = obs;

    % % For door 4_after
        poly1 = polyshape(obs{2}(:,1), obs{2}(:,2));
        poly11 = polyshape(obs{3}(:,1), obs{3}(:,2));
        poly2 = polyshape(door_obs{4}(:,1), door_obs{4}(:,2));
        polyout = union(poly1, poly2);
        polyout = union(polyout, poly11);

        % Flip the order of the vertices
        [x,y] = poly2ccw(polyout.Vertices(:,1), polyout.Vertices(:,2));
        obs{2} = [x,y];
        obs(:,[3]) = [];
        new_obs = obs;


end