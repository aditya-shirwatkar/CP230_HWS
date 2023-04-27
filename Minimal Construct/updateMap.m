function new_obs = updateMap(obs, boundary_obs, door_obs)
    %updateMap - Description
    %
    % Syntax: new_obs = updateMap(obs, boundary_obs, door_obs)
    %

    % Merge door and wall obstacles randomly with a probability of 0.5

    eps = 0.05;
    new_obs = obs;

    % For door 1
    % if rand(1) < eps
        poly1 = polyshape(obs{1}(:,1), obs{1}(:,2));
        poly2 = polyshape(door_obs{1}(:,1), door_obs{1}(:,2));
        polyout = union(poly1,poly2);
        % Flip the order of the vertices
        [x,y] = poly2ccw(polyout.Vertices(:,1), polyout.Vertices(:,2));
        obs{1} = [x,y];

        new_obs = obs;
    % end

end