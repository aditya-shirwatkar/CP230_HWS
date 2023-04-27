clear
clc
% Define planner to use
planner = 1; % minimalConstruct
%planner = 2; % aStarGrid
% planner = 3; % visibilityGraph


% Define the start and goal positions
start = [0.6, 0.5];
goal = [13, 8];

boundary_obs = { [0,0; 16,0; 16,12; 0,12] };

door_obs = {
    [2,-0.1; 2.1,-0.1; 2.1,2; 2,2], ...
    [4,6; 4.1,6; 4.1,7.5; 4,7.5], ...
    [4,10; 4.1,10; 4.1,12; 4,12], ...
    [6,6; 6.1,6; 6.1,7; 6,7], ...
    [9,3; 10,3; 10,3.1; 9,3.1], ...
};

obs = {
    [2,2; 2.5,2; 2.5,7.5; 4.5,7.5; 4.5,10; 4,10; 4,8; 2,8], ...
    [4,5.4; 10,5.4; 10,3; 12,3; 12,6; 4,6], ...
    [6,7; 12,7; 12,9; 14,9; 14,6; 15,6; 15,10; 6,10], ...
    [8,0; 9,0; 9,4; 8,4], ...
    [14,0; 15,0; 15,4; 14,4], ...
};

tic
if planner == 1
    [path,G] = minimalConstruct(obs, start, goal, boundary_obs);
elseif planner == 2
    path = aStarGrid(start, goal, obs, boundary_obs);    
end

elapsed_time = toc;
disp(['Elapsed time: ' num2str(elapsed_time) ' seconds.']);

robotInitialLocation = path(1,:);
robotGoal = path(end,:);

initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';

robot = differentialDriveKinematics("TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");

controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.7;
controller.MaxAngularVelocity = 10;
controller.LookaheadDistance = 0.3;

goalRadius = 0.5;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Initialize the simulation loop
sampleTime = 0.05;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;

time = 0;
new_obs_prev = obs;

while( distanceToGoal > goalRadius )
    time = time + 1;
    if time <= 100 
    %     % Update the polygonal map
        new_obs = updateMap(obs, boundary_obs, door_obs);
    else
        new_obs = obs;
    end
    % new_obs = updateMap(obs, boundary_obs, door_obs);

    % Check if current path intersects with obstacles using polxpoly
    if ~isempty(path)
        for i = 1:numel(new_obs)
            [xi, yi] = polyxpoly(path(:,1), path(:,2), new_obs{i}(:,1), new_obs{i}(:,2));
            p = [xi, yi];
            num_pts_path = size(path);
            all_p_in_obs_vertex = all(ismember(p, new_obs{i}, 'rows'));
            for j = 1:(num_pts_path-1)
                u = path(j,:);
                v = path(j+1,:);
                is_on_same_side(j) = all(are_points_on_same_side_of_line(new_obs{i}, u, v));
                polygon_intersecting = ~isempty(p) && (~all_p_in_obs_vertex || (all_p_in_obs_vertex && all(is_on_same_side)));
            end

            %if (~isempty(p) && size(p, 1) > 1)  || ~isequal(new_obs, new_obs_prev)
            if(polygon_intersecting || ~isequal(new_obs, new_obs_prev))
                %if (~isempty(p) && size(p, 1) > 1)
                if(polygon_intersecting)
                    disp("Path intersects with obstacle")
                else
                    disp("Map changed")
                end

                disp(p)
                disp("Recalculating path at time: " + num2str(time))
                start = [robotCurrentPose(1), robotCurrentPose(2)];

                if planner == 1
                    [path,G] = minimalConstruct(new_obs, round(start, 2), goal, boundary_obs);
                elseif planner == 2
        
                    path = aStarGrid(round(start, 2), goal, new_obs, boundary_obs);
                end
                controller.Waypoints = path;
                break
            end
        end
    end

    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    % plot(path(:,1), path(:,2),"k--d")
    plot(start(1), start(2), 'r*')
    plot(goal(1), goal(2), 'g*')
    plot([start(1), goal(1)], [start(2), goal(2)], 'b--', 'LineWidth', 2)
    
    if ~isempty(path)
        plot(path(:,1), path(:,2), 'r', 'LineWidth', 2);
    end

    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    for i = 1:numel(new_obs)
        patch(new_obs{i}(:,1), new_obs{i}(:,2), 'k');
    end    
    light;

    % plot boundary
    plot([0 16 16 0 0], [0 0 11 11 0], "k--d");

    axis equal

    waitfor(vizRate);

    new_obs_prev = new_obs;
end