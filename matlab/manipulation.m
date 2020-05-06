% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

%% Milestone Manipulation

function manipulation(vrep, id, h)

    %%%%%%%%%%%%%%%%%%%%%%%
    %% General variables %%
    %%%%%%%%%%%%%%%%%%%%%%%

    % The time step the simulator is using
    timestep = 0.05;

    % Movements tolerances
    movPrecision = 0.3;
    rotPrecision = 0.002;

    % Map parameters
    mapSize = [15, 15];
    mapPrec = 5;

    % Velocity multipliers
    forwBackVelFact = 4;
    rotVelFact = 1.5;

    % Map inflation factor
    mapInflatedFact = 0.6;

    % Possible connection distance between points for A*
    pathDist = 10;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Values initialisation %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%

    fprintf('Starting robot\n');

    % Parameters for controlling the youBot's wheels
    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;

    % Position
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);

    initPos = youbotPos - [mapSize(1), mapSize(2), 0];

    %%%%%%%%%%%%%%%%%%%%%
    %% Loading the map %%
    %%%%%%%%%%%%%%%%%%%%%

    % We reset possible previous figure(s)
    clf;

    % We check if map exists and load it
    if ~isfile('mat/map.mat')
        fprintf('Map representation does not exist. Navigation will begin...\n\n');

        navigation(vrep, id, h);
    end

    load('mat/map', 'map');

    fprintf('The explored map has been loaded.\n\n');

    % Get the occupancy matrix
    occMat = occupancyMatrix(map, 'ternary');

    % Show the map
    utils.map(occMat);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% FSM, objective list and path %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Initial state of the Finite State Machine (FSM)
    state = 'manipulation';

    % List of actions classed by categories
    displacementActions = {'forward', 'backward'};
    rotationActions = {'rotate'};
    
    % Initialise the objective list
    objectiveList = {};

    % Initialise the flag for the current objective
    hasAccCurrentObj = true;

    % Define the path
    pathList = [];
    pathDisp = pathList;

    % Initialize the flag
    removePath = false;


    %%%%%%%%%%%%%%%%%%%%%%%%%
    %% Display information %%
    %%%%%%%%%%%%%%%%%%%%%%%%%

    % Console debug
    fprintf('\n*****************************************\n');
    fprintf('* The robot begins state "%s" *\n', state);
    fprintf('*****************************************\n\n');

    if strcmp(state, 'manipulation')
        fprintf('In this state, he will grab all the object on table, without any falling on the ground and put them on the target table.\n\n');
    end

    % Make sure everything is settled before we start
    pause(2);


    %%%%%%%%%%%%%%%
    %% Main loop %%
    %%%%%%%%%%%%%%%

    while true
        tic

        % If connection is lost
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% 1. Get position and orientation of the robot %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Position
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        % Orientation
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        % Update values
        pos = youbotPos - initPos;

        pos_x = round(pos(1) * mapPrec);
        pos_y = round(pos(2) * mapPrec);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% 2. Finite state machine %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %% State 'manipulation'

        % In this state, the robot will grab all the object on table,
        % without any falling on the ground and put them on the
        % target table.
        if strcmp(state, 'manipulation')

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% 2.1 Define new objective(s) %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % If the robot has accomplished his objective
            if hasAccCurrentObj

                % If there remains no objective in the list
                if length(objectiveList) == 0

                    % If there remains no points in the path
                    if length(pathList) == 0

                        % Determine next point to go
                        nextPoint = [size(occMat, 1) - pos_y + 1, pos_x];
                        
                        % Get the path to this point
                        startPoint = [size(occMat, 1) - pos_y + 1, pos_x];
                        stopPoint = [nextPoint(1), nextPoint(2)];
                        
                        goalPoint = zeros(size(occMat));
                        goalPoint(stopPoint(1), stopPoint(2)) = 1;

                        % Calculate the path with A*
                        pathList = utils.astar(startPoint(2), startPoint(1), occMat, goalPoint, pathDist);

                        % Optimize the path
                        pathList = utils.optimize(pathList);

                        % Get the path to display
                        pathDisp = pathList;
                    end
                    
                    %% Determine next action and objective to follow the path

                    % Get next objective
                    [xObj, yObj] = utils.tocart(pathList(end, 1), pathList(end, 2), size(occMat, 1));

                    % Get rotation angle to align the robot with the objective
                    a = [0 -1];
                    b = [(xObj / mapPrec) - pos(1), (yObj / mapPrec) - pos(2)];

                    rotAngl = sign((xObj / mapPrec) - pos(1)) * acos(dot(a, b) / (norm(a) * norm(b)));

                    % Set the objectives
                    objective = {'rotate', rotAngl};
                    objectiveList{end + 1} = objective;
                    
                    objective = {'forward', [(xObj / mapPrec), (yObj / mapPrec)]};
                    objectiveList{end + 1} = objective;

                    % Remove this point from the path
                    pathList(end, :) = [];
                end

                % Get the next objective
                action = objectiveList{1}{1};
                objective = objectiveList{1}{2};
                
                objectiveList(1) = [];

                % Reset the flag
                hasAccCurrentObj = false;
            end


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% 2.2 Calculate distance to objective %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % If we are in a displacement action
            if any(strcmp(action, displacementActions))
                distObj = [abs(pos(1) - objective(1)), abs(pos(2) - objective(2))];

            % If we are in a rotation action
            elseif any(strcmp(action, rotationActions))
                distObj = abs(angdiff(objective, youbotEuler(3)));
            end


            %%%%%%%%%%%%%%%%%%%%%
            %% 2.3 Set actions %%
            %%%%%%%%%%%%%%%%%%%%%

            %% Action 'forward'

            if strcmp(action, 'forward')
                forwBackVel = -forwBackVelFact * sum(distObj);
            
            %% Action 'backward'

            elseif strcmp(action, 'backward')
                forwBackVel = forwBackVelFact * sum(distObj);

            %% Action 'rotate'

            elseif strcmp(action, 'rotate')
                rotVel = rotVelFact * angdiff(objective, youbotEuler(3));

            %% Unknown action

            else
                error('Unknown action %s.', action);
            end


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% 2.4 Verification of the objective %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % If we are in a displacement action
            if any(strcmp(action, displacementActions))
                if distObj(1) < movPrecision && distObj(2) < movPrecision
                    forwBackVel = 0;

                    hasAccCurrentObj = true;

                    % Set the flag
                    removePath = true;
                end

            % If we are in a rotation action
            elseif any(strcmp(action, rotationActions))
                if distObj < rotPrecision
                    rotVel = 0;
                    
                    hasAccCurrentObj = true;
                end
            end

        %% Unknown state

        else
            error('Unknown state %s.', state);
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% 4. Update information %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Wheel velocities
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% 5. Show the map and its components %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Show the map
        utils.map(occMat, [pos_x, pos_y], pathDisp);

        % We check if we have to remove the last point
        if removePath
            pathDisp(end, :) = [];

            removePath = false;
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% 5. Physic verifications %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Make sure that we do not go faster than the physics simulation (it is useless to go faster)
        elapsed = toc;
        timeleft = timestep - elapsed;

        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end
end
