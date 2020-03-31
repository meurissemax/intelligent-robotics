% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function navigation()

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Simulator initialisation %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    disp('Program started');

    % Connection to the simulator.
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

    % If the connection has failed.
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        
        vrep.delete();
        return;
    end

    % If connection successed.
    fprintf('Connection %d to remote API server open.\n', id);
    
    % Make sure we close the connection whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % Start the simulation.
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, mostly the Hokuyo.
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Make sure everything is settled before we start (wait for the simulation to start).
    pause(0.2);

    % The time step the simulator is using (your code should run close to it).
    timestep = 0.05;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Values initialisation %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%

    disp('Starting robot');
    
    %% Parameters for controlling the youBot's wheels
    
    % At each iteration, those values will be set for the wheels
    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;

    %% Parameters for the movements

    movPrecision = 0.6;
    rotPrecision = 0.005;

    %% Map and meshgrid

    meshSize = 0.1;

    [X, Y] = meshgrid(-5:meshSize:5, -5:meshSize:5);
    X = reshape(X, 1, []);
    Y = reshape(Y, 1, []);

    mapSize = [15, 15];
    mapPrec = 5;

    map = occupancyMap(mapSize(1) * 2, mapSize(2) * 2, mapPrec);

    %% Initial position and orientation
    
    % Position
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Orientation
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);

    % Initial values
    initPos = youbotPos - [mapSize(1), mapSize(2), 0];
    initA1 = youbotEuler(1);
    initA2 = youbotEuler(2);
    initA3 = youbotEuler(3);

    pos = youbotPos - initPos;
    setOccupancy(map, [pos(1), pos(2)], 0);

    %% Initialise the position of the robot in the occupancy map
    for x = pos(1) - 2*(1/mapPrec):(1/mapPrec):pos(1) + 2*(1/mapPrec)
        for y = pos(2) - 2*(1/mapPrec):(1/mapPrec):pos(2) + 2*(1/mapPrec)
            if x >= (1/mapPrec) && y >= (1/mapPrec) && x <= mapSize(1) * 2 && y <= mapSize(2) * 2 && (x ~= pos(1) || y ~= pos(2))
                setOccupancy(map, [x y], 0);
            end
        end
    end

    %% Finite state machine

    % Initial state of the FSM
    state = 'navigation';

    % List of actions classed by categories
    displacementActions = {'forward', 'backward', 'left', 'right'};
    rotationActions = {'rotate'};


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Objective list and path %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Define the objective list
    objectiveList = {
        {'rotate', pi}
    };
    
    action = objectiveList{1}{1};
    objective = objectiveList{1}{2};
    
    objectiveList(1) = [];
    
    % Initialise the flag for the current objective
    hasAccCurrentObj = false;

    % Define the path and its metric
    path = [];
    metric = 'shortest';


    %%%%%%%%%%%%%%%%%%%%%%%%%
    %% Display information %%
    %%%%%%%%%%%%%%%%%%%%%%%%%

    fprintf('\n***************************************\n');
    fprintf('* The robot begins state "%s" *\n', state);
    fprintf('***************************************\n\n');

    fprintf('In this state, he will discover the map and build an appropriate representation.\n\n');

    fprintf('List of actions\n');
    fprintf('---------------\n');
    
    % We reset possible previous figure(s)
    clf;

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


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Position and orientation of the robot %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Position
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        % Orientation
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        % Update values
        pos = youbotPos - initPos;
        A1 = youbotEuler(1);
        A2 = youbotEuler(2);
        A3 = youbotEuler(3);

        % Transformation to robot absolute position
        trf = transl(pos) * trotx(A1) * troty(A2) * trotz(A3);

        % Point obtention (from Hokuyo)
        [pts, cts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
        simplifiedPoly = utils.simplify_polygon([h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1); h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)]);
        in = inpolygon(X, Y, simplifiedPoly(1, :), simplifiedPoly(2, :));

        % Transforming inside points to absolute reference
        inValue = homtrans(trf, [X(in); Y(in); zeros(1, size(X(in), 2))]);
        inValue = transpose([inValue(1, :); inValue(2, :)]);

        % Transforming 'pts' to absolute reference
        inPts = homtrans(trf, [pts(1, cts); pts(2, cts); zeros(1, size(pts(1, cts), 2))]);
        inPts = transpose([inPts(1, :); inPts(2, :)]);

        % Update the map
        setOccupancy(map, inValue, 0);
        setOccupancy(map, inPts, 1);

        % Get the occupancy matrix
        occMat = occupancyMatrix(map, 'ternary');

        % Show the map
        subplot(2, 1, 1);
        
        [i_free, j_free] = find(occMat == 0);
        [x_free, y_free] = utils.mat_to_cart(i_free, j_free, size(occMat, 1));
        plot(x_free, y_free, '.g');
        hold on;

        [i_occ, j_occ] = find(occMat == 1);
        [x_occ, y_occ] = utils.mat_to_cart(i_occ, j_occ, size(occMat, 1));
        plot(x_occ, y_occ, '*r');
        hold on;
        
        plot(round(pos(1) * mapPrec), round(pos(2) * mapPrec), 'or', 'markersize', 10);
        hold off;

        %axis([0, mapSize(1) * mapPrec * 2, 0, mapSize(2) * mapPrec * 2]);
        title('Explored map');

        drawnow;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Check if robot is stuck %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        inFront = round(size(inPts, 1) / 2);
        distFront = pdist2([pos(1), pos(2)], [inPts(inFront, 1), inPts(inFront, 2)], 'euclidean');

        if distFront < 0.4
            fprintf('ROBOT IS STUCK, RESET\n');
            hasAccCurrentObj = false;

            d1 = abs(angdiff(youbotEuler(3), pi / 2));
            d2 = abs(angdiff(youbotEuler(3), - pi / 2));
            d3 = abs(angdiff(youbotEuler(3), 0));
            d4 = abs(angdiff(youbotEuler(3), pi));

            hori = 0;
            vert = 0;

            mind = min([d1, d2, d3, d4]);

            if mind == d1
                hori = -1;
                fprintf('D1\n');
            elseif mind == d2
                hori = 1;
                fprintf('D2\n');
            elseif mind == d3
                vert = 1;
                fprintf('D3\n');
            else
                vert = -1;
                fprintf('D4\n');
            end

            action = 'backward';
            objective = [pos(1) + hori, pos(2) + vert];
            fprintf('Stuck in : (%f, %f). Backward in (%f, %f).\n', youbotPos(1), youbotPos(2), objective(1), objective(2));

            metric = 'shortest';
            objectiveList = {};
            path = [];
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Finite state machine %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        %% State 'navigation'

        % In this state, the robot will explore the map in order to
        % construct an appropriate representation.
        if strcmp(state, 'navigation')

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Define new objective(s) %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % If the robot has accomplished his objective.
            if hasAccCurrentObj

                % If there remains no objective in the list.
                if length(objectiveList) == 0

                    % If there remains no points in the path.
                    if length(path) == 0

                        % If the whole map is explored, we can stop the robot.
                        if utils.is_map_explored(occMat, mapSize, mapPrec)
                            action = 'finished';
                            objective = [0, 0];

                            fprintf('MAP EXPLORED !!\n');

                            break;

                        % If the whole map is not yet fully explored,
                        % we define a new path.
                        else
                            % Determine next nearest inexplorated point
                            mapInflated = copy(map);
                            inflate(mapInflated, 0.6);
                            occMatInf = occupancyMatrix(mapInflated, 'ternary');
                            
                            nextPoint = utils.find_next([size(occMat, 1) - round(pos(2) * mapPrec) + 1, round(pos(1) * mapPrec)], occMatInf, 15, metric);
                            [xNext, yNext] = utils.mat_to_cart(nextPoint(1), nextPoint(2), size(occMat, 1));
                            fprintf('next point is : %f %f\n', xNext, yNext);

                            metric = 'shortest';
                            
                            % Get the path to this point
                            startPoint = [size(occMat, 1) - round(pos(2) * mapPrec) + 1, round(pos(1) * mapPrec)];
                            stopPoint = [nextPoint(1), nextPoint(2)];
                            
                            goalPoint = zeros(size(occMat));
                            goalPoint(stopPoint(1), stopPoint(2)) = 1;

                            % Set stop to 0
                            occMatInf(stopPoint(1), stopPoint(2)) = 0;

                            % Set neighbors of start to 0
                            for x = startPoint(1) - 1:1:startPoint(1) + 1
                                for y = startPoint(2) - 1:1:startPoint(2) + 1
                                    if x >= 1 && y >= 1 && x <= size(occMatInf, 1) && y <= size(occMatInf, 2) && (x ~= startPoint(1) || y ~= startPoint(2))
                                        occMatInf(x, y) = 0;
                                    end
                                end
                            end

                            subplot(2, 1, 2);

                            [i_free, j_free] = find(occMatInf == 0);
                            [x_free, y_free] = utils.mat_to_cart(i_free, j_free, size(occMat, 1));
                            plot(x_free, y_free, '.g');
                            hold on;

                            [i_occ, j_occ] = find(occMatInf == 1);
                            [x_occ, y_occ] = utils.mat_to_cart(i_occ, j_occ, size(occMat, 1));
                            plot(x_occ, y_occ, '*r');
                            hold on;

                            plot(round(pos(1) * mapPrec), round(pos(2) * mapPrec), 'or', 'markersize', 10);
                            hold on;

                            plot(xNext, yNext, '*b', 'markersize', 10);
                            hold off;

                            drawnow;

                            path = utils.a_star(startPoint(2), startPoint(1), occMatInf, goalPoint, 5);

                            if path(1) == Inf
                                fprintf('PATH = INF');
                                metric = 'shortest';
                                path(1, :) = [];

                                continue;
                            end

                            path = utils.optimize_path(path);
                            path(end, :) = [];
                            
                            % Plot the path
                            subplot(2, 1, 2);

                            [i_free, j_free] = find(occMatInf == 0);
                            [x_free, y_free] = utils.mat_to_cart(i_free, j_free, size(occMat, 1));
                            plot(x_free, y_free, '.g');
                            hold on;

                            [i_occ, j_occ] = find(occMatInf == 1);
                            [x_occ, y_occ] = utils.mat_to_cart(i_occ, j_occ, size(occMat, 1));
                            plot(x_occ, y_occ, '*r');
                            hold on;
                            
                            plot(round(pos(1) * mapPrec), round(pos(2) * mapPrec), 'or', 'markersize', 10);
                            hold on;

                            plot(xNext, yNext, '*b', 'markersize', 10);
                            hold on;

                            for i = 1:size(path, 1)
                                [xObj, yObj] = utils.mat_to_cart(path(i, 1), path(i, 2), size(occMat, 1));

                                plot(xObj, yObj, '+m', 'markersize', 5);
                                hold on;
                            end

                            hold off;

                            %axis([0, mapSize(1) * mapPrec * 2, 0, mapSize(2) * mapPrec * 2]);
                            title('Next objective');
                            
                            drawnow;
                        end
                    end
                    
                    % Determine next objective to follow the path
                    if length(path) == 0
                        fprintf('PATH EMPTY\n');
                    end

                    [xObj, yObj] = utils.mat_to_cart(path(end, 1), path(end, 2), size(occMat, 1));

                    a = [0 -1];
                    b = [(xObj / mapPrec) - pos(1), (yObj / mapPrec) - pos(2)];

                    rotAngl = sign((xObj / mapPrec) - pos(1)) * acos(dot(a, b) / (norm(a) * norm(b)));

                    objective = {'rotate', rotAngl};
                    objectiveList{end + 1} = objective;

                    objective = {'forward', [(xObj / mapPrec), (yObj / mapPrec)]};
                    objectiveList{end + 1} = objective;

                    path(end, :) = [];

                    % Get the following objective
                    action = objectiveList{1}{1};
                    objective = objectiveList{1}{2};
                
                    objectiveList(1) = [];

                % If there is still objective(s) in the list
                else
                    action = objectiveList{1}{1};
                    objective = objectiveList{1}{2};
                    
                    objectiveList(1) = [];
                end

                % Reset the flag
                hasAccCurrentObj = false;

                % Print action and objective information
                if strcmp(action, 'finished')
                    fprintf('[position : (%f, %f)] - Next action is "%s"\n', pos(1), pos(2), action);
                elseif any(strcmp(action, displacementActions))
                    fprintf('[position : (%f, %f)] - Next action is "%s" with objective (%f, %f)\n', pos(1), pos(2), action, objective(1), objective(2));
                elseif any(strcmp(action, rotationActions))
                    fprintf('[orientation : %f] - Next action is "%s" with objective %f\n', rad2deg(youbotEuler(3)), action, rad2deg(objective));
                end
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Distance to objective %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%

            % If we are in a displacement action
            if any(strcmp(action, displacementActions))
                distObj = [abs(pos(1) - objective(1)), abs(pos(2) - objective(2))];

            % If we are in a rotation action
            elseif any(strcmp(action, rotationActions))
                distObj = abs(angdiff(objective, youbotEuler(3)));

            % If we are in an unknown action
            else
                distObj = 0;
            end


            %%%%%%%%%%%%%%%%%%%%%%
            %% Action 'forward' %%
            %%%%%%%%%%%%%%%%%%%%%%

            if strcmp(action, 'forward')
                
                % Set the forward/backward velocity of the robot.
                forwBackVel = -3 * sum(distObj);

            
            %%%%%%%%%%%%%%%%%%%%%%%
            %% Action 'backward' %%
            %%%%%%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'backward')

                % Set the forward/backward velocity of the robot.
                forwBackVel = 3 * sum(distObj);

            
            %%%%%%%%%%%%%%%%%%%
            %% Action 'left' %%
            %%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'left')
                
                % Set the left/right velocity of the robot.
                leftRightVel = 3 * sum(distObj);


            %%%%%%%%%%%%%%%%%%%%
            %% Action 'right' %%
            %%%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'right')
                
                % Set the left/right velocity of the robot.
                leftRightVel = -3 * sum(distObj);


            %%%%%%%%%%%%%%%%%%%%%
            %% Action 'rotate' %%
            %%%%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'rotate')

                % Set the rotation velocity of the robot.
                rotVel = angdiff(objective, youbotEuler(3));


            %%%%%%%%%%%%%%%%%%%%%%%
            %% Action 'finished' %%
            %%%%%%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'finished')
                pause(3);

                break


            %%%%%%%%%%%%%%%%%%%%
            %% Unknown action %%
            %%%%%%%%%%%%%%%%%%%%

            else
                error('Unknown action %s.', action);
            end


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Verification of the objective %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % If we are in a displacement action.
            if any(strcmp(action, displacementActions))
                if distObj(1) < movPrecision && distObj(2) < movPrecision
                    forwBackVel = 0;
                    leftRightVel = 0;

                    hasAccCurrentObj = true;
                end

            % If we are in a rotation action.
            elseif any(strcmp(action, rotationActions))
                if distObj < rotPrecision
                    rotVel = 0;
                    
                    hasAccCurrentObj = true;
                end

            % If we are in an unknown action.
            else
                action = 'finished';
            end

        %% Unknown state

        else
            error('Unknown state %s.', state);
        end


        %%%%%%%%%%%%%%%%%%%%%%%%
        %% Update information %%
        %%%%%%%%%%%%%%%%%%%%%%%%
        
        % Wheel velocities.
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);


        %%%%%%%%%%%%%%%%%%%%%%%%%
        %% Physic verification %%
        %%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Make sure that we do not go faster than the physics simulation (it is useless to go faster). 
        elapsed = toc;
        timeleft = timestep - elapsed;

        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end
end
