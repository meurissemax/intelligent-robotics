% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function navigation_a()

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

    movPrecision = 0.1;
    rotPrecision = 0.002;

    %% Map and meshgrid

    meshSize = 0.1;

    [X, Y] = meshgrid(-5:meshSize:5, -5:meshSize:5);
    X = reshape(X, 1, []);
    Y = reshape(Y, 1, []);

    mapSize = [15, 15];
    mapPrec = 10;

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

    %% Finite state machine

    % Initial state of the FSM
    state = 'navigation';

    % List of actions classed by categories
    displacementActions = {'forward', 'backward', 'left', 'right'};
    rotationActions = {'rotate'};


    %%%%%%%%%%%%%%%%%%%%
    %% Objective list %%
    %%%%%%%%%%%%%%%%%%%%

    % Define the objective list
    objectiveList = {};
    
    % Initialise the flag for the current objective
    hasAccCurrentObj = true;


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
        A1 = youbotEuler(1) - initA1;
        A2 = youbotEuler(2) - initA2;
        A3 = youbotEuler(3) - initA3;

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

        [x_free, y_free] = find(occMat == 0);
        plot(y_free, x_free, '.g');
        hold on;

        [x_occ, y_occ] = find(occMat == 1);
        plot(y_occ, x_occ, '*r');
        hold on;
        
        plot(round(pos(1) * mapPrec), round(pos(2) * mapPrec), 'or', 'markersize', 10);
        hold off;

        axis([0, mapSize(1) * 10 * 2, 0, mapSize(2) * 10 * 2]);
        title('Explored map');

        drawnow;


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

                % If there remains no objective in the list
                if length(objectiveList) == 0

                    % If the whole map is explored, we can stop the robot.
                    if utils.is_map_explored(occMat, mapSize, mapPrec)
                        action = 'finished';
                        objective = [0, 0];

                    % If the whole map is not yet fully explored,
                    % we define new objective(s).
                    else
                        % Determine next nearest inexplorated point
                        mapInflated = copy(map);
                        inflate(mapInflated, 0.2);
                        occMatInf = occupancyMatrix(mapInflated, 'ternary');
                        
                        nextPoint = utils.find_next([round(pos(1) * mapPrec), round(pos(2) * mapPrec)], occMatInf);
                        
                        % Get the path to this point
                        startPoint = [round(pos(1) * 10), round(pos(2) * 10)];
                        stopPoint = [nextPoint(1), nextPoint(2)];
                        
                        goalPoint = zeros(size(occMat));
                        goalPoint(stopPoint(1), stopPoint(2)) = 1;
                        
                        path = utils.a_star(startPoint(1), startPoint(2), occMatInf, goalPoint, 15);
                        
                        % Plot the path
                        subplot(2, 1, 2);

                        plot(y_free, x_free, '.g');
                        hold on;
                        
                        plot(y_occ, x_occ, '*r');
                        hold on;
                        
                        plot(round(pos(1) * mapPrec), round(pos(2) * mapPrec), 'or', 'markersize', 10);
                        hold on;

                        plot(nextPoint(1), nextPoint(2), '*b', 'markersize', 10);
                        hold on;
                        
                        line(path(:, 1), path(:, 2), 'LineWidth', 2);
                        hold off;

                        axis([0, mapSize(1) * 10 * 2, 0, mapSize(2) * 10 * 2]);
                        title('Next objective');
                        
                        drawnow;

                        % Determine the list of objectives to follow the path
                        for i = 1:size(path, 1)
                            rotAngl = - pi / 2;

                            objective = {'rotate', rotAngl};
                            objectiveList{end + 1} = objective;

                            objective = {'forward', [(path(i, 1) / 10), (path(i, 2) / 10)]};
                            objectiveList{end + 1} = objective;
                        end

                        % Get the following objective
                        action = objectiveList{1}{1};
                        objective = objectiveList{1}{2};
                    
                        objectiveList(1) = [];
                    end

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
                    fprintf('[position : (%f, %f)] - Next action is "%s" with objective %f\n', pos(1), pos(2), action, objective);
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
                forwBackVel = -1 * sum(distObj);

            
            %%%%%%%%%%%%%%%%%%%%%%%
            %% Action 'backward' %%
            %%%%%%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'backward')

                % Set the forward/backward velocity of the robot.
                forwBackVel = 1 * sum(distObj);

            
            %%%%%%%%%%%%%%%%%%%
            %% Action 'left' %%
            %%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'left')
                
                % Set the left/right velocity of the robot.
                leftRightVel = 1 * sum(distObj);


            %%%%%%%%%%%%%%%%%%%%
            %% Action 'right' %%
            %%%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'right')
                
                % Set the left/right velocity of the robot.
                leftRightVel = -1 * sum(distObj);


            %%%%%%%%%%%%%%%%%%%%%
            %% Action 'rotate' %%
            %%%%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'rotate')

                % Set the rotation velocity of the robot.
                rotVel = angdiff(objective, youbotEuler(3)) / 3;


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
                if abs(angdiff(objective, youbotEuler(3))) < rotPrecision
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
