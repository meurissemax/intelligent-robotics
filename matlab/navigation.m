% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

%% Milestone Navigation - (a)

function navigation()

    %%%%%%%%%%%%%%%%%%%%%%%
    %% General variables %%
    %%%%%%%%%%%%%%%%%%%%%%%

    % The time step the simulator is using
    timestep = 0.05;

    % Movements tolerances
    movPrecision = 0.3;
    rotPrecision = 0.005;

    % Map parameters
    mapSize = [15, 15];
    mapPrec = 5;

    % Parameters when the robot is stuck
    stuckTresh = 0.8;
    stuckDist = 0.5;

    % Velocity multipliers
    forwBackVelFact = 3;
    rotVelFact = 1;

    % Map inflation factor
    mapInflatedFact = 0.6;

    % Exploration threshold (percentage of points to visit)
    explThresh = 0.99;

    % Possible connection distance between points for A*
    pathDist = 10;

    % Minimal distance to keep between points in path
    optiDist = 15;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Simulator initialisation %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    fprintf('Program started\n');

    % Connection to the simulator
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

    % If the connection has failed
    if id < 0
        fprintf('Failed connecting to remote API server. Exiting.\n');
        
        vrep.delete();
        return;
    end

    % If connection successed
    fprintf('Connection %d to remote API server open.\n', id);

    % Make sure we close the connection whenever the script is interrupted
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % Start the simulation
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, mostly the Hokuyo
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Make sure everything is settled before we start (wait for the simulation to start)
    pause(0.2);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Values initialisation %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%

    fprintf('Starting robot\n');

    % Parameters for controlling the youBot's wheels
    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;

    % Meshgrid
    meshSize = 1 / mapPrec;

    [X, Y] = meshgrid(-5:meshSize:5, -5:meshSize:5);
    X = reshape(X, 1, []);
    Y = reshape(Y, 1, []);

    % Map
    map = occupancyMap(mapSize(1) * 2, mapSize(2) * 2, mapPrec);

    % Position
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Orientation
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);

    initPos = youbotPos - [mapSize(1), mapSize(2), 0];

    % Set the position of the robot and his neighboorhood to 0 (free position)
    pos = youbotPos - initPos;
    setOccupancy(map, [pos(1), pos(2)], 0);

    dx = 1 / mapPrec;
    radius = 2;
    xLimits = map.XWorldLimits;
    yLimits = map.YWorldLimits;

    for x = (pos(1) - radius * dx):dx:(pos(1) + radius * dx)
        for y = (pos(2) - radius * dx):dx:(pos(2) + radius * dx)
            if x >= xLimits(1) && y >= yLimits(1) && x <= xLimits(2) && y <= yLimits(2) && (x ~= pos(1) || y ~= pos(2))
                setOccupancy(map, [x, y], 0);
            end
        end
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% FSM, objective list and path %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Initial state of the Finite State Machine (FSM)
    state = 'navigation';

    % List of actions classed by categories
    displacementActions = {'forward', 'backward'};
    rotationActions = {'rotate'};
    
    % Initialise the objective list
    objectiveList = {};

    % Initialise the first action
    objectiveList{1} = {'rotate', youbotEuler(3) + pi};

    % Initialise the flag for the current objective
    hasAccCurrentObj = true;

    % Define the path and its metric
    pathList = [];
    pathMetric = 'longest';


    %%%%%%%%%%%%%%%%%%%%%%%%%
    %% Display information %%
    %%%%%%%%%%%%%%%%%%%%%%%%%

    % Console debug
    fprintf('\n***************************************\n');
    fprintf('* The robot begins state "%s" *\n', state);
    fprintf('***************************************\n\n');

    if strcmp(state, 'navigation')
        fprintf('In this state, he will discover the map and build an appropriate representation.\n\n');
    end

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

        % Transformation to robot absolute position
        trf = transl(pos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));

        % Point obtention (from Hokuyo)
        [pts, cts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
        simplifiedPoly = utils.simplifyPolygon([h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1); h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)]);
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


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% 2. Check if robot is stuck %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Distance to the nearest element in front of the robot
        inFront = round(size(inPts, 1) / 2);
        distFront = pdist2([pos(1), pos(2)], [inPts(inFront, 1), inPts(inFront, 2)], 'euclidean');

        % If robot is too close to something
        if distFront < stuckTresh

            % Get the orientation
            distAngl = [
                abs(angdiff(youbotEuler(3), pi / 2)),
                abs(angdiff(youbotEuler(3), - pi / 2)),
                abs(angdiff(youbotEuler(3), 0)),
                abs(angdiff(youbotEuler(3), pi))
            ];
            
            minDistAngl = min(distAngl);

            % Get the direction where the robot should backward
            dx = 0;
            dy = 0;

            if minDistAngl == distAngl(1)
                dx = -stuckDist;
            elseif minDistAngl == distAngl(2)
                dx = stuckDist;
            elseif minDistAngl == distAngl(3)
                dy = stuckDist;
            else
                dy = -stuckDist;
            end

            % Define new action, new objective and reset the path
            action = 'backward';
            objective = [pos(1) + dx, pos(2) + dy];
            
            hasAccCurrentObj = false;
            
            objectiveList = {};

            pathList = [];
            pathMetric = 'shortest';
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% 3. Finite state machine %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %% State 'navigation'

        % In this state, the robot will explore the map in order to
        % construct an appropriate representation.
        if strcmp(state, 'navigation')

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% 3.1 Define new objective(s) %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % If the robot has accomplished his objective
            if hasAccCurrentObj

                % If there remains no objective in the list
                if length(objectiveList) == 0

                    % If there remains no points in the path
                    if length(pathList) == 0

                        % If the whole map is explored, we can stop the robot
                        if utils.isExplored(occMat, mapSize, mapPrec, explThresh)
                            fprintf('The map has been fully explored !\n');
                            
                            pause(3);
                            break;

                        % If the whole map is not yet fully explored,
                        % we define a new path.
                        else
                            % Determine next nearest inexplorated point
                            mapInflated = copy(map);
                            inflate(mapInflated, mapInflatedFact);
                            occMatInf = occupancyMatrix(mapInflated, 'ternary');

                            from_x = round(inPts(inFront, 1) * mapPrec);
                            from_y = round(inPts(inFront, 2) * mapPrec);

                            nextPoint = utils.findNext([size(occMat, 1) - from_y + 1, from_x], occMatInf, 15, pathMetric);

                            % If we can not find new point, the map is probably explored
                            if nextPoint == Inf
                                fprintf('No possible next point to explore. The map has been probably fully explored.\n');
                                
                                pause(3);
                                break;
                            end

                            % Set the next metric
                            pathMetric = 'shortest';
                            
                            % Get the path to this point
                            startPoint = [size(occMat, 1) - pos_y + 1, pos_x];
                            stopPoint = [nextPoint(1), nextPoint(2)];
                            
                            goalPoint = zeros(size(occMat));
                            goalPoint(stopPoint(1), stopPoint(2)) = 1;

                            % Set stop point to 0 (to be sure that A* can reach it)
                            occMatInf(stopPoint(1), stopPoint(2)) = 0;
                            
                            % Set neighborhood of start point to 0 (to be sure that A* can begin)
                            for x = startPoint(1) - 1:1:startPoint(1) + 1
                                for y = startPoint(2) - 1:1:startPoint(2) + 1
                                    if x >= 1 && y >= 1 && x <= size(occMatInf, 1) && y <= size(occMatInf, 2) && (x ~= startPoint(1) || y ~= startPoint(2))
                                        occMatInf(x, y) = 0;
                                    end
                                end
                            end

                            % Calculate the path with A*
                            pathList = utils.AStar(startPoint(2), startPoint(1), occMatInf, goalPoint, pathDist);

                            hasFinished = false;
                            reduceInflate = mapInflatedFact - 0.1;

                            % If no path can be found, re try with more flexible map (less inflate)
                            while pathList(1) == Inf
                                mapInflated = copy(map);
                                inflate(mapInflated, reduceInflate);
                                occMatInf = occupancyMatrix(mapInflated, 'ternary');

                                pathList = utils.AStar(startPoint(2), startPoint(1), occMatInf, goalPoint, pathDist);

                                reduceInflate = reduceInflate - 0.1;

                                % If inflate factor reach 0, not path is possible so stop the simulation
                                if reduceInflate <= 0
                                    fprintf('Unable to determine a path. Mapping will stop here.\n');

                                    hasFinished = true;
                                end
                            end

                            if hasFinished
                                pause(3);

                                break
                            end

                            % Optimize the path and remove the first point because it is the
                            % position of the robot itself
                            pathList = utils.optimizePath(pathList, optiDist);
                            pathList(end, :) = [];
                        end
                    end
                    
                    %% Determine next action and objective to follow the path

                    % Get next objective
                    [xObj, yObj] = utils.matToCart(pathList(end, 1), pathList(end, 2), size(occMat, 1));

                    % Get rotation angle to align the robot with the objective
                    a = [0 -1];
                    b = [(xObj / mapPrec) - pos(1), (yObj / mapPrec) - pos(2)];

                    rotAngl = sign((xObj / mapPrec) - pos(1)) * acos(dot(a, b) / (norm(a) * norm(b)));

                    % Set the objectives
                    if abs(youbotEuler(3) - rotAngl) > 0.2
                        objective = {'rotate', rotAngl};
                        objectiveList{end + 1} = objective;
                    end
                    
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

                % Print action and objective information (console debug)
                if any(strcmp(action, displacementActions))
                    fprintf('[%s] [position : (%f, %f)] -> [objective : (%f, %f)]\n', upper(action), pos(1) * mapPrec, pos(2) * mapPrec, objective(1) * mapPrec, objective(2) * mapPrec);
                elseif any(strcmp(action, rotationActions))
                    fprintf('[%s] [orientation : %f] -> [objective : %f]\n', upper(action), rad2deg(youbotEuler(3)), rad2deg(objective));
                end
            end


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% 3.2 Calculate distance to objective %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % If we are in a displacement action
            if any(strcmp(action, displacementActions))
                distObj = [abs(pos(1) - objective(1)), abs(pos(2) - objective(2))];

            % If we are in a rotation action
            elseif any(strcmp(action, rotationActions))
                distObj = abs(angdiff(objective, youbotEuler(3)));
            end


            %%%%%%%%%%%%%%%%%%%%%
            %% 3.3 Set actions %%
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
            %% 3.4 Verification of the objective %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % If we are in a displacement action
            if any(strcmp(action, displacementActions))
                if distObj(1) < movPrecision && distObj(2) < movPrecision
                    forwBackVel = 0;

                    hasAccCurrentObj = true;
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

        % Visited free points
        [i_free, j_free] = find(occMat == 0);
        [x_free, y_free] = utils.matToCart(i_free, j_free, size(occMat, 1));
        plot(x_free, y_free, '.g', 'MarkerSize', 10);
        hold on;

        % Visited occupied points
        [i_occ, j_occ] = find(occMat == 1);
        [x_occ, y_occ] = utils.matToCart(i_occ, j_occ, size(occMat, 1));
        plot(x_occ, y_occ, '.r', 'MarkerSize', 10);
        hold on;

        % Position of the robot
        plot(pos_x, pos_y, '.k', 'MarkerSize', 20);
        hold on;

        % Path and objective
        if length(pathList) > 1
            pathConverted = [];

            for i = 1:size(pathList, 1)
                [x, y] = utils.matToCart(pathList(i, 1), pathList(i, 2), size(occMat, 1));
                pathConverted(i, :) = [x, y];
            end

            plot(pathConverted(:, 1), pathConverted(:, 2), '.m', 'MarkerSize', 20);
            hold on;

            line(pathConverted(:, 1), pathConverted(:, 2), 'Color', 'm', 'LineWidth', 2);
            hold on;

            line([pos_x, pathConverted(end, 1)], [pos_y, pathConverted(end, 2)], 'Color', 'black', 'LineWidth', 2);
            hold on;
        end

        % Others
        hold off;

        title('Explored map');

        drawnow;


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% 6. Physic verifications %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Make sure that we do not go faster than the physics simulation (it is useless to go faster)
        elapsed = toc;
        timeleft = timestep - elapsed;

        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end
end
