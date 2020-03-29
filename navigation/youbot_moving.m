% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function youbot_moving()

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
    pause(.2);

    % The time step the simulator is using (your code should run close to it).
    timestep = .05;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Values initialisation %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%

    disp('Starting robot');
    
    %% Parameters for controlling the youBot's wheels
    
    % At each iteration, those values will be set for the wheels
    forwBackVel = 0; % Move straight ahead
    rightVel = 0; % Go sideways
    rotateRightVel = 0; % Rotate 

    %% Map and meshgrid

    meshSize = 0.1;

    [X, Y] = meshgrid(-5:meshSize:5, -5:meshSize:5);
    X = reshape(X, 1, []);
    Y = reshape(Y, 1, []);
    
    map = occupancyMap(30, 30, 10);

    %% Initial position and orientation
    
    % Position
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Orientation
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);

    % Initial values
    initPos = youbotPos - [15 15 0];
    initA1 = youbotEuler(1);
    initA2 = youbotEuler(2);
    initA3 = youbotEuler(3);

    %% Finite state machine

    % Initial state of the FSM
    state = 'navigation';

    % Initial values for state 'navigation'
    action = 'forward'
    has_rotate = false;

    fprintf('\n***************************************\n');
    fprintf('* The robot begins state "%s" *\n', state);
    fprintf('***************************************\n\n');

    fprintf('In this state, he will discover the map and build an appropriate representation.\n\n');

    fprintf('List of actions\n');
    fprintf('---------------\n');
    fprintf('First action is : %s\n', action);

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

        setOccupancy(map, inValue, 0);

        % Transforming 'pts' to absolute reference
        inPts = homtrans(trf, [pts(1, cts); pts(2, cts); zeros(1, size(pts(1, cts), 2))]);
        inPts = transpose([inPts(1, :); inPts(2, :)]);

        setOccupancy(map, inPts, 1);

        % Show the map
        subplot(2, 1, 1);
        show(map);
        
        subplot(2, 1, 2);
        plot(round(pos(1), 1), round(pos(2), 2), 'or', 'markersize', 10);
        hold on;
        plot(round(inValue(:, 1), 1), round(inValue(:, 2), 1), '.g', 'markersize', 10);
        hold on;
        plot(round(inPts(:, 1), 1), round(inPts(:, 2), 1), '.r', 'markersize', 10);
        hold off;

        drawnow;

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Finite state machine %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        %% State 'navigation'

        % In this state, the robot will explore the map in order to
        % construct an appropriate representation.
        if strcmp(state, 'navigation')

            %%%%%%%%%%%%%%%%%%%%%%
            %% Action 'forward' %%
            %%%%%%%%%%%%%%%%%%%%%%

            if strcmp(action, 'forward')
                
                % Make the robot drive with a constant speed (very simple controller, likely to overshoot). 
                % The speed is - 1 m/s, the sign indicating the direction to follow. Please note that the robot has
                % limitations and cannot reach an infinite speed. 
                forwBackVel = -1;
                
                % Stop when the robot is close to y = - 6.5. The tolerance has been determined by experiments: if it is too
                % small, the condition will never be met (the robot position is updated every 50 ms); if it is too large,
                % then the robot is not close enough to the position (which may be a problem if it has to pick an object,
                % for example). 
                if abs(youbotPos(2) + 6.5) < .1
                    forwBackVel = 0;
                    action = 'backward';
                    
                    fprintf('Next action is : %s\n', action);
                end

            
            %%%%%%%%%%%%%%%%%%%%%%%
            %% Action 'backward' %%
            %%%%%%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'backward')

                % A speed which is a function of the distance to the destination can also be used. This is useful to avoid
                % overshooting : with this controller, the speed decreases when the robot approaches the goal. 
                % Here, the goal is to reach y = -4.5. 
                forwBackVel = - 2 * (youbotPos(2) + 4.5);
                
                % Stop when the robot is close to y = 4.5. 
                if abs(youbotPos(2) + 4.5) < .1
                    forwBackVel = 0;
                    action = 'right';
                    
                    fprintf('Next action is : %s\n', action);
                end


            %%%%%%%%%%%%%%%%%%%%
            %% Action 'right' %%
            %%%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'right')
                
                % Move sideways, again with a proportional controller (goal: x = - 4.5). 
                rightVel = - 2 * (youbotPos(1) + 4.5);
                
                % Stop at x = - 4.5
                if abs(youbotPos(1) + 4.5) < .1
                    rightVel = 0;
                    action = 'rotateRight';
                    
                    fprintf('Next action is : %s\n', action);
                end


            %%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Action 'rotateRight' %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'rotateRight')

                % Rotate until the robot has an angle of -pi/2 (measured with respect to the world's reference frame). 
                % Again, use a proportional controller. In case of overshoot, the angle difference will change sign, 
                % and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots, 
                % the anguler speed becomes negative). 
                % youbotEuler(3) is the rotation around the vertical axis. 
                rotateRightVel = angdiff(- pi / 2, youbotEuler(3)); % angdiff ensures the difference is between -pi and pi. 
                
                % Stop when the robot is at an angle close to -pi/2. 
                if abs(angdiff(- pi / 2, youbotEuler(3))) < .002
                    rotateRightVel = 0;
                    action = 'finished';
                    
                    fprintf('Next action is : %s\n', action);
                end

            
            %%%%%%%%%%%%%%%%%%%%%
            %% Action 'rotate' %%
            %%%%%%%%%%%%%%%%%%%%%

            elseif strcmp(action, 'rotate')

                % Rotate until the robot has an angle of 2 pi (measured with respect to the world's reference frame). 
                % Again, use a proportional controller. In case of overshoot, the angle difference will change sign, 
                % and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots, 
                % the anguler speed becomes negative). 
                % youbotEuler(3) is the rotation around the vertical axis. 
                rotateRightVel = angdiff(2 * pi, youbotEuler(3)); % angdiff ensures the difference is between -pi and pi. 
                
                % Stop when the robot is at an angle close to 2 pi. 
                if abs(angdiff(2 * pi, youbotEuler(3))) < .002
                    rotateRightVel = 0;
                    action = 'finished';
                    
                    fprintf('Next action is : %s\n', action);
                end


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
                error('Unknown action %s.', action)
            end
        end


        %%%%%%%%%%%%%%%%%%%%%%%%
        %% Update information %%
        %%%%%%%%%%%%%%%%%%%%%%%%
        
        % Wheel velocities.
        h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);


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
