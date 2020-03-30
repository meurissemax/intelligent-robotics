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

    rotPrecision = 0.002;

    %% Finite state machine

    % Initial state of the FSM
    state = 'navigation';

    % List of actions classed by categories
    rotationActions = {'rotate'};


    %%%%%%%%%%%%%%%%%%%%
    %% Objective list %%
    %%%%%%%%%%%%%%%%%%%%

    % Define the objective list
    objectiveList = {
        {'rotate', - pi / 2},
        {'finished', [0 0]}
    };

    % Get first action and objective
    action = objectiveList{1}{1};
    objective = objectiveList{1}{2};

    objectiveList(1) = [];

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


        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Finite state machine %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        %% State 'navigation'

        % In this state, the robot will explore the map in order to
        % construct an appropriate representation.
        if strcmp(state, 'navigation')

            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Distance to objective %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%

            % If we are in a rotation action
            if any(strcmp(action, rotationActions))
                distObj = abs(angdiff(objective, youbotEuler(3)));

            % If we are in an unknown action
            else
                distObj = 0;
            end


            %%%%%%%%%%%%%%%%%%%%%
            %% Action 'rotate' %%
            %%%%%%%%%%%%%%%%%%%%%

            if strcmp(action, 'rotate')

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

            % If we are in a rotation action.
            if any(strcmp(action, rotationActions))
                if distObj < rotPrecision
                    rotVel = 0;
                    
                    action = objectiveList{1}{1};
                    objective = objectiveList{1}{2};

                    objectiveList(1) = [];
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
