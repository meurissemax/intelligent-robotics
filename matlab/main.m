% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function main()

    %%%%%%%%%%%%%%%%%%%%%%%
    %% General variables %%
    %%%%%%%%%%%%%%%%%%%%%%%

    % Choose whether to navigate and manipulate or
    % only navigate
    onlyNav = true;


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


    %%%%%%%%%%%%%%%%%%%%%%%
    %% Robot controlling %%
    %%%%%%%%%%%%%%%%%%%%%%%

    if onlyNav
        navigation(vrep, id, h);
    else
        manipulation(vrep, id, h);
    end
end
