% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function main()

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


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Resources initialization %%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	% Map dimensions
	mapWidth = 15;
	mapHeight = 15;
	mapPrec = 5;

	% SLAM (for navigation)
	slam = false;

	% Timestep of the simulator
	timestep = .05;

	% Map and robot instance
	map = classes.MapManager(mapWidth, mapHeight, mapPrec);
	robot = classes.RobotController();


	%%%%%%%%%%%%%%%%
	%% Navigation %%
	%%%%%%%%%%%%%%%%
	
	navigation(vrep, id, h, timestep, map, robot, slam);


	%%%%%%%%%%%%%%%%%%
	%% Manipulation %%
	%%%%%%%%%%%%%%%%%%

	manipulation(vrep, id, h, timestep, map, robot);
end
