% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function main()

	%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Values initialization %%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%

	% Scene name (for exportation)
	sceneName = 'house_2020';

	% Map dimensions
	mapWidth = 15;
	mapHeight = 15;
	mapPrec = 5;

	% Navigation difficulty ('easy', 'medium')
	navDifficulty = 'easy';

	% Manipulation difficulty ('easy' or 'hard')
	manDifficulty = 'easy';

	% Information about tables to detect
	numTables = 3;
	radiusTables = 0.4;

	% Timestep of the simulator
	timestep = 0.05;

	% Number of seconds between two scan matching
	secBetScan = 30;


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Simulator initialization %%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	fprintf('Program started.\n');

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

	% Retrieve all handles
	h = youbot_init(vrep, id);
	h = youbot_hokuyo_init(vrep, h);

	% Reset possible previous figure(s)
	clf;

	% Make sure everything is settled before we start (wait for the simulation to start)
	pause(0.2);


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Objects initialization %%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	map = classes.MapManager(mapWidth, mapHeight, mapPrec, numTables, radiusTables, navDifficulty);
	robot = classes.RobotController(vrep, id, h, mapPrec, secBetScan, timestep, navDifficulty);


	%%%%%%%%%%%%%%%%
	%% Milestones %%
	%%%%%%%%%%%%%%%%

	navigation(vrep, id, timestep, map, robot, sceneName);
	manipulation(vrep, id, timestep, map, robot, manDifficulty, sceneName);


	%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Simulator termination %%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%

	vrep.simxStopSimulation(id, vrep.simx_opmode_oneshot_wait);
	vrep.simxFinish(-1);

	fprintf('Program terminated.\n');
end
