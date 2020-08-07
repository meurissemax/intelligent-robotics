% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function manipulation(vrep, id, h, timestep, map, robot, varargin)

	%%%%%%%%%%%%%%%%%%%%
	%% Initialization %%
	%%%%%%%%%%%%%%%%%%%%

	% To initialize the map, either the information comes
	% from the navigation phase (map and robot objects),
	% either an additional argument (scenePath) can be passed
	% to the function. In this latter case, the map is loaded
	% from a .mat file and the position of the robot is
	% initialized.

	% Get the representation of the map and position of
	% the robot
	if nargin > 6
		map.load(varargin{1});

		robot.setInitPos(robot.getRelativePositionFromGPS(vrep, id, h) - [map.mapWidth, map.mapHeight]);
		absPos = robot.getAbsolutePositionFromGPS(vrep, id, h);
	else
		absPos = robot.stopPos;
	end
	
	% Stop the robot (just to be sure)
	robot.stop(absPos);
	robot.drive(vrep, h);


	%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Display information %%
	%%%%%%%%%%%%%%%%%%%%%%%%%

	% Console debug
	fprintf('\n***************************************\n');
	fprintf('* The robot begins state manipulation *\n');
	fprintf('***************************************\n\n');
	
	fprintf('In this state, he will grasp some objects and bring them to a specific table.\n\n');
	fprintf('Robot is manipulating objects...\n');
	
	% We reset possible previous figure(s)
	clf;

	% Make sure everything is settled before we start
	pause(2);


	%%%%%%%%%%%%%%%%%%%%%%%%
	%% Tables information %%
	%%%%%%%%%%%%%%%%%%%%%%%%
	
	% Find center positions and radius of the tables
	map.findTables();


	%%%%%%%%%%%%%%%
	%% Main loop %%
	%%%%%%%%%%%%%%%

	while true
		tic

		% We check if connection is lost
		if vrep.simxGetConnectionId(id) == -1
			error('Lost connection to remote API.');
		end


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Show the map and its components %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		
		% Show the map with robot (absolute) position
		map.show(round(absPos .* map.mapPrec));


		%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Physic verifications %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%
		
		% Make sure that we do not go faster than the physics
		% simulation (it is useless to go faster)
		elapsed = toc;
		timeleft = timestep - elapsed;

		if timeleft > 0
			pause(min(timeleft, .01));
		end
	end
end
