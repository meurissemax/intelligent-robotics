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

	% Initialize action type and state of the finite state machine
	action = 'move';
	state = 'explore';
	
	% Initialize the path and the objective of the robot
	% (for 'move' actions)
	pathList = [];
	objective = [];
	hasAccCurrentObj = true;


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

	% Get information for the 'explore' state
	tablesPositions = map.tablesCenterPositions;
	tablesRadius = map.tablesRadius;


	%%%%%%%%%%%%%%%
	%% Main loop %%
	%%%%%%%%%%%%%%%

	while true
		tic

		% We check if connection is lost
		if vrep.simxGetConnectionId(id) == -1
			error('Lost connection to remote API.');
		end


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Get position and orientation of the robot %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		
		% Get position and orientation
		% TO DO : for this milestone, we can not use the
		% GPS, so we will have to change this code
		absPos = robot.getAbsolutePositionFromGPS(vrep, id, h);
		orientation = robot.getOrientation(vrep, id, h);


		%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Finite state machine %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%

		% For this milestone, we have to define some actions "(state) - action"
		% in a particular order :
		%
		% 	1. Explore each table to determine their type ('explore') - 'move'
		% 	2. Go the table with objects ('objects') - 'move'
		% 	3. Grasp an object ('grasp') - 'arm'
		% 	4. Go the objective table ('objective') - 'move'
		% 	5. Drop grasped object ('drop') - 'arm'
		%
		% Robot has to repeat operations 2 to 5 until milestone
		% is finished.

		%%%%%%%%%%%%%%%%%
		% 'move' action %
		%%%%%%%%%%%%%%%%%

		if strcmp(action, 'move')

			%%%%%%%%%%%%%%%%%%%
			% 'explore' state %
			%%%%%%%%%%%%%%%%%%%

			if strcmp(state, 'explore')

				% If the path is empty, either all tables
				% have been explored, or there remains tables
				% to explore

				if isempty(pathList)

					% If there is no more table, all tables have
					% been explore, we can move to next state,
					% else we have to define path to next table

					if isempty(tablesPositions)
						action = 'move';
						state = 'objects';

						continue;
					else
						% We stop the robot during the calculation
						robot.stop(absPos);
						h = robot.drive(vrep, h);

						% We pop the next table information
						nextTable = tablesPositions(end, :);
						nextRadius = tablesRadius(end);

						tablesPositions(end, :) = [];
						tablesRadius(end) = [];

						% Setup point to determine path
						occMatPos = round(absPos .* map.mapPrec);
						nextPoint = [nextTable(2), nextTable(1)] + nextRadius;

						% We get path to the next table
						pathList = map.getNextPathToExplore(occMatPos, occMatPos, nextPoint);
					end
				end

			%%%%%%%%%%%%%%%%%%%
			% 'objects' state %
			%%%%%%%%%%%%%%%%%%%

			elseif strcmp(state, 'objects')
				% TO DO

			%%%%%%%%%%%%%%%%%%%%%
			% 'objective' state %
			%%%%%%%%%%%%%%%%%%%%%

			elseif strcmp(state, 'objective')
				% TO DO

			%%%%%%%%%%%%%%%%%
			% Unknown state %
			%%%%%%%%%%%%%%%%%
			
			else
				error('Unknown state');
			end

			%%%%%%%%%%%%%%%%%%%%%%%%%%
			% Get the next objective %
			%%%%%%%%%%%%%%%%%%%%%%%%%%

			if hasAccCurrentObj
				% Save the next objective to display it
				mapObjective = pathList(end, :);
				
				% Get the next objective
				[objective(1), objective(2)] = utils.toCartesian(pathList(end, 1), pathList(end, 2), size(map.getOccupancyMatrix(), 1));
				objective = objective ./ map.mapPrec;
				
				% Remove the next objective from the path
				pathList(end, :) = [];
			end

			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			% Move the robot depending of the objective %
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			
			% We check if robot has accomplished its current objective
			hasAccCurrentObj = robot.checkObjective(absPos, objective);
			
			% If robot has not accomplished its objective, we set the velocities
			if ~hasAccCurrentObj
				robot.setVelocitiesToObjective(absPos, orientation, objective, false);
			end
			
			% We drive the robot
			h = robot.drive(vrep, h);

		%%%%%%%%%%%%%%%%
		% 'arm' action %
		%%%%%%%%%%%%%%%%

		elseif strcmp(action, 'arm')

			%%%%%%%%%%%%%%%%%
			% 'grasp' state %
			%%%%%%%%%%%%%%%%%

			if strcmp(state, 'grasp')
				% TO DO

			%%%%%%%%%%%%%%%%
			% 'drop' state %
			%%%%%%%%%%%%%%%%

			elseif strcmp(state, 'drop')
				% TO DO

			%%%%%%%%%%%%%%%%%
			% Unknown state %
			%%%%%%%%%%%%%%%%%

			else
				error('Unknown state');
			end

		%%%%%%%%%%%%%%%%%%
		% Unknown action %
		%%%%%%%%%%%%%%%%%%

		else
			error('Unknown action');
		end


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Show the map and its components %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		
		% Show the map with robot (absolute) position and path (if any)
		if ~isempty(pathList)
			pathDisp = [pathList; mapObjective];
		else
			pathDisp = [];
		end

		map.show(round(absPos .* map.mapPrec), pathDisp);


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
