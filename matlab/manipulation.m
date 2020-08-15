% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function manipulation(vrep, id, timestep, map, robot, difficulty, varargin)

	%%%%%%%%%%%%%%%%%%%%
	%% Initialization %%
	%%%%%%%%%%%%%%%%%%%%

	% Display information
	fprintf('\n****************\n* Manipulation *\n****************\n\n');

	% Set the navigation difficulty
	navigationDifficulty = 'easy';

	% Get the table difficulty
	if strcmp(difficulty, 'hard')
		tableDifficulty = 3;
	else
		tableDifficulty = 2;
	end

	% To initialize the map, either the information comes
	% from the navigation phase (map and robot objects),
	% either an additional argument (sceneName) can be passed
	% to the function. In this latter case, the map is loaded
	% from a .mat file and the position of the robot is
	% initialized.

	% Load the map and set initial position of the robot, if needed
	if nargin > 6
		map.load(varargin{1});
		robot.setInitPos([map.mapWidth, map.mapHeight], navigationDifficulty);
	end

	% Initialize the state of the finite state machine
	state = 'explore';

	% Initialize variables for 'goto' state
	pathList = [];
	objective = [];
	hasAccCurrentObj = true;

	% Initialize variables for 'explore' and 'analyze' states
	currentTable = 1;

	% Initialize variables for 'analyze' state
	displayTypes = {'empty', 'easy', 'hard'};


	%%%%%%%%%%%%%%%%%%%%%%
	%% Tables detection %%
	%%%%%%%%%%%%%%%%%%%%%%

	% Display information
	fprintf('Analyzing the map to find tables...\n');

	% Find center positions and radius of the tables
	map.findTables();

	% Initialize current table for analysis
	if numel(map.tablesRadius) < 2
		fprintf('Not enough table found, manipulation will stop here.\n');

		return;
	else
		fprintf('%d table(s) found !\n', numel(map.tablesRadius));
	end


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Show the map and its components %%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	% Since the rendering of the map is very heavy, we
	% only show the map once at the beginning. For the
	% manipulation phase, the update of the map is not
	% very important (since it is always the same).

	map.show();


	%%%%%%%%%%%%%%%
	%% Main loop %%
	%%%%%%%%%%%%%%%

	while true
		tic

		% We check if connection is lost
		if vrep.simxGetConnectionId(id) == -1
			error('Lost connection to remote API.');
		end


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Update position and orientation of the robot %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		robot.updatePositionAndOrientation(navigationDifficulty);


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Update data from Hokuyo %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		robot.updateDataFromHokuyo();


		%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Finite state machine %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%

		% For this part of the project, robot has to handle several states.
		% It will switch between state until manipulation is done.
		%
		% Tables analysis
		% ---------------
		%	1. 'explore' : go to each table detected and 'analyze' and then 'objects'
		%	2. 'analyze' : infer the type of the table based on sensor data and then 'explore'
		%
		% Grasping of objects
		% -------------------
		%	3. 'objects' : go to the objects table and 'grasp'
		%	4. 'grasp' : grasp an object (if any, else terminate) and then 'objective'
		%	5. 'objective' : go to the objective table and 'drop'
		%	6. 'drop' : drop the grasped object and then 'objects'
		%
		% Other states
		% ------------
		%	'goto' : move the robot to the objective 'gotoObjective'
		%	'rotate' : rotate the robot to the objective 'rotateObjective'

		%%%%%%%%%%%%%%%%
		% 'goto' state %
		%%%%%%%%%%%%%%%%

		if strcmp(state, 'goto')

			% If the robot is to close to an obstacle, stop it
			% and reset the path and objective (in order to re
			% define new ones and save the robot).

			if robot.isNearObstacle()
				robot.stop();

				pathList = [];
				objective = [];

				hasAccCurrentObj = true;
			end

			% Check if robot is currently doing an objective
			if hasAccCurrentObj

				% If path is empty, robot is arrived at destination
				% or robot has to plan a path
				if isempty(pathList)

					% Check if robot is arrived at destination
					if robot.checkObjective(gotoObjective)

						% Update state
						state = previousState;
					else

						% Stop the robot during calculation
						robot.stop();

						% Plan a path to the objective
						pathList = map.getNextPathToExplore(robot.absPos, robot.absPos, gotoObjective);
					end
				else

					% Get the next objective
					objective = map.matrixToMap(pathList(end, :));

					% Remove the next objective from the path
					pathList(end, :) = [];

					% Set the flag
					hasAccCurrentObj = false;
				end
			else

				% We check if robot has accomplished its current objective
				hasAccCurrentObj = robot.checkObjective(objective);

				% If robot has not accomplished its objective, we move it
				if ~hasAccCurrentObj
					robot.move(objective);
				end
			end

		%%%%%%%%%%%%%%%%%%
		% 'rotate' state %
		%%%%%%%%%%%%%%%%%%

		elseif strcmp(state, 'rotate')

			% We check if robot has the objective angle
			if robot.checkObjective(rotateObjective)

				% Update state
				state = previousState;
			else

				% Rotate the robot
				robot.rotate(rotateObjective);
			end

		%%%%%%%%%%%%%%%%%%%
		% 'explore' state %
		%%%%%%%%%%%%%%%%%%%

		elseif strcmp(state, 'explore')

			% Check if all tables have been explored
			if currentTable > numel(map.tablesRadius)

				% Update state
				state = 'objects';
			else

				% Get the position of the current table
				currentTablePos = map.matrixToMap(map.tablesCenterPositions(currentTable, :) + map.tablesRadius(currentTable));

				% Check if the robot is already near the current table
				if robot.checkObjective(currentTablePos)

					% Update state
					currentTableAngle = robot.getAngleTo(map.matrixToMap(map.tablesCenterPositions(currentTable, :)));
					state = 'analyze';
				else

					% Bring the robot to the current table
					previousState = state;
					gotoObjective = currentTablePos;
					state = 'goto';

					% Display information
					fprintf('Go to a table to analyze it.\n');
				end
			end

		%%%%%%%%%%%%%%%%%%%
		% 'analyze' state %
		%%%%%%%%%%%%%%%%%%%

		elseif strcmp(state, 'analyze')

			% We stop the robot
			robot.stop();

			% Check if robot is already aligned with the table
			if robot.checkObjective(currentTableAngle)

				% Display information
				fprintf('Analyzing the table...\n');

				% Take a photo of the table
				img = robot.takePhoto();

				% Determine table type and update data
				tableType = robot.getTableTypeFromImage(img);
				map.tablesType(1, currentTable) = tableType;

				% Update manipulation local variables
				if tableType == 1
					tableObjectivePos = map.matrixToMap(map.tablesCenterPositions(currentTable, :) + map.tablesRadius(currentTable));
				elseif tableType == tableDifficulty
					tableObjectsPos = map.matrixToMap(map.tablesCenterPositions(currentTable, :) + map.tablesRadius(currentTable));
				end

				% Display information
				fprintf('Table detected as "%s".\n', displayTypes{tableType});

				% Update the current table and state
				currentTable = currentTable + 1;
				state = 'explore';
			else

				% Update state
				previousState = state;
				rotateObjective = currentTableAngle;
				state = 'rotate';
			end

		%%%%%%%%%%%%%%%%%%%
		% 'objects' state %
		%%%%%%%%%%%%%%%%%%%

		elseif strcmp(state, 'objects')

			% If no table with objects has been found, stop the manipulation
			if ~exist('tableObjectsPos', 'var')
				fprintf('No table with objects found, manipulation will stop here.\n');

				return;
			end

			% Check if robot is already near the objects table
			if robot.checkObjective(tableObjectsPos)

				% Update state
				state = 'grasp';
			else

				% Bring the robot to the object table
				previousState = state;
				gotoObjective = tableObjectsPos;
				state = 'goto';

				% Display information
				fprintf('Go to the objects table.\n');
			end

		%%%%%%%%%%%%%%%%%
		% 'grasp' state %
		%%%%%%%%%%%%%%%%%

		elseif strcmp(state, 'grasp')

			% Display information
			fprintf('Grasping an object...\n');

			% Stop the robot
			robot.stop();

			% Grasp an object
			robot.graspObject();

			% Update state of the robot
			state = 'objective';

		%%%%%%%%%%%%%%%%%%%%%
		% 'objective' state %
		%%%%%%%%%%%%%%%%%%%%%

		elseif strcmp(state, 'objective')

			% If no empty table has been found, stop the manipulation
			if ~exist('tableObjectivePos', 'var')
				fprintf('No empty table found, manipulation will stop here.\n');

				return;
			end

			% Check if robot is already near the objective table
			if robot.checkObjective(tableObjectivePos)

				% Update state
				state = 'drop';
			else

				% Bring the robot to the objective table
				previousState = state;
				gotoObjective = tableObjectivePos;
				state = 'goto';

				% Display information
				fprintf('Go to the objective table.\n');
			end

		%%%%%%%%%%%%%%%%
		% 'drop' state %
		%%%%%%%%%%%%%%%%

		elseif strcmp(state, 'drop')

			% Display information
			fprintf('Dropping grasped object...\n');

			% Stop the robot
			robot.stop();

			% Drop the grasped object
			disp('DROP TO DO');

			% Update state of the robot
			state = 'objects';

		%%%%%%%%%%%%%%%%%
		% Unknown state %
		%%%%%%%%%%%%%%%%%

		else
			error('Unknown state.');
		end


		%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Physic verifications %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%

		% Make sure that we do not go faster than the physics
		% simulation (it is useless to go faster).

		elapsed = toc;
		timeleft = timestep - elapsed;

		if timeleft > 0
			pause(min(timeleft, .01));
		end
	end
end
