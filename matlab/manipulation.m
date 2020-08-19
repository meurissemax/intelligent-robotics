% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function manipulation(vrep, id, timestep, map, robot, difficulty, varargin)

	%%%%%%%%%%%%%%%%%%%%
	%% Initialization %%
	%%%%%%%%%%%%%%%%%%%%

	% Display information
	fprintf('\n****************\n* Manipulation *\n****************\n\n');

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
		robot.setInitPos([map.mapWidth, map.mapHeight]);
	end

	% Initialize elapsed time for data update
	elapsed = timestep;

	% Initialize total elapsed time counter
	totalElapsed = timestep;

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

	% Initialize variable for 'grasp' state
	currentGraspPoint = [];

	% Initialize variables for 'analyze' and 'grasp' states
	graspPoints = [];

	% Initialize variables for 'drop' state
	dropPoints = [];
	currentDropPoint = [];

	% Initialize variables for 'grasp' and 'drop' states
	halfTable = false;


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

		robot.updatePositionAndOrientation(elapsed, totalElapsed, map, map, true);


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Update data from Hokuyo %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		robot.updateDataFromHokuyo(true);


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

		%%%%%%%%%%%%%%%%%%%
		% 'explore' state %
		%%%%%%%%%%%%%%%%%%%

		if strcmp(state, 'explore')

			% Check if all tables have been explored
			if currentTable > numel(map.tablesRadius)

				% Update state
				state = 'objects';
			else

				% Get the position of the current table
				currentTablePos = map.findClosestToTable(robot.absPos, map.tablesCenterPos(currentTable, :), map.tablesRadius(currentTable), 10);

				% Check if the robot is already near the current table
				if robot.checkObjective(currentTablePos)

					% Update state
					currentTableAngle = robot.getAngleTo(map.tablesCenterPos(currentTable, :));
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

				% Take a 3D point cloud of the table
				pc = robot.take3DPointCloud((-pi / 4):(pi / 32):(pi / 4));

				% Determine table type and object positions
				[tableType, objectPos] = robot.analyzeTable(pc);
				map.tablesType(1, currentTable) = tableType;

				% Update manipulation local variables
				if tableType == 1
					tableObjectivePos = map.tablesCenterPos(currentTable, :);
					tableObjectiveRadius = map.tablesRadius(currentTable);
				elseif tableType == tableDifficulty
					tableObjectsPos = map.tablesCenterPos(currentTable, :);
					tableObjectsRadius = map.tablesRadius(currentTable);

					graspPoints = objectPos;
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

			% Get closest point to the table
			closestTableObjects = map.findClosestToTable(robot.absPos, tableObjectsPos, tableObjectsRadius, 10);

			% Check if robot is already near the objects table
			if robot.checkObjective(closestTableObjects)

				% Update state
				state = 'grasp';

				% Display information
				fprintf('Try to grasp an object...\n');
			else

				% Bring the robot to the object table
				previousState = state;
				gotoObjective = closestTableObjects;
				state = 'goto';

				% Display information
				fprintf('Go to the objects table.\n');
			end

		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% 'grasp' state and substates %
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		elseif strcmp(state, 'grasp')

			% Stop the robot
			robot.stop();

			% Get angle to be aligned with the table
			tableObjectsAngle = robot.getAngleTo(tableObjectsPos);

			% Check if robot is aligned with the table
			if robot.checkObjective(tableObjectsAngle)

				% Update state
				state = 'grasp-position';
			else

				% Update state
				previousState = state;
				rotateObjective = tableObjectsAngle;
				state = 'rotate';
			end

		elseif strcmp(state, 'grasp-position')

			% Stop the robot
			robot.stop();

			% Check if a current point is defined
			if isempty(currentGraspPoint)

				% Check if objects positions have already been defined
				if isempty(graspPoints)

					% Take a 3D point cloud
					pointCloud = robot.take3DPointCloud((-pi / 4):(pi / 32):(pi / 4));

					% Get object positions
					[~, graspPoints] = robot.analyzeTable(pointCloud);

					% Check if manipulation is done
					if isempty(graspPoints)
						fprintf('No (more) object available, manipulation is done !\n');

						return;
					end
				end

				% Pop a grasp position
				currentGraspPoint = graspPoints(end, :);
				graspPoints(end, :) = [];
			end

			% Get nearest point (around the table) to the grasp point
			nearestGraspPoint = map.findClosestToTable(currentGraspPoint, tableObjectsPos, tableObjectsRadius, 50);

			% Check if robot is located at current grasp point
			if robot.checkObjective(nearestGraspPoint)

				% Update state
				state = 'grasp-align';
			else

				% Update state
				previousState = state;
				gotoObjective = nearestGraspPoint;
				state = 'goto';
			end

		elseif strcmp(state, 'grasp-align')

			% Stop the robot
			robot.stop();

			% Get angle to be aligned with the table
			objectAngle = robot.getAngleTo(currentGraspPoint);

			% Check if robot is aligned with the table
			if robot.checkObjective(objectAngle)

				% Update state
				state = 'grasp-forward';
			else

				% Update state
				previousState = state;
				rotateObjective = objectAngle;
				state = 'rotate';
			end

		elseif strcmp(state, 'grasp-forward')

			% Move the robot forward until it is near the table
			if robot.forward()

				% Update state
				state = 'grasp-half';
			end

		elseif strcmp(state, 'grasp-half')

			% Stop the robot
			robot.stop();

			% Get angle for an half turn
			if ~halfTable
				halfAngle = robot.orientation(3) - pi;
				halfTable = true;
			end

			% Check if robot has done the quarter turn
			if robot.checkObjective(halfAngle)

				% Reset flag
				halfTable = false;

				% Update state
				state = 'grasp-grasp';
			else

				% Update state
				previousState = state;
				rotateObjective = halfAngle;
				state = 'rotate';
			end

		elseif strcmp(state, 'grasp-grasp')

			% Stop the robot
			robot.stop();

			% Grasp the object
			robot.grasp();

			% Update the state
			state = 'objective';

			% Reset the current point
			currentGraspPoint = [];

		%%%%%%%%%%%%%%%%%%%%%
		% 'objective' state %
		%%%%%%%%%%%%%%%%%%%%%

		elseif strcmp(state, 'objective')

			% If no empty table has been found, stop the manipulation
			if ~exist('tableObjectivePos', 'var')
				fprintf('No empty table found, manipulation will stop here.\n');

				return;
			end

			% Get closest point to the table
			closestTableObjective = map.findClosestToTable(robot.absPos, tableObjectivePos, tableObjectiveRadius, 10);

			% Check if robot is already near the objective table
			if robot.checkObjective(closestTableObjective)

				% Update state
				state = 'drop';

				% Display information
				fprintf('Dropping grasped object...\n');
			else

				% Bring the robot to the objective table
				previousState = state;
				gotoObjective = closestTableObjective;
				state = 'goto';

				% Display information
				fprintf('Go to the objective table.\n');
			end

		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% 'drop' state and substates %
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		elseif strcmp(state, 'drop')

			% Stop the robot
			robot.stop();

			% If there is no more current drop point, get it
			if isempty(currentDropPoint)

				% If there is no more drop points, generate them
				if isempty(dropPoints)
					dropPoints = map.aroundTable(tableObjectivePos, tableObjectiveRadius, 10);
				end

				% Pop the drop point
				currentDropPoint = dropPoints(end, :);
				dropPoints(end, :) = [];
			end

			% Check if robot is located at the drop point
			if robot.checkObjective(currentDropPoint)

				% Update state
				state = 'drop-align';
			else

				% Update state
				previousState = 'drop';
				gotoObjective = currentDropPoint;
				state = 'goto';
			end

		elseif strcmp(state, 'drop-align')

			% Stop the robot
			robot.stop();

			% Get angle to be aligned with the table
			tableObjectiveAngle = robot.getAngleTo(tableObjectivePos);

			% Check if robot is aligned with the table
			if robot.checkObjective(tableObjectiveAngle)

				% Update state
				state = 'drop-forward';
			else

				% Update state
				previousState = state;
				rotateObjective = tableObjectiveAngle;
				state = 'rotate';
			end

		elseif strcmp(state, 'drop-forward')

			% Move the robot forward until it is near the table
			if robot.forward()

				% Update state
				state = 'drop-half';
			end

		elseif strcmp(state, 'drop-half')

			% Stop the robot
			robot.stop();

			% Get angle for an half turn
			if ~halfTable
				halfAngle = robot.orientation(3) - pi;
				halfTable = true;
			end

			% Check if robot has done the quarter turn
			if robot.checkObjective(halfAngle)

				% Reset flag
				halfTable = false;

				% Update state
				state = 'drop-drop';
			else

				% Update state
				previousState = state;
				rotateObjective = halfAngle;
				state = 'rotate';
			end

		elseif strcmp(state, 'drop-drop')

			% Stop the robot
			robot.stop();

			% Drop the grasped object
			robot.drop();

			% Reset the flag
			currentDropPoint = [];

			% Update state
			state = 'objects';

		%%%%%%%%%%%%%%%%
		% 'goto' state %
		%%%%%%%%%%%%%%%%

		elseif strcmp(state, 'goto')

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

						% If no path has been found, stop the manipulation
						if pathList(1) == Inf
							fprintf('Unable to find a path to objective, manipulation will stop here.\n');

							return;
						end
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
		totalElapsed = totalElapsed + max(elapsed, timestep);

		timeleft = timestep - elapsed;

		if timeleft > 0
			pause(timeleft);
		end
	end
end
