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
	navigationDifficulty = 'medium';

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

	% Initialize elapsed time for data update
	elapsed = timestep;

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
	pointsAround = [];
	graspInProgress = false;
	currentPoint = [];
	quarterTable = false;


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

		robot.updatePositionAndOrientation(navigationDifficulty, elapsed);


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

				% Take a photo of the table
				img = robot.takePhoto();

				% Determine table type and update data
				tableType = robot.analyzeTable(img);
				map.tablesType(1, currentTable) = tableType;

				% Update manipulation local variables
				if tableType == 1
					tableObjectivePos = map.tablesCenterPos(currentTable, :);
					tableObjectiveRadius = map.tablesRadius(currentTable);
				elseif tableType == tableDifficulty
					tableObjectsPos = map.tablesCenterPos(currentTable, :);
					tableObjectsRadius = map.tablesRadius(currentTable);
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

			% We check if the 'pointsAround' array is empty.
			% If yes, either there is no objects or we need to
			% generate the points.
			% If no, robot has still job to do !

			if isempty(pointsAround)

				% Check if points was previously generated
				if graspInProgress

					% No object found, manipulation is done
					fprintf('No (more) object on the table, manipulation is finished !\n');

					break;
				else

					% Generate points around table
					pointsAround = map.aroundTable(tableObjectsPos, tableObjectsRadius, 8);

					% Update the flag
					graspInProgress = true;
				end
			else

				% Check if there is a current point
				if isempty(currentPoint)

					% Assign a point
					currentPoint = pointsAround(end, :);

					% Pop the point
					pointsAround(end, :) = [];
				end

				% Check if robot is located on the current point
				if robot.checkObjective(currentPoint)

					% Update state
					state = 'grasp-align';
				else

					% Update state
					previousState = state;
					gotoObjective = currentPoint;
					state = 'goto';
				end
			end

		elseif strcmp(state, 'grasp-align')

			% Stop the robot
			robot.stop();

			% Get angle to be aligned with the table
			tableObjectsAngle = robot.getAngleTo(tableObjectsPos);

			% Check if robot is aligned with the table
			if robot.checkObjective(tableObjectsAngle)

				% Update state
				state = 'grasp-quarter';
			else

				% Update state
				previousState = state;
				rotateObjective = tableObjectsAngle;
				state = 'rotate';
			end

		elseif strcmp(state, 'grasp-quarter')

			% Stop the robot
			robot.stop();

			% Get angle for a quarter turn
			if ~quarterTable
				quarterAngle = robot.orientation(3) - pi / 2;
				quarterTable = true;
			end

			% Check if robot has done the quarter turn
			if robot.checkObjective(quarterAngle)

				% Reset flag
				quarterTable = false;

				% Update state
				state = 'grasp-slide';
			else

				% Update state
				previousState = state;
				rotateObjective = quarterAngle;
				state = 'rotate';
			end

		elseif strcmp(state, 'grasp-slide')

			% Slide robot to the left until it is near the table
			if robot.slide('left', 'in')

				% Update state
				state = 'grasp-grasp';
			end

		elseif strcmp(state, 'grasp-grasp')

			% Stop the robot
			robot.stop();

			% Take a 3D point cloud and analyze it
			pointCloud = robot.take3DPointCloud();
			objectPos = robot.analyzeObjects(pointCloud);

			% Check if there is graspable object
			if ~isempty(objectPos)

				% Grasp the object
				robot.graspObject(objectPos);

				% Reset the data
				pointsAround = [];
				graspInProgress = false;
			end

			% Update the state
			state = 'grasp-back';

			% Reset the current point
			currentPoint = [];

		elseif strcmp(state, 'grasp-back')

			% Slide robot to the right until it has space to move
			if robot.slide('right', 'out')

				% Update state according to previous state
				if isempty(objectPos)
					state = 'grasp';
				else
					state = 'objective';
				end
			end

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
			else

				% Bring the robot to the objective table
				previousState = state;
				gotoObjective = closestTableObjective;
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
