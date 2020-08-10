% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function manipulation(vrep, id, h, timestep, map, robot, difficulty, varargin)

	%%%%%%%%%%%%%%%%%%%%
	%% Initialization %%
	%%%%%%%%%%%%%%%%%%%%

	% To initialize the map, either the information comes
	% from the navigation phase (map and robot objects),
	% either an additional argument (scenePath) can be passed
	% to the function. In this latter case, the map is loaded
	% from a .mat file and the position of the robot is
	% initialized.

	% Stop the robot (just to be sure)
	robot.setVelocitiesToStop();
	h = robot.drive(vrep, h);

	% Get the table difficulty
	if strcmp(difficulty, 'hard')
		tableDifficulty = 3;
	else
		tableDifficulty = 2;
	end

	% Load the map and set initial position of the robot, if needed
	if nargin > 6
		map.load(varargin{1});
		robot.setInitPos(robot.getRelativePositionFromGPS(vrep, id, h) - [map.mapWidth, map.mapHeight]);
	end

	% Initialize action type and state of the finite state machine
	action = 'move';
	state = 'explore';
	
	% Initialize the path and the objective of the robot
	% (for 'move' actions)
	pathList = [];
	objective = [];
	hasAccCurrentObj = true;

	% Initialize variable for tables analyzing
	hasAccAnalyze = true;

	% Initialize variables for tables navigation
	nearObjects = false;
	nearObjective = false;


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

	% Initialize current table for analysis
	if isempty(map.tablesRadius)
		fprintf('No table found, simulation stops here.\n');

		return;
	else
		currentTable = 0;
	end


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
		% 	1. Go to a table in order to analyze it ('explore') - 'move'
		% 	2. Analyze the current table do determine its type ('analyze') - 'static'
		%
		% Robot has to repeat operations 1 to 2 until all tables have been analyzed.
		%
		% 	3. Go the table with objects ('objects') - 'move'
		% 	4. Grasp an object ('grasp') - 'static'
		% 	5. Go the objective table ('objective') - 'move'
		% 	6. Drop grasped object ('drop') - 'static'
		%
		% Robot has to repeat operations 3 to 6 until milestone is finished.

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

					% If the path is empty (and init movement has
					% been done), the robot is near a table so
					% analyze it.

					if hasAccAnalyze
						hasAccAnalyze = false;
					else
						action = 'static';
						state = 'analyze';

						continue;
					end

					% If there is no more table, all tables have
					% been explore, we can move to next state,
					% else we have to define path to next table

					currentTable = currentTable + 1;

					if currentTable > numel(map.tablesRadius)
						action = 'move';
						state = 'objects';

						continue;
					else
						% We stop the robot during the calculation
						robot.setVelocitiesToStop();
						h = robot.drive(vrep, h);

						% We get the next table information
						nextTable = map.tablesCenterPositions(currentTable, :);
						nextRadius = map.tablesRadius(currentTable);

						% Setup point to determine path
						occMatPos = round(absPos .* map.mapPrec);
						nextPoint = nextTable + nextRadius;

						% We get path to the next table
						pathList = map.getNextPathToExplore(occMatPos, occMatPos, nextPoint);
					end
				end

			%%%%%%%%%%%%%%%%%%%
			% 'objects' state %
			%%%%%%%%%%%%%%%%%%%

			elseif strcmp(state, 'objects')

				% If the path list is empty, either the robot is near
				% the object table, and it is OK or the robot has to
				% go the this object table and so we determine a path

				if isempty(pathList)

					% We check if the robot is already near the object
					% table
					
					if nearObjects
						% Set the flag
						nearObjective = false;

						% Update the state of the robot
						action = 'static';
						state = 'grasp';

						continue;
					else
						% Set the flag
						nearObjects = true;

						% Get table position
						tableID = find(map.tablesType == tableDifficulty);
						tablePos = map.tablesCenterPositions(tableID, :);
						tableRadius = map.tablesRadius(tableID);

						% Setup point to determine path
						occMatPos = round(absPos .* map.mapPrec);
						nextPoint = tablePos + tableRadius;

						% We get path to the table
						pathList = map.getNextPathToExplore(occMatPos, occMatPos, nextPoint);
					end
				end

			%%%%%%%%%%%%%%%%%%%%%
			% 'objective' state %
			%%%%%%%%%%%%%%%%%%%%%

			elseif strcmp(state, 'objective')

				% If the path list is empty, either the robot is near
				% the objective table, and it is OK or the robot has to
				% go the this objective table and so we determine a path

				if isempty(pathList)

					% We check if the robot is already near the objective
					% table
					
					if nearObjective
						% Set the flag
						nearObjects = false;

						% Update the state of the robot
						action = 'static';
						state = 'drop';

						continue;
					else
						% Set the flag
						nearObjective = true;

						% Get table position
						tableID = find(map.tablesType == 1);
						tablePos = map.tablesCenterPositions(tableID, :);
						tableRadius = map.tablesRadius(tableID);

						% Setup point to determine path
						occMatPos = round(absPos .* map.mapPrec);
						nextPoint = tablePos + tableRadius;

						% We get path to the table
						pathList = map.getNextPathToExplore(occMatPos, occMatPos, nextPoint);
					end
				end

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
			hasAccCurrentObj = robot.checkObjective(absPos, orientation, objective, false);
			
			% If robot has not accomplished its objective, we set the velocities
			if ~hasAccCurrentObj
				robot.setVelocitiesToMove(absPos, orientation, objective, false);
			end
			
			% We drive the robot
			h = robot.drive(vrep, h);

		%%%%%%%%%%%%%%%%%%%
		% 'static' action %
		%%%%%%%%%%%%%%%%%%%

		elseif strcmp(action, 'static')

			%%%%%%%%%%%%%%%%%%%
			% 'analyze' state %
			%%%%%%%%%%%%%%%%%%%

			if strcmp(state, 'analyze')
				% The goal is to take a photo of the table and
				% to analyze it. So we have to adjust the robot
				% and take a photo, and then analyze it.

				% Rotate the robot to align it with the table
				[rotObj(1), rotObj(2)] = utils.toCartesian(nextTable(1), nextTable(2), size(map.getOccupancyMatrix(), 1));
				rotObj = rotObj ./ map.mapPrec;
				
				rotAngl = robot.setVelocitiesToRotate(absPos, orientation, rotObj);
				h = robot.drive(vrep, h);

				% If the distance is smaller than a threshold,
				% stop and take a photo
				if robot.checkObjective(absPos, orientation, rotAngl, true)

					% Stop the robot
					robot.setVelocitiesToStop();
					h = robot.drive(vrep, h);

					% Take a photo
					pause(2);
					
					img = robot.takePhoto(vrep, id, h);

					% Determine table type and update the data
					tableType = robot.getTableTypeFromImage(img);
					map.tablesType(1, currentTable) = tableType;

					% Go back to 'explore' state to continue
					% tables exploration
					hasAccAnalyze = true;

					action = 'move';
					state = 'explore';

					continue;
				end

			%%%%%%%%%%%%%%%%%
			% 'grasp' state %
			%%%%%%%%%%%%%%%%%

			elseif strcmp(state, 'grasp')
				disp('GRASP');

				action = 'move';
				state = 'objective';

				continue;

			%%%%%%%%%%%%%%%%
			% 'drop' state %
			%%%%%%%%%%%%%%%%

			elseif strcmp(state, 'drop')
				disp('DROP');

				action = 'move';
				state = 'objects';

				continue;

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
