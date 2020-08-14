% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function navigation(vrep, id, timestep, map, robot, difficulty, sceneName)

	%%%%%%%%%%%%%%%%%%%%
	%% Initialization %%
	%%%%%%%%%%%%%%%%%%%%

	% Display information
	fprintf('\n**************\n* Navigation *\n**************\n\n');

	% Set the initial position of the robot
	robot.setInitPos([map.mapWidth, map.mapHeight], difficulty);

	% Set the position of the robot and his neighborhood
	% (radius of 2) to 0 (free position)
	map.setNeighborhood(robot.absPos, 2, 0);

	% Initialize the path and the objective of the robot
	pathList = [];
	objective = [];

	% Initialize the flag for the objective
	hasAccCurrentObj = true;

	% Initialize counter for the map rendering (map will
	% refresh each 'mapRefresh' iterations)
	mapCounter = 0;
	mapRefresh = 100;


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

		robot.updatePositionAndOrientation(difficulty);


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Update data from Hokuyo %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		% Update the data of the robot
		robot.updateDataFromHokuyo();

		% Update the map with data from Hokuyo
		map.setPoints(robot.inValue, 0);
		map.setPoints(robot.inPts, 1);


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Check if robot is near an obstacle %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		% If the robot is to close to an obstacle, stop it
		% and reset the path and objective (in order to re
		% define new ones and save the robot).

		if robot.isNearObstacle()
			robot.stop();

			pathList = [];
			objective = [];

			hasAccCurrentObj = true;
		end


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Define new path and objective, if needed %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		% We check if the robot has accomplished his current objective.
		% If yes, set a new objective.
		% If no, the robot has to move to this objective (so, change nothing).

		if hasAccCurrentObj

			% We check if the path is empty.
			% If yes, we have to define a new path to get a new objective.

			if isempty(pathList)

				% We stop the robot during the calculation
				robot.stop();

				% We check if map is possibly explored.
				% If yes, we stop the simulation : exploration is done.
				% If no, we define a new path.

				[explored, p] = map.isExplored();

				fprintf('Map is explored at %.2f%%.\n', p * 100);

				if explored

					% Display information
					fprintf('Map is fully explored ! Its representation will be exported.\n');

					% Export the map
					map.export(sceneName);

					% Stop the simulation
					break;
				else

					% Display information
					fprintf('Determining new objective and path...\n');

					% We get the point from which we determine new path
					% (front point of the Hokuyo)
					inFront = round(size(robot.inPts, 1) / 2);
					from = round(robot.inPts(inFront, :) .* map.mapPrec);

					% We determine the new path
					pathList = map.getNextPathToExplore(round(robot.absPos .* map.mapPrec), from);

					% If we can not find a new path, the map has been probably
					% fully explored
					if pathList == Inf

						% Display information
						fprintf('Unable to determine new objective or path. Navigation will stop here.\n');

						% Export the map
						map.export(sceneName);

						% Stop the simulation
						break;
					end
				end
			end

			% Save the next objective to display it
			mapObjective = pathList(end, :);

			% Get the next objective
			[objective(1), objective(2)] = utils.toCartesian(pathList(end, 1), pathList(end, 2), map.matrixWidth);
			objective = objective ./ map.mapPrec;

			% Remove the next objective from the path
			pathList(end, :) = [];
		end


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Move the robot depending of the objective %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		% We check if robot has accomplished its current objective
		hasAccCurrentObj = robot.checkObjective(objective);

		% If robot has not accomplished its objective, we move it
		if ~hasAccCurrentObj
			robot.move(objective);
		end


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Show the map and its components %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		% Only refresh the map each 'mapRefresh' iteration (because it
		% is very heavy to render the map each iteration).

		if mod(mapCounter, mapRefresh) == 0
			map.show(round(robot.absPos .* map.mapPrec), [pathList; mapObjective], [robot.inPts; robot.inValue]);
		end

		mapCounter = mapCounter + 1;


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
