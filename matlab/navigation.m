% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function navigation(vrep, id, timestep, map, robot, sceneName)

	%%%%%%%%%%%%%%%%%%%%
	%% Initialization %%
	%%%%%%%%%%%%%%%%%%%%

	% Display information
	fprintf('\n**************\n* Navigation *\n**************\n\n');

	% Set the initial position of the robot
	robot.setInitPos([map.width, map.height]);

	% Set the position of the robot and his neighborhood
	% (radius of 2) to 0 (free position)
	map.setNeighborhood(robot.absPos, 2, 0, true);

	% Initialize copy of the map for correction purposes
	correctMap = copy(map);

	% Initialize elapsed time for data update
	elapsed = timestep;

	% Initialize total elapsed time counter
	totalElapsed = timestep;

	% Initialize the path and the objective of the robot
	pathList = [];
	objective = [];

	% Initialize the flag for the objective
	hasAccCurrentObj = true;

	% Initialize the iteration counter
	itCounter = 0;

	% Number of iteration between each map refresh
	mapRefresh = 50;


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

		map = robot.updatePositionAndOrientation(elapsed, totalElapsed, map, correctMap);


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

				% Display information
				fprintf('Determining new objective and path...\n');

				% We get the point from which we determine new path
				% (front point of the Hokuyo)
				inFront = round(size(robot.inPts, 1) / 2);
				from = robot.inPts(inFront, :);

				% We determine the new path
				pathList = map.getNextPathToExplore(robot.absPos, from);

				% If we can not find a new path, the map has been probably
				% fully explored
				if pathList == Inf

					% Display information
					fprintf('No new path available, map is fully explored ! Navigation will stop here.\n');

					% Export the map
					map.export(sceneName);

					% Stop the simulation
					return;
				end
			end

			% Save the next objective to display it
			mapObjective = pathList(end, :);

			% Get the next objective
			objective = map.matrixToMap(pathList(end, :));

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

		if mod(itCounter, mapRefresh) == 0
			map.show(robot.absPos, [pathList; mapObjective], [robot.inPts; robot.inValue]);
		end


		%%%%%%%%%%%%%%%%%%%%%%%
		%% Iteration counter %%
		%%%%%%%%%%%%%%%%%%%%%%%

		itCounter = itCounter + 1;


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
