% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function navigation(vrep, id, h, timestep, map, robot, slam, scenePath)
	
	%%%%%%%%%%%%%%%%%%%%
	%% Initialisation %%
	%%%%%%%%%%%%%%%%%%%%
	
	% Initialize the initial position of the robot and
	% get it
	% Remark : for the SLAM, there is no difference between
	% relative (relPos) and absolute (absPos) position. We
	% completely define the coordinate system of the robot
	% since it only depends of our approximations
	if slam
		robot.setInitPos([map.mapWidth, map.mapHeight]);
		absPos = [map.mapWidth, map.mapHeight];
	else
		robot.setInitPos(robot.getRelativePositionFromGPS(vrep, id, h) - [map.mapWidth, map.mapHeight]);
		absPos = robot.getAbsolutePositionFromGPS(vrep, id, h);
	end
	
	% Set the position of the robot and his neighborhood to 0
	% (free position)
	map.setNeighborhood(absPos, 2, 1 / map.mapPrec, 0);

	% Initialize the path and the objective of the robot
	pathList = [];
	objective = [];
	hasAccCurrentObj = true;


	%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Display information %%
	%%%%%%%%%%%%%%%%%%%%%%%%%

	% Console debug
	fprintf('\n*************************************\n');
	fprintf('* The robot begins state navigation *\n');
	fprintf('*************************************\n\n');
	
	fprintf('In this state, he will discover the map and build an appropriate representation.\n\n');
	fprintf('Robot is exploring the map...\n');
	
	% We reset possible previous figure(s)
	clf;

	% Make sure everything is settled before we start
	pause(2);


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
		if slam
			absPos = robot.getEstimatedPosition();
		else
			absPos = robot.getAbsolutePositionFromGPS(vrep, id, h);
		end

		orientation = robot.getOrientation(vrep, id, h);


		%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Get data from Hokuyo %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%
		
		% Get the data from the Hokuyo
		[inPts, inValue] = robot.getDataFromHokuyo(vrep, h, absPos, orientation);

		% Update the map with data from Hokuyo
		map.setPoints(inValue, 0);
		map.setPoints(inPts, 1);


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Check if robot is stuck %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		[stuck, stuckObjective] = robot.checkIfStuck(inPts, absPos, orientation);
		
		% If robot is stucked, reset the path and move the robot to
		% 'stuckObjective' in order to save it. A new path will be
		% defined at next iteration.
		if stuck
			pathList = [];
			objective = stuckObjective;
	
			hasAccCurrentObj = false;
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

				% We check if map is possibly explored.
				% If yes, we stop the simulation : exploration is done.
				% If no, we define a new path.

				% We stop the robot during the calculation
				robot.setVelocitiesToStop();
				h = robot.drive(vrep, h);

				if map.isExplored()

					% Export the map
					map.export(scenePath);

					% Save the position of the robot
					robot.stopPos = absPos;
					
					% Stop the simulation
					pause(2);
					break;
				else
					% We get the point from which we determine new path
					% (front point of the Hokuyo)
					inFront = round(size(inPts, 1) / 2);
					from = round(inPts(inFront, :) .* map.mapPrec);

					% We determine the new path
					pathList = map.getNextPathToExplore(round(absPos .* map.mapPrec), from);

					% If we can not find a new path, the map has been probably
					% fully explored.
					if pathList == Inf

						% Export the map
						map.export(scenePath);

						% Save the position of the robot
						robot.stopPos = absPos;
						
						% Stop the simulation
						pause(2);
						break;
					end
				end
			end

			% Save the next objective to display it
			mapObjective = pathList(end, :);

			% Get the next objective
			[objective(1), objective(2)] = utils.toCartesian(pathList(end, 1), pathList(end, 2), size(map.getOccupancyMatrix(), 1));
			objective = objective ./ map.mapPrec;

			% Remove the next objective from the path
			pathList(end, :) = [];
		end


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Move the robot depending of the objective %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		% We check if robot has accomplished its current objective
		hasAccCurrentObj = robot.checkObjective(absPos, orientation, objective, false);

		% If robot has not accomplished its objective, we set the velocities
		if ~hasAccCurrentObj
			robot.setVelocitiesToMove(absPos, orientation, objective, stuck);
		end

		% We drive the robot
		h = robot.drive(vrep, h);


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Show the map and its components %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		
		% Show the map with robot (absolute) position and path
		map.show(round(absPos .* map.mapPrec), [pathList; mapObjective], [inPts; inValue]);


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
