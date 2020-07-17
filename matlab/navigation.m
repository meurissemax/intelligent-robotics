% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function navigation(vrep, id, h, timestep, map, robot)
	
	%%%%%%%%%%%%%%%%%%%%
	%% Initialisation %%
	%%%%%%%%%%%%%%%%%%%%
	
	% Initialize the initial position of the robot
	relPos = robot.getRelativePosition(vrep, id, h);
	robot.setInitPos(relPos - [map.mapWidth, map.mapHeight]);
	
	% Set the position of the robot and his neighborhood to 0
	% (free position)
	absPos = robot.getAbsolutePosition(vrep, id, h);
	map.setNeighborhood(absPos, 2, 1 / map.mapPrec, 0);

	% Initialize the mesh grid (for the data retrieving
	% of the Hokuyo)
	meshSize = 1 / map.mapPrec;
	robot.setMeshGrid(meshSize);

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
		
		% Get position and orientation from sensors
		absPos = robot.getAbsolutePosition(vrep, id, h);
		orientation = robot.getOrientation(vrep, id, h);


		%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Get data from Hokuyo %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%
		
		% Get the data from the Hokuyo
		[inPts, inValue] = robot.getDataFromHokuyo(vrep, h, absPos, orientation);

		% Update the map with data from Hokuyo
		map.setPoints(inValue, 0);
		map.setPoints(inPts, 1);
		
		% Adjust the position (to correspond to occupancy matrix coordinates)
		absPos = round(absPos .* map.mapPrec);


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

				if map.isExplored()

					% Export the map
					map.export();
					
					% Stop the simulation
					pause(3);
					break;
				else
					% We get the point from which we determine new path
					% (front point of the Hokuyo)
					inFront = round(size(inPts, 1) / 2);
					from = round(inPts(inFront, :) .* map.mapPrec);

					% We determine the new path
					pathList = map.getNextPathToExplore(absPos, from);

					% If we can not find a new path, the map has been probably
					% fully explored.
					if pathList == Inf

						% Export the map
						map.export();
						
						% Stop the simulation
						pause(3);
						break;
					end
				end
			end

			% Get the next objective
			[objective(1), objective(2)] = utils.toCartesian(pathList(end, 1), pathList(end, 2), size(map.getOccupancyMatrix(), 1));
			pathList(end, :) = [];
		end


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Check if the objective is accomplished %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		
		hasAccCurrentObj = robot.checkObjective(absPos, objective);


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% Move the robot to the objective (if not accomplished) %
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

		if ~hasAccCurrentObj
			robot.setVelocitiesToObjective(absPos, orientation, objective, map.mapPrec);
			h = robot.drive(vrep, h);
		end


		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%% Show the map and its components %%
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		
		% Show the map with robot (absolute) position and path
		map.show(absPos, pathList);


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
