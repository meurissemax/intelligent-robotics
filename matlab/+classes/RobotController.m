% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

classdef RobotController < handle
	% This class is used to manage the robot, its sensors
	% and its movements.

	%%%%%%%%%%%%%%
	% Attributes %
	%%%%%%%%%%%%%%

	properties (Access = private)
		% Initial position of the robot
		initPos

		% Mesh grid (for Hokuyo's data)
		X
		Y

		% Parameters for controlling the robot's wheels
		forwBackVel = 0;
		rotVel = 0;

		% Velocity multipliers
		forwBackVelFact = 4;
		rotVelFact = 1.5;

		% Movements tolerances
		movPrecision = 0.3;

		% Parameters when the robot is stuck
		stuckTresh = 0.8;
		stuckDist = 0.5;
	end


	%%%%%%%%%%%
	% Methods %
	%%%%%%%%%%%

	methods (Access = public)

		function setInitPos(obj, initPos)
			% Set the initial position of the robot. This initial position
			% is set once and does not change in the future.

			obj.initPos = initPos;
		end

		function setMeshGrid(obj, meshSize)
			% Set the mesh grid for the data retrieving from the Hokuyo.
			
			% Get the mesh grid
			[meshX, meshY] = meshgrid(-5:meshSize:5, -5:meshSize:5);
			
			% Reshape the elements
			meshX = reshape(meshX, 1, []);
			meshY = reshape(meshY, 1, []);
			
			% Save the elements
			obj.X = meshX;
			obj.Y = meshY;
		end
		
		function relPos = getRelativePositionFromGPS(~, vrep, id, h)
			% Get the relative position of the robot. The relative position
			% is the position obtained via the GPS of the robot. So, this
			% position does not correspond to the position of the robot
			% in the occupancy map.

			[res, relPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
			vrchk(vrep, res, true);
			
			% By default, 'relPos' is a [x y z] vector. We only keep
			% [x y] values (because y is useless for this project)
			relPos = relPos(1:2);
		end
		
		function absPos = getAbsolutePositionFromGPS(obj, vrep, id, h)
			% Get the absolute position of the robot. The absolute position
			% corresponds to the position of the robot in the occupancy
			% map.
			%
			% The absolute position is calculated as : relPos - initPos.

			absPos = obj.getRelativePositionFromGPS(vrep, id, h) - obj.initPos;
		end

		function absPos = getEstimatedPosition(obj)
			% TO DO
		end

		function orientation = getOrientation(~, vrep, id, h)
			% Get the orientation of the robot. By default, orientation
			% is a vector [phi theta psi] (Euler's angles). For this
			% project, only 'psi' angle really matters.

			[res, orientation] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
			vrchk(vrep, res, true);
		end

		function [inPts, inValue] = getDataFromHokuyo(obj, vrep, h, position, orientation)
			% Retrieve data from Hokuyo sensor. 'inValue' corresponds to
			% free points detected by the Hokuyo and 'inPts' corresponds
			% to unreachable points detected by Hokuyo (wall, obstacle, 
			% etc).
			%
			% The points are transformed in absolute reference in order
			% to include them in the map.
			
			% Transformation to robot absolute position
			trf = transl([position, 0]) * trotx(orientation(1)) * troty(orientation(2)) * trotz(orientation(3));

			% Point obtention (from Hokuyo)
			[pts, cts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
			simplifiedPoly = imported.simplifyPolygon([h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1); h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)]);
			in = inpolygon(obj.X, obj.Y, simplifiedPoly(1, :), simplifiedPoly(2, :));

			% Transforming inside points to absolute reference
			inValue = homtrans(trf, [obj.X(in); obj.Y(in); zeros(1, size(obj.X(in), 2))]);
			inValue = transpose([inValue(1, :); inValue(2, :)]);

			% Transforming 'pts' to absolute reference
			inPts = homtrans(trf, [pts(1, cts); pts(2, cts); zeros(1, size(pts(1, cts), 2))]);
			inPts = transpose([inPts(1, :); inPts(2, :)]);
		end
		
		function [stuck, objective] = checkIfStuck(obj, inPts, position, orientation)
			% Check if the robot is stucked (in front of a wall). If the
			% robot is stucked, the return value 'stuck' will be set to
			% 'true' and the 'objective' will be the position where the
			% robot has to go to unstuck it.
			
			% By default, the robot is not stuck
			stuck = false;
			objective = [];

			% Distance to the nearest element in front of the robot
			inFront = round(size(inPts, 1) / 2);
			distFront = pdist2([position(1), position(2)], [inPts(inFront, 1), inPts(inFront, 2)], 'euclidean');
	
			% If robot is too close to something
			if distFront < obj.stuckTresh
	
				% Get the orientation
				distAngl = [
					abs(angdiff(orientation(3), pi / 2)),
					abs(angdiff(orientation(3), -pi / 2)),
					abs(angdiff(orientation(3), 0)),
					abs(angdiff(orientation(3), pi))
				];
				
				minDistAngl = min(distAngl);
	
				% Get the direction where the robot should backward
				dx = 0;
				dy = 0;
	
				if minDistAngl == distAngl(1)
					dx = -obj.stuckDist;
				elseif minDistAngl == distAngl(2)
					dx = obj.stuckDist;
				elseif minDistAngl == distAngl(3)
					dy = obj.stuckDist;
				else
					dy = -obj.stuckDist;
				end
	
				% Define the objective to save the robot from a certain
				% death
				stuck = true;
				objective = [position(1) + dx, position(2) + dy];
			end
		end
		
		function hasAccCurrentObj = checkObjective(obj, position, objective)
			% Check if the robot has accomplished its current objective
			% 'objective'. It is a simple comparison between the two
			% positions 'position' (position of the robot) and 'objective'
			% (objective of the robot).
			%
			% The rotation is not taken into account because it is not
			% really important for the pure navigation (it will be for
			% the manipulation).

			% By default, robot has not accomplished its objective
			hasAccCurrentObj = false;
			
			% Calculate distance to objective
			distObj = [abs(position(1) - objective(1)), abs(position(2) - objective(2))];
			
			% We check if robot has accomplished its objective
			if sum(distObj < obj.movPrecision) == numel(distObj)
				obj.forwBackVel = 0;
				obj.rotVel = 0;

				hasAccCurrentObj = true;
			end
		end
		
		function setVelocitiesToObjective(obj, position, orientation, objective, backward)
			% Set the velocities of the robot according to its position
			% 'position' and its orientation 'orientation' so that the
			% robot can reach the objective 'objective'.
			% The 'backward' parameter is a boolean that indicates if
			% the robot has to go forward (false) or backward (true).

			% We get angle between robot position and objective position
			a = [0, -1];
			b = [objective(1) - position(1), objective(2) - position(2)];
			
			rotAngl = sign(objective(1) - position(1)) * acos(dot(a, b) / (norm(a) * norm(b)));

			% We get distance to objective, for position and rotation
			distObjPos = [abs(position(1) - objective(1)), abs(position(2) - objective(2))];
			distObjRot = abs(angdiff(rotAngl, orientation(3)));
			
			% We define default values
			obj.forwBackVel = 0;
			obj.rotVel = 0;
			
			if backward
				obj.forwBackVel = obj.forwBackVelFact * sum(distObjPos);
			else
				forward = -obj.forwBackVelFact * sum(distObjPos);
				rotation = obj.rotVelFact * angdiff(rotAngl, orientation(3));

				% We set velocities of the robot according to its objective
				if distObjRot > 0.5
					obj.rotVel = rotation;
				elseif distObjRot > 0.01
					obj.forwBackVel = forward / 2;
					obj.rotVel = rotation;
				else
					obj.forwBackVel = forward;
				end
			end
		end
		
		function h = drive(obj, vrep, h)
			% Use the defined velocities of the robot to move it.
			% Remark : left-right velocity has been set to 0 because
			% we decided to not use it since it is very difficult
			% to apply SLAM techniques with such displacements.

			h = youbot_drive(vrep, h, obj.forwBackVel, 0, obj.rotVel);
		end
	end
end
