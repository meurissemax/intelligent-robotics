% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

classdef RobotController < handle

	%%%%%%%%%%%%%%%%
	%% Attributes %%
	%%%%%%%%%%%%%%%%

	properties (Access = public)
		% Absolute position of the robot (occupancy
		% map coordinate system)
		absPos

		% Orientation of the robot
		orientation;

		% Data from Hokuyo
		inPts;
		inValue;
	end

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
		rotPrecision = 0.01;

		% Parameters when the robot is stuck
		stuckTresh = 0.6;
		stuckDist = 0.4;
	end


	%%%%%%%%%%%%%
	%% Methods %%
	%%%%%%%%%%%%%

	methods (Access = public)
		function setInitPos(obj, vrep, id, h, mapDimensions, difficulty)
			% Set the initial position of the robot. This initial position
			% is set once and does not change in the future.
			%
			% The initialization depends on the difficulty (i.e. the milestone).

			if strcmp(difficulty, 'easy')
				obj.initPos = obj.getRelativePositionFromGPS(vrep, id, h) - mapDimensions;
			else
				obj.initPos = mapDimensions;
			end

			obj.absPos = obj.initPos;
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

		function updatePositionAndOrientation(obj, vrep, id, h, difficulty)
			% Update the position and orientation of the robot.
			%
			% The update depends on the difficulty (i.e. the milestone).

			% Position
			if strcmp(difficulty, 'easy')
				obj.absPos = obj.getRelativePositionFromGPS(vrep, id, h) - obj.initPos;
			else
				obj.absPos = obj.initPos;
			end

			% Orientation
			if strcmp(difficulty, 'hard')
				obj.orientation = pi;
			else
				obj.orientation = obj.getOrientationFromSensor(vrep, id, h);
			end
		end

		function updateDataFromHokuyo(obj, vrep, h)
			% Retrieve data from Hokuyo sensor. 'inValue' corresponds to
			% free points detected by the Hokuyo and 'inPts' corresponds
			% to unreachable points detected by Hokuyo (wall, obstacle,
			% etc).
			%
			% The points are transformed in absolute reference in order
			% to include them in the map.

			% Transformation to robot absolute position
			trf = transl([obj.absPos, 0]) * trotx(obj.orientation(1)) * troty(obj.orientation(2)) * trotz(obj.orientation(3));

			% Point obtention (from Hokuyo)
			[pts, cts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
			simplifiedPoly = imported.simplifyPolygon([h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1); h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)]);
			in = inpolygon(obj.X, obj.Y, simplifiedPoly(1, :), simplifiedPoly(2, :));

			% Transforming inside points to absolute reference
			obj.inValue = homtrans(trf, [obj.X(in); obj.Y(in); zeros(1, size(obj.X(in), 2))]);
			obj.inValue = transpose([obj.inValue(1, :); obj.inValue(2, :)]);

			% Transforming 'pts' to absolute reference
			obj.inPts = homtrans(trf, [pts(1, cts); pts(2, cts); zeros(1, size(pts(1, cts), 2))]);
			obj.inPts = transpose([obj.inPts(1, :); obj.inPts(2, :)]);
		end

		function [stuck, objective] = checkIfStuck(obj)
			% Check if the robot is stucked (in front of a wall). If the
			% robot is stucked, the return value 'stuck' will be set to
			% 'true' and the 'objective' will be the position where the
			% robot has to go to unstuck it.

			% By default, the robot is not stuck
			stuck = false;
			objective = [];

			% Distance to the nearest element in front of the robot
			inFront = round(size(obj.inPts, 1) / 2);
			distFront = pdist2([obj.absPos(1), obj.absPos(2)], [obj.inPts(inFront, 1), obj.inPts(inFront, 2)], 'euclidean');

			% If robot is too close to something
			if distFront < obj.stuckTresh

				% Get the orientation
				distAngl = [
					abs(angdiff(obj.orientation(3), pi / 2)), ...
					abs(angdiff(obj.orientation(3), -pi / 2)), ...
					abs(angdiff(obj.orientation(3), 0)), ...
					abs(angdiff(obj.orientation(3), pi))
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

				% Define the objective to save the robot
				stuck = true;
				objective = [obj.absPos(1) + dx, obj.absPos(2) + dy];
			end
		end

		function hasAccCurrentObj = checkObjective(obj, objective, rotation)
			% Check if the robot has accomplished its current objective
			% 'objective'. It is a simple comparison between its position
			% and 'objective' (objective of the robot).
			%
			% 'rotation' is a boolean flag that indicates if the movement
			% is a rotation or not. In this case, it is a comparison between
			% the objective angle and the current orientation.

			if rotation
				distObj = abs(angdiff(objective, obj.orientation(3)));
				hasAccCurrentObj = distObj < obj.rotPrecision;
			else
				distObj = [abs(obj.absPos(1) - objective(1)), abs(obj.absPos(2) - objective(2))];
				hasAccCurrentObj = sum(distObj < obj.movPrecision) == numel(distObj);
			end
		end

		function h = move(obj, vrep, h, objective, backward)
			% Set the velocities of the robot according to its position
			% and its orientation so that the robot can reach the
			% objective 'objective'.
			%
			% The 'backward' parameter is a boolean that indicates if
			% the robot has to go forward (false) or backward (true).

			% We get angle between robot position and objective position
			rotAngl = obj.getAngleTo(objective);

			% We get distance to objective, for position and rotation
			distObjPos = [abs(obj.absPos(1) - objective(1)), abs(obj.absPos(2) - objective(2))];
			distObjRot = abs(angdiff(rotAngl, obj.orientation(3)));

			% We define default values
			obj.forwBackVel = 0;
			obj.rotVel = 0;

			if backward
				obj.forwBackVel = obj.forwBackVelFact * sum(distObjPos);
			else
				forward = -obj.forwBackVelFact * sum(distObjPos);
				rotation = obj.rotVelFact * angdiff(rotAngl, obj.orientation(3));

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

			% We drive the robot
			h = obj.drive(vrep, h);
		end

		function [h, rotAngl] = rotate(obj, vrep, h, objective)
			% Set velocities so that the robot only do a rotation to
			% align itself with an objective 'objective'.

			% We get angle between robot position and objective position
			rotAngl = obj.getAngleTo(objective);

			% We set velocities
			obj.forwBackVel = 0;
			obj.rotVel = obj.rotVelFact * angdiff(rotAngl, obj.orientation(3)) / 2;

			% We drive the robot
			h = obj.drive(vrep, h);
		end

		function h = stop(obj, vrep, h)
			% Set the velocities to stop the robot.

			% We set all velocities to 0
			obj.forwBackVel = 0;
			obj.rotVel = 0;

			% We drive the robot
			h = obj.drive(vrep, h);
		end

		function img = takePhoto(~, vrep, id, h)
			% Take a (front) picture with the robot and return
			% the image.

			% Setup the sensor for capturing an image
			res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
			vrchk(vrep, res);

			% Capturing the image
			[res, ~, img] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
			vrchk(vrep, res);
		end

		function tableType = getTableTypeFromImage(~, img)
			% Analyze the input image (an image of a table) to
			% determine its type ('empty', 'easy' or 'hard').
			% This technique is based on the colors of the
			% elements of the table.

			% Get colors channel
			r = img(:, :, 1);
			g = img(:, :, 2);
			b = img(:, :, 3);

			% Find specific colors
			red_pattern = r > 220 & r < 260 & g > 55 & g < 75 & b > 55 & b < 75;
			purple_pattern = r > 210 & r < 240 & g > 20 & g < 40 & b > 214 & b < 240;

			[x_red, ~] = find(red_pattern);
			[x_purple, ~] = find(purple_pattern);

			% Calculate proportions of specific colors
			imsize = numel(r);

			p_red = numel(x_red) / imsize;
			p_purple = numel(x_purple) / imsize;

			% Decide type of the table based on proportions
			if p_red > 0.01
				tableType = 2;
			elseif p_purple > 0.002
				tableType = 3;
			else
				tableType = 1;
			end
		end

		function graspObject(~, vrep, id, h)
			% Grasp an object.

			% Set the inverse kinematics (IK) mode to position and orientation (km_mode = 2)
			res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
			vrchk(vrep, res, true);

			% Set the new position to the expected one for the gripper (predetermined value)
			tpos = [0.3259, -0.0010, 0.2951];

			res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos, vrep.simx_opmode_oneshot);
			vrchk(vrep, res, true);

			% Wait long enough so that the tip is at the right position
			pause(5);

			% Remove the inverse kinematics (IK) mode so that joint angles can be set individually
			res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
			vrchk(vrep, res, true);

			% Set the new gripper angle
			tangle = 0;

			res = vrep.simxSetJointTargetPosition(id, h.armJoints(5), tangle, vrep.simx_opmode_oneshot);
			vrchk(vrep, res, true);

			% Wait long enough so that the tip is at the right position
			pause(5);

			% Open the gripper
			res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
			vrchk(vrep, res);

			% Make MATLAB wait for the gripper to be closed
			pause(3);
		end
	end

	methods (Access = private)
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

		function orientation = getOrientationFromSensor(~, vrep, id, h)
			% Get the orientation of the robot. By default, orientation
			% is a vector [phi theta psi] (Euler's angles). For this
			% project, only 'psi' angle really matters.

			[res, orientation] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
			vrchk(vrep, res, true);
		end

		function rotAngl = getAngleTo(obj, objective)
			% Get angle to aligne robot with 'objective' position.

			a = [0, -1];
			b = [objective(1) - obj.absPos(1), objective(2) - obj.absPos(2)];

			rotAngl = sign(objective(1) - obj.absPos(1)) * acos(dot(a, b) / (norm(a) * norm(b)));
		end

		function h = drive(obj, vrep, h)
			% Use the defined velocities of the robot to move it.
			%
			% Remark : left-right velocity has been set to 0 because
			% we decided to not use it since it is very difficult
			% to apply SLAM techniques with such displacements.

			h = youbot_drive(vrep, h, obj.forwBackVel, 0, obj.rotVel);
		end
	end
end
