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
		% Simulator linked to the robot
		vrep
		id
		h

		% Initial position of the robot
		initPos

		% Mesh grid (for Hokuyo's data)
		X
		Y

		% Parameters for controlling the robot's wheels
		forwBackVel = 0;
		rotVel = 0;

		% Velocity multipliers
		forwBackVelFact = 5;
		rotVelFact = 1.5;

		% Movements tolerances
		movPrecision = 0.25;
		rotPrecision = 0.01;

		% Parameter when the robot is near an obstacle
		nearTresh = 0.45;
	end


	%%%%%%%%%%%%%
	%% Methods %%
	%%%%%%%%%%%%%

	methods (Access = public)
		function obj = RobotController(vrep, id, h)
			% Constructor of the class. It sets the parameters of the
			% simulator linked to the robot.

			obj.vrep = vrep;
			obj.id = id;
			obj.h = h;
		end

		function setInitPos(obj, mapDimensions, difficulty)
			% Set the initial position of the robot. This initial position
			% is set once and does not change in the future.
			%
			% The initialization depends on the difficulty (i.e. the milestone).

			if strcmp(difficulty, 'easy')
				obj.initPos = obj.getRelativePositionFromGPS() - mapDimensions;
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

		function updatePositionAndOrientation(obj, difficulty)
			% Update the position and orientation of the robot.
			%
			% The update depends on the difficulty (i.e. the milestone).

			% Position
			if strcmp(difficulty, 'easy')
				obj.absPos = obj.getRelativePositionFromGPS() - obj.initPos;
			else
				obj.absPos = obj.initPos;
			end

			% Orientation
			if strcmp(difficulty, 'hard')
				obj.orientation = pi;
			else
				obj.orientation = obj.getOrientationFromSensor();
			end
		end

		function updateDataFromHokuyo(obj)
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
			[pts, cts] = youbot_hokuyo(obj.vrep, obj.h, obj.vrep.simx_opmode_buffer);
			simplifiedPoly = imported.simplifyPolygon([obj.h.hokuyo1Pos(1), pts(1, :), obj.h.hokuyo2Pos(1); obj.h.hokuyo1Pos(2), pts(2, :), obj.h.hokuyo2Pos(2)]);
			in = inpolygon(obj.X, obj.Y, simplifiedPoly(1, :), simplifiedPoly(2, :));

			% Transforming inside points to absolute reference
			obj.inValue = homtrans(trf, [obj.X(in); obj.Y(in); zeros(1, size(obj.X(in), 2))]);
			obj.inValue = transpose([obj.inValue(1, :); obj.inValue(2, :)]);

			% Transforming 'pts' to absolute reference
			obj.inPts = homtrans(trf, [pts(1, cts); pts(2, cts); zeros(1, size(pts(1, cts), 2))]);
			obj.inPts = transpose([obj.inPts(1, :); obj.inPts(2, :)]);
		end

		function near = isNearObstacle(obj)
			% Check if the robot is to close to an obstacle. If it
			% is the case, the return value 'near' will be set to
			% 'true'.

			% By default, the robot is not near an obstacle
			near = false;

			% If robot don't move (forward or backward), don't check
			if obj.forwBackVel == 0
				return;
			end

			% Distance to some elements in front of the robot
			inFrontPts = [0.25, 0.5, 0.75];
			distFront = zeros(1, numel(inFrontPts));

			sizeInPts = size(obj.inPts, 1);

			for i = 1:numel(inFrontPts)
				inFront = round(sizeInPts * inFrontPts(i));
				distFront(i) = pdist2([obj.absPos(1), obj.absPos(2)], [obj.inPts(inFront, 1), obj.inPts(inFront, 2)], 'euclidean');
			end

			% Check if robot is too close to something
			if sum(distFront < obj.nearTresh) > 0
				near = true;
			end
		end

		function hasAccCurrentObj = checkObjective(obj, objective)
			% Check if the robot has accomplished its current objective
			% 'objective'. It is a simple comparison between its position
			% and 'objective' (objective of the robot) or a comparison
			% between the objective angle and the current orientation
			% (if the movement is a rotation).

			rotation = numel(objective) == 1;

			if rotation
				distObj = abs(angdiff(objective, obj.orientation(3)));
				hasAccCurrentObj = distObj < obj.rotPrecision;
			else
				distObj = [abs(obj.absPos(1) - objective(1)), abs(obj.absPos(2) - objective(2))];
				hasAccCurrentObj = sum(distObj < obj.movPrecision) == numel(distObj);
			end
		end

		function move(obj, objective)
			% Set the velocities of the robot according to its position
			% and its orientation so that the robot can reach the
			% objective 'objective'.

			% We get angle between robot position and objective position
			rotAngl = obj.getAngleTo(objective);

			% We get distance to objective, for position and rotation
			distObjPos = [abs(obj.absPos(1) - objective(1)), abs(obj.absPos(2) - objective(2))];
			distObjRot = abs(angdiff(rotAngl, obj.orientation(3)));

			% We define default values
			obj.forwBackVel = 0;
			obj.rotVel = 0;

			forward = -obj.forwBackVelFact * sum(distObjPos);
			rotation = obj.rotVelFact * angdiff(rotAngl, obj.orientation(3));

			% We set velocities of the robot according to its objective
			if distObjRot > 0.7
				obj.rotVel = rotation;
			elseif distObjRot > 0.35
				obj.forwBackVel = forward / 4;
				obj.rotVel = rotation / 1.5;
			elseif distObjRot > 0.01
				obj.forwBackVel = forward / 2;
				obj.rotVel = rotation / 3;
			else
				obj.forwBackVel = forward;
			end

			% We drive the robot
			obj.drive();
		end

		function rotAngl = rotate(obj, objective)
			% Set velocities so that the robot only do a rotation to
			% align itself with an objective 'objective'.

			% We get angle between robot position and objective position
			rotAngl = obj.getAngleTo(objective);

			% We set velocities
			obj.forwBackVel = 0;
			obj.rotVel = obj.rotVelFact * angdiff(rotAngl, obj.orientation(3)) / 2;

			% We drive the robot
			obj.drive();
		end

		function stop(obj)
			% Set the velocities to stop the robot.

			% We set all velocities to 0
			obj.forwBackVel = 0;
			obj.rotVel = 0;

			% We drive the robot
			while obj.h.previousForwBackVel ~= 0 || obj.h.previousRotVel ~=0
				obj.drive();
			end
		end

		function img = takePhoto(obj)
			% Take a (front) picture with the robot and return
			% the image.

			% Setup the sensor for capturing an image
			res = obj.vrep.simxSetIntegerSignal(obj.id, 'handle_rgb_sensor', 1, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res);

			% Capturing the image
			[res, ~, img] = obj.vrep.simxGetVisionSensorImage2(obj.id, obj.h.rgbSensor, 0, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res);
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

		function graspObject(obj)
			% Grasp an object.

			% Set the inverse kinematics (IK) mode to position and orientation (km_mode = 2)
			res = obj.vrep.simxSetIntegerSignal(obj.id, 'km_mode', 2, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res, true);

			% Set the new position to the expected one for the gripper (predetermined value)
			tpos = [0.3259, -0.0010, 0.2951];

			res = obj.vrep.simxSetObjectPosition(obj.id, obj.h.ptarget, obj.h.armRef, tpos, obj.vrep.simx_opmode_oneshot);
			vrchk(obj.vrep, res, true);

			% Wait long enough so that the tip is at the right position
			pause(5);

			% Remove the inverse kinematics (IK) mode so that joint angles can be set individually
			res = obj.vrep.simxSetIntegerSignal(obj.id, 'km_mode', 0, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res, true);

			% Set the new gripper angle
			tangle = 0;

			res = obj.vrep.simxSetJointTargetPosition(obj.id, obj.h.armJoints(5), tangle, obj.vrep.simx_opmode_oneshot);
			vrchk(obj.vrep, res, true);

			% Wait long enough so that the tip is at the right position
			pause(5);

			% Open the gripper
			res = obj.vrep.simxSetIntegerSignal(obj.id, 'gripper_open', 0, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res);

			% Make MATLAB wait for the gripper to be closed
			pause(3);
		end
	end

	methods (Access = private)
		function relPos = getRelativePositionFromGPS(obj)
			% Get the relative position of the robot. The relative position
			% is the position obtained via the GPS of the robot. So, this
			% position does not correspond to the position of the robot
			% in the occupancy map.

			[res, relPos] = obj.vrep.simxGetObjectPosition(obj.id, obj.h.ref, -1, obj.vrep.simx_opmode_buffer);
			vrchk(obj.vrep, res, true);

			% By default, 'relPos' is a [x y z] vector. We only keep
			% [x y] values (because y is useless for this project)
			relPos = relPos(1:2);
		end

		function orientation = getOrientationFromSensor(obj)
			% Get the orientation of the robot. By default, orientation
			% is a vector [phi theta psi] (Euler's angles). For this
			% project, only 'psi' angle really matters.

			[res, orientation] = obj.vrep.simxGetObjectOrientation(obj.id, obj.h.ref, -1, obj.vrep.simx_opmode_buffer);
			vrchk(obj.vrep, res, true);
		end

		function rotAngl = getAngleTo(obj, objective)
			% Get angle to aligne robot with 'objective' position.

			a = [0, -1];
			b = [objective(1) - obj.absPos(1), objective(2) - obj.absPos(2)];

			rotAngl = sign(objective(1) - obj.absPos(1)) * acos(dot(a, b) / (norm(a) * norm(b)));
		end

		function drive(obj)
			% Use the defined velocities of the robot to move it.
			%
			% Remark : left-right velocity has been set to 0 because
			% we decided to not use it since it is very difficult
			% to apply SLAM techniques with such displacements.

			obj.h = youbot_drive(obj.vrep, obj.h, obj.forwBackVel, 0, obj.rotVel);
		end
	end
end
