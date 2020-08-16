% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

classdef RobotController < handle

	%%%%%%%%%%%%%%%%
	%% Attributes %%
	%%%%%%%%%%%%%%%%

	properties (Access = public)
		% Absolute position of the robot (occupancy
		% map coordinate system, cartesian)
		absPos

		% Orientation of the robot
		orientation

		% Data from Hokuyo
		inPts
		inValue
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
		leftRightVel = 0;
		rotVel = 0;

		% Velocity multipliers
		forwBackVelFact = 5;
		rotVelFact = 1.5;

		% Movements tolerances
		movPrecision = 0.25;
		rotPrecision = 0.01;

		% Parameter when the robot is near an obstacle
		nearTresh = 0.45;

		% Iteration counter
		it = 0;

		%%%%%%%%%%%%
		% Odometry %
		%%%%%%%%%%%%

		% Previous values for the wheel angles
		prevy1 = NaN;
		prevy2 = NaN;
		prevy3 = NaN;
		prevy4 = NaN;

		% Previous orientation value
		prevOr = NaN;

		% Estimated positions and orientation
		posX = NaN;
		posY = NaN;
		theta = NaN;

		%%%%%%%%%%%%%%%%%
		% Scan matching %
		%%%%%%%%%%%%%%%%%

		% Amount of iterations before two scan matching
		ctr = 100;

		% Previous pose
		pX = NaN;
		pY = NaN; % To give a value sometime
		or = NaN;

		prevScan
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
				initOr = obj.getOrientationFromSensor();

				obj.posX = mapDimensions(1);
				obj.posY = mapDimensions(2);
				obj.theta = initOr(3) + pi/2;

				obj.pX = mapDimensions(1);
				obj.pY = mapDimensions(2);
				obj.or = initOr(3) + pi/2;

				[pts, cts] = youbot_hokuyo(obj.vrep, obj.h, obj.vrep.simx_opmode_buffer);
				cart = [double(pts(1, cts)); double(pts(2, cts))].';
				scan = lidarScan(cart);
				obj.prevScan = scan;

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

			%%%%%%%%%%%%
			% Position %
			%%%%%%%%%%%%

			if strcmp(difficulty, 'easy')
				obj.absPos = obj.getRelativePositionFromGPS() - obj.initPos;
			else

				%%%%%%%%%%%%
				% Odometry %
				%%%%%%%%%%%%

				% Get angular positions of the wheels
				[~, y1] = obj.vrep.simxGetJointPosition(obj.id, obj.h.wheelJoints(1), obj.vrep.simx_opmode_buffer);
				[~, y2] = obj.vrep.simxGetJointPosition(obj.id, obj.h.wheelJoints(2), obj.vrep.simx_opmode_buffer);
				[~, y3] = obj.vrep.simxGetJointPosition(obj.id, obj.h.wheelJoints(3), obj.vrep.simx_opmode_buffer);
				[~, y4] = obj.vrep.simxGetJointPosition(obj.id, obj.h.wheelJoints(4), obj.vrep.simx_opmode_buffer);

				% If first entry in the loop
				if isnan(obj.prevy1)
					obj.prevy1 = y1;
					obj.prevy2 = y2;
					obj.prevy3 = y3;
					obj.prevy4 = y4;
				end

				% Angular difference in the time considered
				dw = [y1 - obj.prevy1, y2 - obj.prevy2, y3 - obj.prevy3, y4 - obj.prevy4];
				dw = wrapToPi(dw);

				% Angular speeds -> displacement
				fb = - (1 / 4) * (sum(dw)) * 0.05;
				rot = (1 / 4) * (dw(1) + dw(2) - dw(3) - dw(4)) * 0.1274852; % Determined experimentally

				% Linear speeds
				dt = 0.05;
				fb = fb / dt;
				rot = rot / dt;

				if isnan(fb)
					fb = 0;
				end

				% Use equations to determine pose. Theta is the angle of the robot (Euler3)
				if abs(rot) > 10e-4
					obj.posX = obj.posX - fb/rot * sin(obj.theta) + fb/rot * sin(obj.theta + rot * dt);
					obj.posY = obj.posY + fb/rot * cos(obj.theta) - fb/rot * cos(obj.theta + rot * dt);
					obj.theta = obj.theta + rot * dt; % Available as Euler(3) at any time for b), to estimate for c)
				else
					% No change in orientation.
					obj.posX = obj.posX + fb * sin(obj.theta);
					obj.posY = obj.posY + fb * cos(obj.theta);
				end

				obj.prevy1 = y1;
				obj.prevy2 = y2;
				obj.prevy3 = y3;
				obj.prevy4 = y4;

				%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
				% Scan matching, every x iterations %
				%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

				if obj.it == obj.ctr

					% Obtain points from the Hokuyo and create a lidar object
					[pts, cts] = youbot_hokuyo(obj.vrep, obj.h, obj.vrep.simx_opmode_buffer);
					cart = [double(pts(1,cts)); double(pts(2,cts))].';
					scan = lidarScan(cart);

					% Compute pose. First two values are x and y, last one is orientation
					pose = matchScans(scan, obj.prevScan);
					obj.pX = obj.pX + pose(1);
					obj.pY = obj.pY + pose(2);
					obj.or = obj.or + pose(3);

					% Step to combine the two values obtained via odometry and scan matching

					% Maybe create a condition if scan matching is really dysfunctional in a given case
					obj.posX = 0.8*obj.posX + 0.2*obj.pX;
					obj.posY = 0.8*obj.posY + 0.2*obj.pY;
					obj.theta = 0.8*obj.theta + 0.2*obj.or;
					obj.pX = obj.posX;
					obj.pY = obj.posY;
					obj.or = obj.theta;

					obj.prevScan = scan;
				end

				% Update position
				obj.absPos = [obj.posX, obj.posY];
				obj.it = mod(obj.it + 1, 1000);
			end

			%%%%%%%%%%%%%%%
			% Orientation %
			%%%%%%%%%%%%%%%

			if strcmp(difficulty, 'hard')
				obj.orientation = [pi, pi, pi]; % TO DO for SLAM
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

		function rotAngl = getAngleTo(obj, objective)
			% Get angle to aligne robot with 'objective' position.

			a = [0, -1];
			b = [objective(1) - obj.absPos(1), objective(2) - obj.absPos(2)];

			rotAngl = sign(objective(1) - obj.absPos(1)) * acos(dot(a, b) / (norm(a) * norm(b)));
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
			obj.leftRightVel = 0;
			obj.rotVel = 0;

			forward = -obj.forwBackVelFact * sum(distObjPos);
			rotation = obj.rotVelFact * angdiff(rotAngl, obj.orientation(3));

			% We set velocities of the robot according to its objective
			if distObjRot > pi / 6
				obj.rotVel = rotation;
			elseif distObjRot > pi / 8
				obj.rotVel = rotation;
				obj.forwBackVel = forward / 2;
			elseif distObjRot > pi / 18
				obj.rotVel = rotation / 2;
				obj.forwBackVel = forward;
			else
				obj.forwBackVel = forward;
			end

			% We drive the robot
			obj.drive();
		end

		function rotate(obj, objective)
			% Set velocities so that the robot only do a rotation to
			% have the (absolute) angle 'objective'.

			% We set velocities
			obj.forwBackVel = 0;
			obj.leftRightVel = 0;
			obj.rotVel = obj.rotVelFact * angdiff(objective, obj.orientation(3)) / 2;

			% We drive the robot
			obj.drive();
		end

		function stop(obj)
			% Set the velocities to stop the robot.

			% We set all velocities to 0
			obj.forwBackVel = 0;
			obj.leftRightVel = 0;
			obj.rotVel = 0;

			% We drive the robot
			while obj.h.previousForwBackVel ~= 0 || obj.h.previousRotVel ~=0
				obj.drive();
			end
		end

		function near = slide(obj, direction)
			% Slide the robot left or right (depends on 'direction'
			% value) until the robot is near an obstacle. It returns
			% a flag that indicates if robot is near to the obstacle.

			% By default, robot is not near the obstacle
			near = false;

			% Distance to some elements at 'direction'
			if strcmp(direction, 'left')
				inFrontPts = 0.2;
			else
				inFrontPts = 0.8;
			end

			inFront = round(size(obj.inPts, 1) * inFrontPts);
			distFront = pdist2([obj.absPos(1), obj.absPos(2)], [obj.inPts(inFront, 1), obj.inPts(inFront, 2)], 'euclidean');

			% Set velocities of the robot
			obj.forwBackVel = 0;
			obj.leftRightVel = 1;
			obj.rotVel = 0;

			% Check if robot is too close to something
			if distFront < 0.2
				near = true;
			else
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
			if p_red > 0.008
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

		function drive(obj)
			% Use the defined velocities of the robot to move it.

			obj.h = youbot_drive(obj.vrep, obj.h, obj.forwBackVel, obj.leftRightVel, obj.rotVel);
		end
	end
end
