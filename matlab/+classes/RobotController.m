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

		% Precision of the map where the robot
		% will stand
		mapPrec

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
		isNear = false;
		nearCounter = 0;

		% Navigation difficulty
		navigationDifficulty

		%%%%%%%%%%%%
		% Odometry %
		%%%%%%%%%%%%

		% Previous values for the wheel angles
		prevWheelAngles = [];

		% Estimated position
		estimatedPos = [];

		%%%%%%%%%%%%%%%%%
		% Scan matching %
		%%%%%%%%%%%%%%%%%

		% Scan index
		scanIndex = 1;

		% Number of iterations between two scan matching
		itBetScan

		% List of scans
		scans
	end


	%%%%%%%%%%%%%
	%% Methods %%
	%%%%%%%%%%%%%

	methods (Access = public)
		function obj = RobotController(vrep, id, h, mapPrec, itBetScan, navigationDifficulty)
			% Constructor of the class.

			% Simulator parameters
			obj.vrep = vrep;
			obj.id = id;
			obj.h = h;

			% Map information
			obj.mapPrec = mapPrec;

			% Scan matching
			obj.itBetScan = itBetScan;
			obj.scans = cell(itBetScan, 5);

			% Navigation difficulty
			obj.navigationDifficulty = navigationDifficulty;
		end

		function setInitPos(obj, mapDimensions)
			% Set the initial position of the robot. This initial position
			% is set once and does not change in the future.

			%%%%%%%%%%%%%%%%%%%
			% Milestone 1. a) %
			%%%%%%%%%%%%%%%%%%%

			% We have access to GPS for position and sensor for orientation.

			if strcmp(obj.navigationDifficulty, 'easy')
				obj.initPos = obj.getRelativePositionFromGPS() - mapDimensions;
				obj.orientation = obj.getOrientationFromSensor();

			%%%%%%%%%%%%%%%%%%%
			% Milestone 1. b) %
			%%%%%%%%%%%%%%%%%%%

			% We do not have constantly access to GPS (only each X minutes) for
			% position but we have sensor for orientation.

			elseif strcmp(obj.navigationDifficulty, 'medium')
				obj.initPos = obj.getRelativePositionFromGPS() - mapDimensions;
				obj.orientation = obj.getOrientationFromSensor();

				obj.estimatedPos = mapDimensions;
			end

			% Set the position of the robot
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

		function updatePositionAndOrientation(obj, elapsed, itCounter, map, varargin)
			% Update the position and orientation of the robot.

			% Check if we are in manipulation
			if nargin > 4
				manipulation = varargin{1};
			else
				manipulation = false;
			end

			%%%%%%%%%%%%
			% Position %
			%%%%%%%%%%%%

			%%%%%%%%%%%%%%%%%%%
			% Milestone 1. a) %
			%%%%%%%%%%%%%%%%%%%

			if strcmp(obj.navigationDifficulty, 'easy')
				obj.absPos = obj.getRelativePositionFromGPS() - obj.initPos;

			%%%%%%%%%%%%%%%%%%%
			% Milestone 1. b) %
			%%%%%%%%%%%%%%%%%%%

			elseif strcmp(obj.navigationDifficulty, 'medium')

				%%%%%%%%%%%%
				% Odometry %
				%%%%%%%%%%%%

				% Initialize estimated position (if needed, for example
				% if the difficulty of the navigation was different from
				% manipulation)
				if isempty(obj.estimatedPos)
					obj.estimatedPos = obj.absPos;
				end

				% Get angular positions of the wheels
				wheelAngles = zeros(1, 4);

				for i = 1:numel(wheelAngles)
					[~, wheelAngles(i)] = obj.vrep.simxGetJointPosition(obj.id, obj.h.wheelJoints(i), obj.vrep.simx_opmode_buffer);
				end

				% Set psi angle
				psiAngle = obj.orientation(3) + pi / 2;

				% If first entry in the loop
				if isempty(obj.prevWheelAngles)
					obj.prevWheelAngles = wheelAngles;
					elapsed = 0.05;
				end

				% Angular difference in the time considered
				dw = wheelAngles - obj.prevWheelAngles;
				dw = wrapToPi(dw);

				% Angular speeds to get displacement (values determined experimentally)
				fb = -0.25 * (sum(dw)) * 0.05;
				rot = 0.25 * (dw(1) + dw(2) - dw(3) - dw(4)) * 0.1274852;

				% Linear speeds (forward-backward and rotational)
				if elapsed < 0.05
					elapsed = 0.05;
				end

				dt = elapsed;

				fb = fb / dt;
				rot = rot / dt;

				if isnan(fb)
					fb = 0;
				end

				% Use equations to determine pose
				if abs(rot) > 10e-5

					% If change in orientation
					obj.estimatedPos(1) = obj.estimatedPos(1) - (fb / rot) * sin(psiAngle) + (fb / rot) * sin(psiAngle + rot * dt);
					obj.estimatedPos(2) = obj.estimatedPos(2) + (fb / rot) * cos(psiAngle) - (fb / rot) * cos(psiAngle + rot * dt);
				else

					% If no change in orientation
					obj.estimatedPos(1) = obj.estimatedPos(1) + fb * sin(psiAngle);
					obj.estimatedPos(2) = obj.estimatedPos(2) + fb * cos(psiAngle);
				end

				% Save wheel angles
				obj.prevWheelAngles = wheelAngles;

				% Update position
				obj.absPos = obj.estimatedPos;

				%%%%%%%%%%%%%%%%%
				% Scan matching %
				%%%%%%%%%%%%%%%%%

				itCounter = mod(itCounter, obj.itBetScan);

				if itCounter == obj.itBetScan - 1
					obj.stop();
					obj.correctPositionAndScans(map, manipulation);
				end
			end

			%%%%%%%%%%%%%%%
			% Orientation %
			%%%%%%%%%%%%%%%

			obj.orientation = obj.getOrientationFromSensor();
		end

		function updateDataFromHokuyo(obj, varargin)
			% Retrieve data from Hokuyo sensor. 'inValue' corresponds to
			% free points detected by the Hokuyo and 'inPts' corresponds
			% to unreachable points detected by Hokuyo (wall, obstacle,
			% etc).
			%
			% The points are transformed in absolute reference in order
			% to include them in the map.

			% Check if we are in manipulation
			if nargin > 1
				manipulation = varargin{1};
			else
				manipulation = false;
			end

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

			% Save the scans to correct them later
			if ~strcmp(obj.navigationDifficulty, 'easy') && ~manipulation
				obj.scans{obj.scanIndex, 1} = in;
				obj.scans{obj.scanIndex, 2} = pts;
				obj.scans{obj.scanIndex, 3} = cts;
				obj.scans{obj.scanIndex, 4} = obj.absPos;
				obj.scans{obj.scanIndex, 5} = obj.orientation;

				obj.scanIndex = mod(obj.scanIndex + 1, obj.itBetScan);

				if obj.scanIndex == 0
					obj.scanIndex = 1;
				end
			end
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

			% If robot was already detected as near an obstacle
			if obj.isNear

				% If the counter reach the limit, reset
				% it and the near flag
				if obj.nearCounter == 100
					obj.isNear = false;
					obj.nearCounter = 0;

				% Increment the counter and stop the function
				% to let time to the robot to move
				else
					obj.nearCounter = obj.nearCounter + 1;

					return;
				end
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
				obj.isNear = true;
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

				% Small trick to adapt the 'gotoObjective'
				objective = round(objective .* obj.mapPrec) ./ obj.mapPrec;

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
			while obj.h.previousForwBackVel ~= 0 || obj.h.previousLeftRightVel ~= 0 || obj.h.previousRotVel ~=0
				obj.drive();
			end
		end

		function near = forward(obj)
			% Move the robot forward until the robot is near an
			% obstacle. It returns a flag that indicates if robot
			% is near to the obstacle.

			% By default, robot is not near the obstacle
			near = false;

			% Get point of Hokuyo to check
			distPts = round(size(obj.inPts, 1) * 0.5);

			% Distance to nearest element in front
			distNear = pdist2([obj.absPos(1), obj.absPos(2)], [obj.inPts(distPts, 1), obj.inPts(distPts, 2)], 'euclidean');

			% Set velocities of the robot
			obj.forwBackVel = -1;
			obj.leftRightVel = 0;
			obj.rotVel = 0;

			% Check the condition
			if distNear < 0.5
				near = true;
			else
				obj.drive();
			end
		end

		function img = takePhoto(obj, varargin)
			% Take a picture with the sensor and return it.
			%
			% An additional argument can be passed to choose
			% the orientation of the sensor.

			% Setup the sensor for capturing an image
			res = obj.vrep.simxSetIntegerSignal(obj.id, 'handle_rgb_sensor', 1, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res);

			% Rotate the sensor, if needed
			if nargin > 1
				res = obj.vrep.simxSetObjectOrientation(obj.id, obj.h.rgbdCasing, obj.h.ref, [0, 0, (-pi / 2) + varargin{1}], obj.vrep.simx_opmode_oneshot_wait);
				vrchk(obj.vrep, res);

				pause(2);
			end

			% Capturing the image
			[res, ~, img] = obj.vrep.simxGetVisionSensorImage2(obj.id, obj.h.rgbSensor, 0, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res);
		end

		function pointCloud = take3DPointCloud(obj, varargin)
			% Take a 3D point cloud with the sensor and return it.
			%
			% An additional argument can be passed to choose
			% multiples angles.

			% Setup the scan angles
			if nargin > 1
				scanAngles = varargin{1};
			else
				scanAngles = 0;
			end

			% Setup structure to handle 3D point cloud(s)
			pointClouds = cell(1, numel(scanAngles));

			% Reduce the view angle to pi / 8 in order to better see the objects
			res = obj.vrep.simxSetFloatSignal(obj.id, 'rgbd_sensor_scan_angle', pi / 32, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res);

			% Take 3D point cloud for each scan angles
			for i = 1:numel(scanAngles)

				% Rotate the sensor
				res = obj.vrep.simxSetObjectOrientation(obj.id, obj.h.rgbdCasing, obj.h.ref, [0, 0, (-pi / 2) + scanAngles(i)], obj.vrep.simx_opmode_oneshot_wait);
				vrchk(obj.vrep, res);

				% Turn the sensor for point cloud on
				res = obj.vrep.simxSetIntegerSignal(obj.id, 'handle_xyz_sensor', 1, obj.vrep.simx_opmode_oneshot_wait);
				vrchk(obj.vrep, res);

				% Get the 3D point cloud
				pts = youbot_xyz_sensor(obj.vrep, obj.h, obj.vrep.simx_opmode_oneshot_wait);

				% Define the rotation matrix
				rotMat = [cos(scanAngles(i)), 0, sin(scanAngles(i)), 0; 0, 1, 0, 0; -sin(scanAngles(i)), 0, cos(scanAngles(i)), 0; 0, 0, 0, 1];

				% Apply the rotation matrix
				pointClouds{i} = rotMat * pts;
			end

			% Concatenate all point clouds in one
			pointCloud = zeros(size(pointClouds{1}));

			for i = 1:numel(scanAngles)
				pointCloud = cat(2, pointCloud, pointClouds{i});
			end
		end

		function tableType = analyzeTable(~, data)
			% Analyze the input data to determine the type of the table
			% ('empty', 'easy' or 'hard').

			% Get colors channel
			r = data(:, :, 1);
			g = data(:, :, 2);
			b = data(:, :, 3);

			% Find specific colors
			red_pattern = r > 220 & r < 260 & g > 50 & g < 80 & b > 50 & b < 80;
			purple_pattern = r > 210 & r < 256 & g > 10 & g < 50 & b > 210 & b < 256;

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

		function objectPos = analyzeObjects(~, data)
			% Analyze data in order to find object and return the position
			% of the object (if any).

			% TO DO
			plot(data);

			% Set the object position
			objectPos = [];
		end

		function grasp(obj, objectPos)
			% Grasp an object.

			% Set the inverse kinematics (IK) mode to position and orientation (km_mode = 2)
			res = obj.vrep.simxSetIntegerSignal(obj.id, 'km_mode', 2, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res, true);

			% Set the new position to the expected one for the gripper (predetermined value)
			tpos = objectPos;

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

		function drop(obj)
			% Drop the grasped object. The drop movement is always
			% the same. There are intermediate positions to be sure
			% that the arm displacements will not be too quick and
			% so dangerous for the object.

			% Remove inverse kinematic mode (to be sure)
			res = obj.vrep.simxSetIntegerSignal(obj.id, 'km_mode', 0, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res, true);

			% Intermediate position 1
			chooseAngle = [0, pi / 4, -pi / 2, 0, 0];

			for i = 1:numel(chooseAngle)
				res = obj.vrep.simxSetJointTargetPosition(obj.id, obj.h.armJoints(i), chooseAngle(i), obj.vrep.simx_opmode_oneshot);
                vrchk(obj.vrep, res, true);
			end

			pause(3);

			% Intermediate position 2
			chooseAngle = [0, - (pi / 8) * 2, - (pi / 8) * 6, pi / 2, 0];

			for i = 1:numel(chooseAngle)
				res = obj.vrep.simxSetJointTargetPosition(obj.id, obj.h.armJoints(i), chooseAngle(i), obj.vrep.simx_opmode_oneshot);
				vrchk(obj.vrep, res, true);
			end

			pause(3);

			% Intermediate position 3
			chooseAngle = [0, - (pi / 8) * 2, - (pi / 8) * 4, (pi / 2) - (2 * pi) / 8, 0];

			for i = 1:numel(chooseAngle)
				res = obj.vrep.simxSetJointTargetPosition(obj.id, obj.h.armJoints(i), chooseAngle(i), obj.vrep.simx_opmode_oneshot);
                vrchk(obj.vrep, res, true);
			end

			pause(3);

			% Open the gripper
			res = obj.vrep.simxSetIntegerSignal(obj.id, 'gripper_open', 0, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res);

			pause(3);

			% Reset the arm position
			chooseAngle = [0, 0, pi / 2, pi / 2, 0];

			for i = 1:numel(chooseAngle)
				res = obj.vrep.simxSetJointTargetPosition(obj.id, obj.h.armJoints(i), chooseAngle(i), obj.vrep.simx_opmode_oneshot);
				vrchk(obj.vrep, res, true);
			end

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

		function correctPositionAndScans(obj, map, varargin)
			% Corrects the position of the robot with the GPS as
			% well as the map with the saved scans.

			% Check if we are in manipulation
			if nargin > 2
				manipulation = varargin{1};
			else
				manipulation = false;
			end

			% Get the true position
			truePos = obj.getRelativePositionFromGPS() - obj.initPos;

			% Update the estimated position
			obj.estimatedPos = truePos;

			% If we are not in manipulation milestone
			if ~manipulation

				% Reset scan index
				obj.scanIndex = 1;

				% Detect deviation of our position from that one
				devPos = obj.estimatedPos - truePos;

				% Increment of deviation at each step
				incDev = devPos ./ obj.itBetScan;

				% Updating all scans from the 'itBetScan' previous iterations
				i = 1;

				while i < obj.itBetScan && ~isempty(obj.scans{i, 1})

					% Get saved data
					in = obj.scans{i, 1};
					pts = obj.scans{i, 2};
					cts = obj.scans{i, 3};
					sAbsPos = obj.scans{i, 4};
					sOrientation = obj.scans{i, 5};

					% Corrective term
					sAbsPos = sAbsPos - (incDev .* (i - 1));

					% Compute the corrected scans
					trf = transl([sAbsPos, 0]) * trotx(sOrientation(1)) * troty(sOrientation(2)) * trotz(sOrientation(3));

					% Transforming inside points to absolute reference
					cInValue = homtrans(trf, [obj.X(in); obj.Y(in); zeros(1, size(obj.X(in), 2))]);
					cInValue = transpose([cInValue(1, :); cInValue(2, :)]);

					% Transforming 'pts' to absolute reference
					cInPts = homtrans(trf, [pts(1, cts); pts(2, cts); zeros(1, size(pts(1, cts), 2))]);
					cInPts = transpose([cInPts(1, :); cInPts(2, :)]);

					% Putting it in the map to correct it
					map.setPoints(cInValue, 0, 'medium');
					map.setPoints(cInPts, 1, 'medium');

					% Reset scans
					obj.scans{i, 1} = [];
					obj.scans{i, 2} = [];
					obj.scans{i, 3} = [];
					obj.scans{i, 4} = [];
					obj.scans{i, 5} = [];

					% Increment the counter
					i = i + 1;
				end
			end
		end
	end
end
