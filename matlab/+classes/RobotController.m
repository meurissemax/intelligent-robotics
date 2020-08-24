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

		% Navigation difficulty
		navDifficulty
	end

	properties (Access = private)
		% Simulator linked to the robot
		vrep
		id
		h

		% Precision of the map where the robot will stand
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

		% Velocities multipliers
		forwBackVelFact = 5;
		leftRightVelFact = 3;
		rotVelFact = 1.5;

		% Movements tolerances
		movPrec = 0.25;
		rotPrec = 0.01;

		% Parameter when the robot is near an obstacle
		nearTresh = 0.45;
		isNear = false;
		nearCounter = 0;

		% Distance between center of the robot and sensor
		sensorToRef = [-0.0003, -0.25];

		% Image of the empty gripper
		emptyGripper

		%%%%%%%%%%%%
		% Odometry %
		%%%%%%%%%%%%

		% Previous values for the wheel angles
		prevWheelAngles = [];

		% Previous orientation
		prevOr = [];

		% Estimated position
		estimatedPos = [];

		%%%%%%%%%%%%%%%%%
		% Scan matching %
		%%%%%%%%%%%%%%%%%

		% Scan index
		scanIndex = 1;

		% Number of seconds between two scan matching
		secBetScan

		% Counter to have the correct number of seconds
		secBetScanCounter = 1;

		% Number of iterations between two scan matching
		itBetScan

		% List of scans
		scans

		% Flag for the map correction
		hasCorrected
	end


	%%%%%%%%%%%%%
	%% Methods %%
	%%%%%%%%%%%%%

	methods (Access = public)
		function obj = RobotController(vrep, id, h, mapPrec, secBetScan, timestep, navDifficulty)
			% Constructor of the class.

			% Simulator parameters
			obj.vrep = vrep;
			obj.id = id;
			obj.h = h;

			% Map information
			obj.mapPrec = mapPrec;

			% Scan matching
			obj.secBetScan = secBetScan;
			obj.itBetScan = round(secBetScan / timestep);
			obj.scans = cell(obj.itBetScan, 5);

			% Image of the empty gripper
			load('mat/empty_gripper.mat', 'emptyGripper');
			obj.emptyGripper = emptyGripper;

			% Navigation difficulty
			obj.navDifficulty = navDifficulty;

			% Initialize the mesh grid (for the data retrieving
			% of the Hokuyo)
			obj.setMeshGrid(1 / mapPrec);
		end

		function setInitPos(obj, mapDimensions)
			% Set the initial position of the robot. This initial position
			% is set once and does not change in the future.

			%%%%%%%%%%%%%%%%%%%
			% Milestone 1. a) %
			%%%%%%%%%%%%%%%%%%%

			% We have access to GPS for position and sensor for orientation.

			if strcmp(obj.navDifficulty, 'easy')
				obj.initPos = obj.getRelativePositionFromGPS() - mapDimensions;
				obj.orientation = obj.getOrientationFromSensor();

			%%%%%%%%%%%%%%%%%%%
			% Milestone 1. b) %
			%%%%%%%%%%%%%%%%%%%

			% We do not have constantly access to GPS (only each X minutes) for
			% position but we have sensor for orientation.

			elseif strcmp(obj.navDifficulty, 'medium')
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

		function updatedMap = updatePositionAndOrientation(obj, totalElapsed, map, correctMap, varargin)
			% Update the position and orientation of the robot.

			% By default, the returned map does not change
			updatedMap = map;

			% Check if we are in manipulation
			if nargin > 5
				manipulation = varargin{1};
			else
				manipulation = false;
			end

			%%%%%%%%%%%%%%%
			% Orientation %
			%%%%%%%%%%%%%%%

			obj.orientation = obj.getOrientationFromSensor();

			%%%%%%%%%%%%
			% Position %
			%%%%%%%%%%%%

			%%%%%%%%%%%%%%%%%%%
			% Milestone 1. a) %
			%%%%%%%%%%%%%%%%%%%

			if strcmp(obj.navDifficulty, 'easy')
				obj.absPos = obj.getRelativePositionFromGPS() - obj.initPos;

			%%%%%%%%%%%%%%%%%%%
			% Milestone 1. b) %
			%%%%%%%%%%%%%%%%%%%

			elseif strcmp(obj.navDifficulty, 'medium')

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

				% If first entry in the loop
				if isempty(obj.prevWheelAngles) || isempty(obj.prevOr)
					obj.prevWheelAngles = wheelAngles;
					obj.prevOr = obj.orientation;
				end

				% Set psi angle
				psiAngle = obj.prevOr(3) + pi / 2;

				% Angular difference in the time considered
				dw = wheelAngles - obj.prevWheelAngles;
				dw = wrapToPi(dw);

				% Angular speeds to get displacement (values determined experimentally)
				fb = -0.25 * (sum(dw)) * 0.05;
				rot = obj.orientation(3) - obj.prevOr(3);

				if isnan(fb)
					fb = 0;
				end

				% Use equations to determine pose
				if abs(rot) > 10e-5

					% If change in orientation
					obj.estimatedPos(1) = obj.estimatedPos(1) - (fb / rot) * sin(psiAngle) + (fb / rot) * sin(psiAngle + rot);
					obj.estimatedPos(2) = obj.estimatedPos(2) + (fb / rot) * cos(psiAngle) - (fb / rot) * cos(psiAngle + rot);
				else

					% If no change in orientation
					obj.estimatedPos(1) = obj.estimatedPos(1) + fb * sin(psiAngle);
					obj.estimatedPos(2) = obj.estimatedPos(2) + fb * cos(psiAngle);
				end

				% Save wheel angles
				obj.prevWheelAngles = wheelAngles;

				% Save orientation
				obj.prevOr = obj.orientation;

				% Update position
				obj.absPos = obj.estimatedPos;

				%%%%%%%%%%%%%%%%%
				% Scan matching %
				%%%%%%%%%%%%%%%%%

				obj.hasCorrected = false;

				if totalElapsed > obj.secBetScan * obj.secBetScanCounter
					obj.stop();
					updatedMap = obj.correctPositionAndScans(correctMap, manipulation);

					obj.secBetScanCounter = obj.secBetScanCounter + 1;
				end
			end
		end

		function corrected = hasCorrectedMap(obj)
			% Check if the robot has corrected the map (with
			% scan matching) in the current iteration. This
			% verification is useful only for difficulty that
			% are not 'easy'.

			% By default, map is corrected
			corrected = true;

			% If the difficulty is not 'easy'
			if ~strcmp(obj.navDifficulty, 'easy')
				corrected = obj.hasCorrected;
			end
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
			if ~strcmp(obj.navDifficulty, 'easy') && ~manipulation
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

			% Check if robot is too close to something
			if obj.checkFront(obj.nearTresh)
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
				hasAccCurrentObj = distObj < obj.rotPrec;
			else

				% Small trick to adapt the objective of the path generated
				objective = round(objective .* obj.mapPrec) ./ obj.mapPrec;

				distObj = [abs(obj.absPos(1) - objective(1)), abs(obj.absPos(2) - objective(2))];
				hasAccCurrentObj = sum(distObj < obj.movPrec) == numel(distObj);
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

		function near = forward(obj, direction, maxDist)
			% Move the robot forward until it is under a distance
			% to an obstacle. It returns a flag that indicates if
			% robot is near to the obstacle.

			% By default, robot is not near the obstacle
			near = false;

			% Set velocities of the robot
			if strcmp(direction, 'in')
				obj.forwBackVel = -1;
			else
				obj.forwBackVel = 1;
			end

			obj.leftRightVel = 0;
			obj.rotVel = 0;

			% Check if robot is too close to something
			if obj.checkFront(maxDist)
				near = true;
			else
				obj.drive();
			end
		end

		function increment(obj, displacement)
			% Move the robot forward or backward a little bit.

			% Set robot velocities
			obj.forwBackVel = -sign(displacement);
			obj.leftRightVel = 0;
			obj.rotVel = 0;

			% Initialize previous wheel angles
			prev = zeros(1, 4);

			% Initialize displacement
			fb = 0;

			% Move the robot
			while fb < abs(displacement)

				% Move the robot
				obj.drive();

				% Get angular positions of the wheels
				wheelAngles = zeros(1, 4);

				for i = 1:numel(wheelAngles)
					[~, wheelAngles(i)] = obj.vrep.simxGetJointPosition(obj.id, obj.h.wheelJoints(i), obj.vrep.simx_opmode_buffer);
				end

				% If first entry in the loop
				if isempty(prev)
					prev = wheelAngles;
				end

				% Angular difference in the time considered
				dw = wheelAngles - prev;
				dw = wrapToPi(dw);

				% Angular speeds to get displacement (values determined experimentally)
				fb = fb + abs(-0.25 * (sum(dw)) * 0.05);

				% Let time to the robot to move
				pause(0.15);
			end
		end

		function img = takePhoto(obj, varargin)
			% Take a photo with the sensor and return it.
			%
			% An additional argument can be passed to choose
			% the angle.

			% Set the view angle
			res = obj.vrep.simxSetFloatSignal(obj.id, 'rgbd_sensor_scan_angle', pi / 4, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res);

			% Get the angle
			if nargin > 1
				ang = varargin{1};
			else
				ang = 0;
			end

			% Rotate the sensor
			res = obj.vrep.simxSetObjectOrientation(obj.id, obj.h.rgbdCasing, obj.h.ref, [0, 0, (-pi / 2) + ang], obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res);

			% Capture the photo
			res = obj.vrep.simxSetIntegerSignal(obj.id, 'handle_rgb_sensor', 1, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res);

			% Get the captured photo
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

				% Setup the view angle
				if nargin > 2
					viewAngle = varargin{2};
				else
					viewAngle = pi / 32;
				end
			else
				scanAngles = 0;
			end

			% Setup structure to handle 3D point cloud(s)
			pointClouds = cell(1, numel(scanAngles));

			% Reduce the view angle to 'viewAngle'
			res = obj.vrep.simxSetFloatSignal(obj.id, 'rgbd_sensor_scan_angle', viewAngle, obj.vrep.simx_opmode_oneshot_wait);
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

		function nCenter = adjustTable(obj, data, radius)
			% Analyze the data (point cloud of a table) to determine
			% the nearest point to the robot center and infer the
			% center of the table coordinates ('nCenter').

			% Filter points
			f = data(4, :) < 1.5 & data(2, :) < -0.05 & data(2, :) > -0.2;
			data = data(:, f);

			% Remove center point (robot itself)
			f = data(4, :) ~= 0;
			data = data(:, f);

			% Get nearest point
			minDist = Inf;
			minIndex = -1;

			for i = 1:size(data, 2)
				d = pdist2(obj.sensorToRef, [data(1, i), data(3, i)], 'euclidean');

				if d < minDist
					minDist = d;
					minIndex = i;
				end
			end

			% Get absolute coordinates of nearest point
			nCoord = [data(1, minIndex), data(3, minIndex)];
			nCoord(2) = -nCoord(2);
			nCoord = obj.sensorToAbs(nCoord);

			% Get absolute coordinates of the center
			nCenter = nCoord - ((obj.absPos - nCoord) ./ minDist) .* radius;
		end

		function [tableType, objectPos] = analyzeTable(obj, data)
			% Analyze the input data to determine the type of the
			% table and the position of the objects (if any).

			% Filter points
			f = data(4, :) < 1.5 & data(2, :) > -0.04;
			data = data(:, f);

			% Remove center point (robot itself)
			f = data(4, :) ~= 0;
			data = data(:, f);

			% Convert data to point cloud object
			pc = pointCloud([data(1, :)', data(3, :)', data(2, :)']);

			% Find clusters in point cloud
			[labels, numClusters] = pcsegdist(pc, 0.1);

			% Determine table type based on clusters
			if numClusters == 0
				tableType = 'empty';
			elseif numClusters > 1
				tableType = 'easy';
			else
				tableType = 'hard';

				[labels, numClusters] = pcsegdist(pc, 0.02);
			end

			% Initialize object positions
			objectPos = zeros(numClusters, 2);
			objectIndex = 1;

			% Check if there are some clusters
			if numClusters == 0
				objectPos = [];
			else

				% Find the center point of each cluster
				for i = 1:numClusters
					inlierIndices = find(labels == i);
					cyl = select(pc, inlierIndices);

					midPoint = sum(cyl.Location) / size(cyl.Location, 1);

					% Check if middle point has two coordinates
					if numel(midPoint) < 2
						continue;
					end

					% Check if middle point is not [0, 0]
					if sum(midPoint == 0) == numel(midPoint)
						continue;
					end

					midPoint = midPoint(1:2);
					midPoint(2) = -midPoint(2);

					% Transform middle point to absolute position
					objectPos(objectIndex, :) = obj.sensorToAbs(midPoint);

					% Increment the index
					objectIndex = objectIndex + 1;
				end

				% Calculate distance to robot for each point
				distRobot = zeros(size(objectPos, 1), 1);

				for i = 1:size(objectPos, 1)
					distRobot(i) = pdist2(obj.absPos, objectPos(i, :), 'euclidean');
				end

				% Sort the distance and re order information
				[~, distOrder] = sort(distRobot, 'descend');

				objectPos = objectPos(distOrder, :);
			end
		end

		function [rotAlign, sensorDist, currentObj] = adjustPosition(obj, data, current)
			% Analyze the data 'data' in order to determine if
			% the robot is aligned with the nearest object
			% visible in data. Returns the rotation angle to do
			% to be aligned, the distance between the sensor
			% and the closest object to it and the relative position
			% of the closest object to 'current' previous position.

			% Get objects center positions
			[~, objectPos] = obj.analyzeTable(data);

			% Setup and calculte minimal distance.
			% 'sensorDist' is for the forward-backward adjustment.
			% 'objectDist' is for the alignement adjustment.

			sensorDist = Inf;

			objectDist = Inf;
			objectIndex = 1;

			for i = 1:size(objectPos, 1)
				objectPos(i, :) = obj.absToSensor(objectPos(i, :));

				sDist = pdist2([0, 0], objectPos(i, :), 'euclidean');
				oDist = pdist2(current, objectPos(i, :), 'euclidean');

				if sDist < sensorDist
					sensorDist = sDist;
				end

				if oDist < objectDist
					objectDist = oDist;
					objectIndex = i;
				end
			end

			% Setup current object, i.e. nearest object to the previous
			% detected closest object
			currentObj = objectPos(objectIndex, :);

			% Check if object is aligned and set the rotation
			% to do to be aligned
			xCoord = objectPos(objectIndex, 1);

			if xCoord > 0.1
				rotAlign = 5 * (pi / 180);
			elseif xCoord > 0.02
				rotAlign = 3 * (pi / 180);
			elseif xCoord > 0.008
				rotAlign = 1.5 * (pi / 180);
			elseif xCoord < -0.1
				rotAlign = -5 * (pi / 180);
			elseif xCoord < -0.02
				rotAlign = -3 * (pi / 180);
			elseif xCoord < -0.008
				rotAlign = -1.5 * (pi / 180);
			else
				rotAlign = 0;
			end
		end

		function arm(obj, action)
			% Use the arm of the robot. The robot will grasp or drop an element
			% depending on the value of 'action'.

			% Set the action and angles for the gripper
			if strcmp(action, 'grasp')
				gripperAction = 0;

				armAngles = [ ...
					0, -pi / 4, -(3 * pi) / 4, pi / 2, 0; ...
					0, -pi / 4, -pi / 2, (2 * pi) / 8, 0; ...
					0, -(2.6 * pi) / 8, -pi / 2, (2.6 * pi) / 8, 0 ...
				];
			else
				gripperAction = 1;

				armAngles = [0, -(2.3 * pi) / 8, -pi / 2, (2.3 * pi) / 8, 0];
			end

			% Remove inverse kinematic mode (to be sure)
			res = obj.vrep.simxSetIntegerSignal(obj.id, 'km_mode', 0, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res, true);

			% Open the gripper before to be sure (for grasp only)
			if strcmp(action, 'grasp')

				% Check if gripper is already open
				[~, gripperSignal] = obj.vrep.simxGetIntegerSignal(obj.id, 'gripper_open', obj.vrep.simx_opmode_oneshot_wait);

				% Open the gripper, if needed
				if gripperSignal ~= 1
					res = obj.vrep.simxSetIntegerSignal(obj.id, 'gripper_open', 1, obj.vrep.simx_opmode_oneshot_wait);
					vrchk(obj.vrep, res);

					pause(2);
				end
			end

			% Set the arm angles
			for i = 1:size(armAngles, 1)
				currentAngles = armAngles(i, :);

				for j = 1:numel(currentAngles)
					res = obj.vrep.simxSetJointTargetPosition(obj.id, obj.h.armJoints(j), currentAngles(j), obj.vrep.simx_opmode_oneshot);
					vrchk(obj.vrep, res, true);
				end

				pause(4);
			end

			% Use the gripper
			res = obj.vrep.simxSetIntegerSignal(obj.id, 'gripper_open', gripperAction, obj.vrep.simx_opmode_oneshot_wait);
			vrchk(obj.vrep, res);

			pause(2);

			% Reset the arm position
			resetAngles = [0, 0.74, pi / 4, pi / 2, 0];

			for i = 1:numel(resetAngles)
				res = obj.vrep.simxSetJointTargetPosition(obj.id, obj.h.armJoints(i), resetAngles(i), obj.vrep.simx_opmode_oneshot);
				vrchk(obj.vrep, res, true);
			end

			pause(4);
		end

		function success = checkGrasp(obj, img)
			% Analyze an image (of the gripper) to determine
			% if the grasp has been a success (i.e. check
			% if the gripper in the photo is empty or not).

			% Crop images
			rect = [130.51, 304.51, 242.98, 207.98];

			empty = imcrop(obj.emptyGripper, rect);
			img = imcrop(img, rect);

			% Resize images
			resFact = 0.25;

			empty = imresize(empty, resFact);
			img = imresize(img, resFact);

			% Convert to gray
			empty = rgb2gray(empty);
			img = rgb2gray(img);

			% Compute correlation coefficient
			corrCoef = corr2(empty, img);

			% Check if the grasp is a success or not
			if corrCoef > 0.6
				success = false;
			else
				success = true;
			end
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

		function near = checkFront(obj, maxDist)
			% Check if an obstancle is located in front of the
			% robot. If the obstacle is too close (distance under
			% 'maxDist'), 'check' is 'true', else 'check' is 'false'.
			% Distances are calculted with Hokuyo sensor.

			% By default, no obstacle
			near = false;

			% Distance to some elements in front of the robot
			sizeInPts = size(obj.inPts, 1) / 5;
			inFrontPts = round(2 * sizeInPts):1:round(3 * sizeInPts);
			distFront = zeros(1, numel(inFrontPts));

			for i = 1:numel(inFrontPts)
				distFront(i) = pdist2([obj.absPos(1), obj.absPos(2)], [obj.inPts(inFrontPts(i), 1), obj.inPts(inFrontPts(i), 2)], 'euclidean');
			end

			% Check if robot is too close to something
			if sum(distFront < maxDist) > 0
				near = true;
			end
		end

		function updatedMap = correctPositionAndScans(obj, correctMap, varargin)
			% Corrects the position of the robot with the GPS as
			% well as the map with the saved scans.

			% Display information
			fprintf('Correcting information with sensors and scan matching...\n');

			% By default, updated map is a simple copy of the correct map
			updatedMap = copy(correctMap);

			% Check if we are in manipulation
			if nargin > 2
				manipulation = varargin{1};
			else
				manipulation = false;
			end

			% Get the true position
			truePos = obj.getRelativePositionFromGPS() - obj.initPos;

			% If we are not in manipulation milestone
			if ~manipulation

				% Detect deviation of our position from that one
				devPos = obj.estimatedPos - truePos;

				% Increment of deviation at each step
				incDev = devPos ./ (obj.scanIndex - 1);

				% Reset scan index
				obj.scanIndex = 1;

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
					correctMap.setPoints(cInValue, 0);
					correctMap.setPoints(cInPts, 1);

					% Reset scans
					obj.scans{i, 1} = [];
					obj.scans{i, 2} = [];
					obj.scans{i, 3} = [];
					obj.scans{i, 4} = [];
					obj.scans{i, 5} = [];

					% Increment the counter
					i = i + 1;
				end

				% Assign the corrected map
				updatedMap = copy(correctMap);
			end

			% Flag for the correction
			obj.hasCorrected = true;

			% Update the estimated position
			obj.estimatedPos = truePos;

			% Update the true position
			obj.absPos = obj.estimatedPos;
		end

		function absPos = sensorToAbs(obj, relPos)
			% Convert a relative (to the sensor) position to
			% an absolute position.

			% Define rotation matrix
			rotMat = [ ...
				cos(obj.orientation(3)), -sin(obj.orientation(3)); ...
				sin(obj.orientation(3)) cos(obj.orientation(3)) ...
			];

			% Convert relative to absolute
			absPos = rotMat * (relPos + obj.sensorToRef)' + obj.absPos';
			absPos = absPos';
		end

		function relPos = absToSensor(obj, absPos)
			% Convert an absolute position to a relative (to
			% the sensor) position.

			% Define rotation matrix
			rotMat = [ ...
				cos(obj.orientation(3)), -sin(obj.orientation(3)); ...
				sin(obj.orientation(3)) cos(obj.orientation(3)) ...
			];

			% Convert absolute to relative
			relPos = (rotMat \ (absPos' - obj.absPos')) - obj.sensorToRef';
			relPos = relPos';
		end
	end
end
