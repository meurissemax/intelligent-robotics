% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

classdef MapManager < handle

	%%%%%%%%%%%%%%%%
	%% Attributes %%
	%%%%%%%%%%%%%%%%

	properties (Access = public)
		% Dimensions of the map
		mapWidth
		mapHeight

		% Precision of the map
		mapPrec

		% Representation of the map (occupancy map)
		map

		% Dimensions of the matrix representation (occupancy matrix)
		matrixWidth
		matrixHeight

		% Tables information (map coordinates)
		tablesCenterPos
		tablesRadius

		% Types :
		%	- 0 : undefined
		%	- 1 : empty
		%	- 2 : easy
		%	- 3 : hard
		tablesType
	end

	properties (Access = private)
		% Inflation factor
		inflatedFact = 0.6;

		% Exploration threshold (percentage of points to visit)
		explThresh = 0.995;

		% Possible connection distance between points for
		% path algorithm
		pathDist = 10;
	end


	%%%%%%%%%%%%%
	%% Methods %%
	%%%%%%%%%%%%%

	methods (Access = public)
		function obj = MapManager(mapWidth, mapHeight, mapPrec)
			% Constructor of the class. It sets the dimensions and the
			% precision of the map and instantiate the occupancy map.

			% Set the dimensions
			obj.mapWidth = mapWidth;
			obj.mapHeight = mapHeight;
			obj.mapPrec = mapPrec;

			% Set the matrix dimensions
			obj.matrixWidth = mapWidth * 2 * mapPrec;
			obj.matrixHeight = mapHeight * 2 * mapPrec;

			% Instantiate the occupancy map (with dimensions x2 because
			% we do not know where the robot starts in the map)
			obj.map = occupancyMap(mapWidth * 2, mapHeight * 2, mapPrec);
		end

		function occMat = getOccupancyMatrix(obj)
			% Get the occupancy matrix corresponding to the occupancy map.

			occMat = occupancyMatrix(obj.map, 'ternary');
		end

		function setPoints(obj, points, value, difficulty)
			% Assign a value 'value' to some points 'points' of the
			% occupancy map ('points' is an array [x y] where x and y
			% are column of values, e.g points = [1 2; 3 4; 5 6].
			%
			% If we have access to GPS each iteration, only assign
			% free position to points that are not walls. This verification
			% is useful because the Hokuyo is not always reliable for
			% the point detection.

			if value == 0
				if strcmp(difficulty, 'easy')
					occVal = getOccupancy(obj.map, points);
					points = points(occVal < 0.9, :);
				end
			end

			setOccupancy(obj.map, points, value);
		end

		function setNeighborhood(obj, point, radius, value, difficulty)
			% Assign a value 'value' to a point 'point' and his
			% neighborhood (of radius 'radius' in the occupancy map) in
			% the occupancy map.

			% Set the point itself
			obj.setPoints(point, value, difficulty);

			% Get the limits of the map (to be sure to not exceed it)
			xLimits = obj.map.XWorldLimits;
			yLimits = obj.map.YWorldLimits;

			% Set the neighborhood of the point
			dx = 1 / obj.mapPrec;

			for x = (point(1) - radius * dx):dx:(point(1) + radius * dx)
				for y = (point(2) - radius * dx):dx:(point(2) + radius * dx)
					if x >= xLimits(1) && y >= yLimits(1) && x <= xLimits(2) && y <= yLimits(2) && (x ~= point(1) || y ~= point(2))
						obj.setPoints([x, y], value, difficulty);
					end
				end
			end
		end

		function nextPath = getNextPathToExplore(obj, pos, from, varargin)
			% Get the path to the next point to explore in the map. The
			% path is calculated from the point [x y] 'from' and requires
			% the position 'pos' of the robot.
			%
			% An pre-determined point can be passed to the function. In
			% this case, the function will simply determine path to this
			% point.

			% Transform points to have matrix coordinates
			pos = round(pos .* obj.mapPrec);
			from = round(from .* obj.mapPrec);

			% Inflate the occupancy matrix
			mapInflated = copy(obj.map);
			inflate(mapInflated, obj.inflatedFact);
			occMatInf = occupancyMatrix(mapInflated, 'ternary');

			% Get next point to explore
			if nargin > 3
				nextPoint = varargin{1};
				nextPoint = round(nextPoint .* obj.mapPrec);
				nextPoint = obj.toMatrix(nextPoint);
			else
				nextPoint = obj.getNextPointToExplore(obj.toMatrix(from), occMatInf);

				% If we can not find new point, the map is probably explored
				if nextPoint == Inf
					nextPath = Inf;

					return;
				end
			end

			% Get the path to this point
			startPoint = obj.toMatrix(pos);
			stopPoint = [nextPoint(1), nextPoint(2)];

			goalPoint = zeros([obj.matrixWidth, obj.matrixHeight]);
			goalPoint(stopPoint(1), stopPoint(2)) = 1;

			% Set stop point to 0 (to be sure that A* can reach it)
			occMatInf(stopPoint(1), stopPoint(2)) = 0;

			% Set neighborhood of start point to 0 (to be sure that A* can begin)
			for x = startPoint(1) - 1:1:startPoint(1) + 1
				for y = startPoint(2) - 1:1:startPoint(2) + 1
					if x >= 1 && y >= 1 && x <= obj.matrixWidth && y <= obj.matrixHeight && (x ~= startPoint(1) || y ~= startPoint(2))
						occMatInf(x, y) = 0;
					end
				end
			end

			% Set neighborhood of stop point to 0 (to be sure that A* can begin)
			for x = stopPoint(1) - 1:1:stopPoint(1) + 1
				for y = stopPoint(2) - 1:1:stopPoint(2) + 1
					if x >= 1 && y >= 1 && x <= obj.matrixWidth && y <= obj.matrixHeight && (x ~= stopPoint(1) || y ~= stopPoint(2))
						occMatInf(x, y) = 0;
					end
				end
			end

			% Calculate the path with A*
			pathList = imported.astar(startPoint(2), startPoint(1), occMatInf, goalPoint, obj.pathDist);

			% If no path can be found, re try with more flexible map (less inflate)
			reduceInflate = obj.inflatedFact - 0.1;

			while pathList(1) == Inf
				mapInflated = copy(obj.map);

				if reduceInflate > 0
					inflate(mapInflated, reduceInflate);
				end

				occMatInf = occupancyMatrix(mapInflated, 'ternary');

				pathList = imported.astar(startPoint(2), startPoint(1), occMatInf, goalPoint, obj.pathDist);

				reduceInflate = reduceInflate - 0.1;

				% If inflate factor reach 0, not path is possible so return
				% an empty path
				if reduceInflate < 0
					nextPath = Inf;

					return;
				end
			end

			% Optimize the path
			nextPath = obj.optimizePath(pathList);
		end

		function [explored, p] = isExplored(obj)
			% Check if the map can be consired as explored or not.

			% Set the exploration threshold factor (to take into
			% account small errors of the sensors)
			explFact = 1.01;

			% By default, map is not yet fully explored
			explored = false;

			% Get the occupancy matrix
			occMat = obj.getOccupancyMatrix();

			% Number of points representing the map in the occupancy matrix
			totalPts = obj.mapWidth * obj.mapHeight * power(obj.mapPrec, 2);

			% Count the number of points discovered (free and obstacle)
			discoveredPts = nnz(occMat == 0) + nnz(occMat == 1);

			% Get the percentage of points discovered
			p = discoveredPts / totalPts;

			% Evaluate if the map is considered as fully explored
			if p >= obj.explThresh * explFact
				explored = true;
			end
		end

		function findTables(obj)
			% Find the center position and radius of each
			% table of the map.

			% Initialize parameter
			guessRadius = 3;
			resizeFactor = 4;

			% Inflate a copy of the map
			mapInflated = copy(obj.map);
			inflate(mapInflated, 0.1);

			% Get corresponding occupancy matrix
			occMatInf = occupancyMatrix(mapInflated);

			% Increase the resolution of the occupancy matrix
			occMatInf = imresize(occMatInf, resizeFactor);

			% Find the centers and radii of the tables
			radiusRange = [guessRadius * resizeFactor, guessRadius * resizeFactor * 3];
			[centers, radii] = imfindcircles(occMatInf, radiusRange);

			% If tables have been found, save data
			if numel(radii) > 0

				% Update coordinate system of center points (to be the
				% same as the project)
				centers = [centers(:, 2), centers(:, 1)];

				% Set centers positions and radii
				centers = round(centers ./ resizeFactor);
				radii = round(radii ./ resizeFactor);

				% Transform to map coordinates system
				obj.tablesCenterPos = obj.matrixToMap(centers);
				obj.tablesRadius = radii ./ obj.mapPrec;

				% Set types
				obj.tablesType(1, 1:numel(radii)) = 0;

			% If no table has been found
			else

				% Set empty arrays
				obj.tablesCenterPos = [];
				obj.tablesRadius = [];
				obj.tablesType = [];
			end
		end

		function points = aroundTable(~, center, radius, number)
			% Generate 'number' points equally spaced around a
			% table centered at 'center' position and with a
			% radius of 'radius'.

			% Increase radius (to be sure to be accessible)
			radius = (5 / 4) * radius;

			% Initialize array of points
			points = zeros(number, 2);

			% Calculte the angle increment
			dx = (2 * pi) / number;

			% Initialize the angle
			ang = 0;

			% Generate each point
			for i = 1:number
				points(i, 1) = center(1) + radius * cos(ang);
				points(i, 2) = center(2) + radius * sin(ang);

				ang = ang + dx;
			end
		end

		function closest = findClosestToTable(obj, from, center, radius, number)
			% Find the closest point in 'points' which are around
			% a table centered at 'center' with a radius of 'radius'.

			% Generate points around the table
			points = obj.aroundTable(center, radius, number);

			% Initialize the distance
			minDist = Inf;

			% Initialize the index
			index = 1;

			% Iterate over each point
			for i = 1:size(points, 1)

				% Calculate distance
				pointDist = pdist2(from, points(i, :), 'euclidean');

				% Update min distance and index
				if pointDist < minDist
					minDist = pointDist;
					index = i;
				end
			end

			% Set the closest
			closest = points(index, :);
		end

		function show(obj, varargin)
			% Display the map. By default, the function displays the
			% occupancy matrix (corresponding to the occupancy map) with
			% some colors : green for accessible points and red for
			% unaccessible points and the differents tables of the
			% map if there are defined.
			%
			% Some other arguments can be passed :
			%	- pos		: the position of the robot (will be displayed
			%				  by a dark point)
			%	- pathList	: the path (will be display by mauve points and
			%				  dark lines)
			%	- hokuyo	: the points detected by the Hokuyo (will be
			%				  displayed by cyan points)

			% Get the occupancy matrix
			occMat = obj.getOccupancyMatrix();

			% Visited free points
			[i_free, j_free] = find(occMat == 0);
			xy_free = obj.toCartesian([i_free, j_free]);
			plot(xy_free(:, 1), xy_free(:, 2), '.g', 'MarkerSize', 10);
			hold on;

			% Points detected by Hokuyo
			if nargin > 3
				hokuyo = varargin{3};
				hokuyo = round(hokuyo .* obj.mapPrec);

				plot(hokuyo(:, 1), hokuyo(:, 2), '.c', 'MarkerSize', 10);
				hold on;
			end

			% Visited occupied points
			[i_occ, j_occ] = find(occMat == 1);
			xy_occ = obj.toCartesian([i_occ, j_occ]);
			plot(xy_occ(:, 1), xy_occ(:, 2), '.r', 'MarkerSize', 10);
			hold on;

			% Position of the robot
			if nargin > 1
				pos = varargin{1};
				pos = round(pos .* obj.mapPrec);

				plot(pos(1), pos(2), '.k', 'MarkerSize', 20);
				hold on;
			end

			% Path and objective
			if nargin > 2
				pathDisp = varargin{2};

				if length(pathDisp) > 1
					pathConverted = zeros(size(pathDisp, 1), 2);

					for i = 1:size(pathDisp, 1)
						pathConverted(i, :) = obj.toCartesian(pathDisp(i, :));
					end

					plot(pathConverted(:, 1), pathConverted(:, 2), '.m', 'MarkerSize', 20);
					hold on;

					line(pathConverted(:, 1), pathConverted(:, 2), 'Color', 'm', 'LineWidth', 2);
					hold on;

					line([pos(1), pathConverted(end, 1)], [pos(2), pathConverted(end, 2)], 'Color', 'black', 'LineWidth', 2);
					hold on;
				end
			end

			% Tables positions
			if ~isempty(obj.tablesCenterPos)
				for i = 1:numel(obj.tablesRadius)
					xy = obj.tablesCenterPos(i, :) * obj.mapPrec;

					viscircles(xy, obj.tablesRadius(i) * obj.mapPrec, 'Color', 'black', 'LineWidth', 3);
					hold on;
				end
			end

			% Others
			hold off;

			title('Map representation');

			drawnow;
		end

		function export(obj, sceneName)
			% Export the representation of the map (the 'occupancyMap'
			% object) in a '<sceneName>.mat' file.

			% Get the Matlab variable to export
			exportMap = obj.map;

			% Save the map
			save(strcat('mat/', sceneName), 'exportMap');
		end

		function load(obj, sceneName)
			% Load the representation of a map (the 'occupancyMap'
			% object) from a '<sceneName>.mat' file.

			% Load the map
			load(strcat('mat/', sceneName, '.mat'), 'exportMap');

			% Save the loaded map
			obj.map = exportMap;

			% Update information of the object
			obj.mapPrec = exportMap.Resolution;

			resizeFactor = 2 * exportMap.Resolution;

			obj.mapWidth = exportMap.GridSize(1) / resizeFactor;
			obj.mapHeight = exportMap.GridSize(2) / resizeFactor;

			obj.matrixWidth = obj.mapWidth * 2 * obj.mapPrec;
			obj.matrixHeight = obj.mapHeight * 2 * obj.mapPrec;
		end

		function mapPoint = matrixToMap(obj, matrixPoint)
			% Convert a point with coordinates of a matrix (in
			% matrix convention) to a point with occupancy map
			% coordinates (cartesian convention).

			% To cartesian
			mapPoint = obj.toCartesian(matrixPoint);

			% To occupancy map coordinates
			mapPoint = round(mapPoint ./ obj.mapPrec, 1);
		end
	end

	methods (Access = private)
		function nextPoint = getNextPointToExplore(obj, pos, occMat)
			% Get the next unexplored point (in the occupanct
			% matrix 'occMat') of the map to visit.

			% We get the occupancy matrix size
			sizeX = obj.matrixWidth;
			sizeY = obj.matrixHeight;

			% We initialize the distance
			mDist = Inf;

			% We iterate over each point of the occupancy matrix
			for i = 1:sizeX
				for j = 1:sizeY

					% Possible candidate for closest point are inexplored points
					if occMat(i, j) < 0
						hasFreeNeighbor = false;

						% We check if there is (at least) an explored and free point
						% in the neighborhood of the possible candidate
						for x = i - 1:1:i + 1
							for y = j - 1:1:j + 1
								if x >= 1 && y >= 1 && x <= sizeX && y <= sizeY && (x ~= i || y ~= j)
									if occMat(x, y) == 0
										hasFreeNeighbor = true;
									end
								end
							end
						end

						% If the inexplored point is a valid possible candidate
						if hasFreeNeighbor
							d = pdist2([i, j], pos, 'euclidean');

							if d < mDist
								if d > 15
									mDist = d;
									nextPoint = [i, j];
								end
							end
						end
					end
				end
			end

			% If no point has been find
			if mDist == Inf
				nextPoint = Inf;
			end
		end

		function optimizedPath = optimizePath(~, pathList)
			% Optimize the path by removing some near points.

			% We put the first point in the optimized path
			optimizedPath = [pathList(1, 1), pathList(1, 2)];
			count = 2;

			keep = false;

			% We check the size of the path
			if size(pathList, 1) < 3
				return;
			end

			% We iterate over the path
			for i = 3:size(pathList, 1)

				% We get last three points
				x = [pathList(i, 1), pathList(i - 1, 1), pathList(i - 2, 1)];
				y = [pathList(i, 2), pathList(i - 1, 2), pathList(i - 2, 2)];

				% We calculate the area
				a = polyarea(x, y);

				% We check if we have to keep the point
				if a > 0.4
					keep = true;
				end

				% We keep the point (if necessary)
				if keep
					optimizedPath(count, :) = [pathList(i - 1, 1), pathList(i - 1, 2)];
					count = count + 1;

					keep = false;
				end
			end
		end

		function xy = toCartesian(obj, ij)
			% Returns the coordinates in a cartesian convention.
			%
			% 'ij' is an array [i1, j1; i2, j2; ...] where
			%	- 'i' the first coordinate in matrix convention
			%	- 'j' the second coordinate in matrix convention

			xy(:, 1) = ij(:, 2);
			xy(:, 2) = obj.matrixWidth - (ij(:, 1) - 1);
		end

		function ij = toMatrix(obj, xy)
			% Returns the coordinates in a matrix convention.
			%
			% 'xy' is an array [x1, y1; x2, y2; ...] where
			%	- 'x' the first coordinate in cartesian convention
			%	- 'y' the second coordinate in cartesian convention

			ij(1) = obj.matrixWidth - (xy(:, 2) - 1);
			ij(2) = xy(:, 1);
		end
	end
end
