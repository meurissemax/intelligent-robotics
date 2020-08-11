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

		% Tables information
		tablesCenterPositions
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
		explThresh = 0.985;

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

		function setPoints(obj, points, value)
			% Assign a value 'value' to some points 'points' of the
			% occupancy map ('points' is an array [x y] where x and y
			% are column of values, e.g points = [1 2; 3 4; 5 6].
			%
			% Only assign free position to points that are not walls.
			% This verification is useful because the Hokuyo is not
			% always reliable for the point detection.

			if value == 0
				occVal = getOccupancy(obj.map, points);
				points = points(occVal < 0.9, :);
			end

			setOccupancy(obj.map, points, value);
		end

		function setNeighborhood(obj, point, radius, value)
			% Assign a value 'value' to a point 'point' and his
			% neighborhood (of radius 'radius' in the occupancy map) in
			% the occupancy map.

			% Set the point itself
			obj.setPoints(point, value);

			% Get the limits of the map (to be sure to not exceed it)
			xLimits = obj.map.XWorldLimits;
			yLimits = obj.map.YWorldLimits;

			% Set the neighborhood of the point
			dx = 1 / obj.mapPrec;

			for x = (point(1) - radius * dx):dx:(point(1) + radius * dx)
				for y = (point(2) - radius * dx):dx:(point(2) + radius * dx)
					if x >= xLimits(1) && y >= yLimits(1) && x <= xLimits(2) && y <= yLimits(2) && (x ~= point(1) || y ~= point(2))
						obj.setPoints([x, y], value);
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

			% Get the occupancy matrix
			occMat = obj.getOccupancyMatrix();

			% Inflate the occupancy matrix
			mapInflated = copy(obj.map);
			inflate(mapInflated, obj.inflatedFact);
			occMatInf = occupancyMatrix(mapInflated, 'ternary');

			% Get next point to explore
			if nargin > 3
				nextPoint = varargin{1};
			else
				nextPoint = obj.getNextPointToExplore([size(occMat, 1) - from(2) + 1, from(1)], occMatInf);

				% If we can not find new point, the map is probably explored
				if nextPoint == Inf
					nextPath = Inf;

					return;
				end
			end

			% Get the path to this point
			startPoint = [size(occMat, 1) - pos(2) + 1, pos(1)];
			stopPoint = [nextPoint(1), nextPoint(2)];

			goalPoint = zeros(size(occMat));
			goalPoint(stopPoint(1), stopPoint(2)) = 1;

			% Set stop point to 0 (to be sure that A* can reach it)
			occMatInf(stopPoint(1), stopPoint(2)) = 0;

			% Set neighborhood of start point to 0 (to be sure that A* can begin)
			for x = startPoint(1) - 1:1:startPoint(1) + 1
				for y = startPoint(2) - 1:1:startPoint(2) + 1
					if x >= 1 && y >= 1 && x <= size(occMatInf, 1) && y <= size(occMatInf, 2) && (x ~= startPoint(1) || y ~= startPoint(2))
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
				inflate(mapInflated, reduceInflate);
				occMatInf = occupancyMatrix(mapInflated, 'ternary');

				pathList = imported.astar(startPoint(2), startPoint(1), occMatInf, goalPoint, obj.pathDist);

				reduceInflate = reduceInflate - 0.1;

				% If inflate factor reach 0, not path is possible so return
				% an empty path
				if reduceInflate <= 0
					nextPath = Inf;

					return;
				end
			end

			% Optimize the path
			nextPath = obj.optimizePath(pathList);
		end

		function explored = isExplored(obj)
			% Check if the map can be consired as explored or not.

			% By default, map is not yet fully explored
			explored = false;

			% Get the occupancy matrix
			occMat = obj.getOccupancyMatrix();

			% Number of points representing the map in the occupancy matrix
			totalPts = numel(occMat);

			% Count the number of points discovered (free and obstacle)
			discoveredPts = nnz(occMat == 0) + nnz(occMat == 1);

			% Get the percentage of points discovered
			p = discoveredPts / totalPts;

			% Evaluate if the map is considered as fully explored
			if p >= obj.explThresh
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

			% Update coordinate system of center points (to be the
			% same as the project)
			centers = [centers(:, 2), centers(:, 1)];

			% Set centers positions and radii
			obj.tablesCenterPositions = round(centers ./ resizeFactor);
			obj.tablesRadius = round(radii ./ resizeFactor);
			obj.tablesType(1, 1:numel(radii)) = 0;
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
			[x_free, y_free] = utils.toCartesian(i_free, j_free, size(occMat, 1));
			plot(x_free, y_free, '.g', 'MarkerSize', 10);
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
			[x_occ, y_occ] = utils.toCartesian(i_occ, j_occ, size(occMat, 1));
			plot(x_occ, y_occ, '.r', 'MarkerSize', 10);
			hold on;

			% Position of the robot
			if nargin > 1
				pos = varargin{1};

				plot(pos(1), pos(2), '.k', 'MarkerSize', 20);
				hold on;
			end

			% Path and objective
			if nargin > 2
				pathDisp = varargin{2};

				if length(pathDisp) > 1
					pathConverted = zeros(size(pathDisp, 1), 2);

					for i = 1:size(pathDisp, 1)
						[x, y] = utils.toCartesian(pathDisp(i, 1), pathDisp(i, 2), size(occMat, 1));
						pathConverted(i, :) = [x, y];
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
			if ~isempty(obj.tablesCenterPositions)
				for i = 1:numel(obj.tablesRadius)
					i_center = obj.tablesCenterPositions(i, 1);
					j_center = obj.tablesCenterPositions(i, 2);

					[x, y] = utils.toCartesian(i_center, j_center, size(occMat, 2));

					tableType = obj.tablesType(i);

					if tableType == 1
						circleColor = '#3498db';
					elseif tableType == 2
						circleColor = '#27ae60';
					elseif tableType == 3
						circleColor = '#e74c3c';
					else
						circleColor = 'black';
					end

					viscircles([x, y], obj.tablesRadius(i), 'Color', circleColor, 'LineWidth', 3);
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
	end

	methods (Access = private)
		function nextPoint = getNextPointToExplore(~, pos, occMat)
			% Get the next unexplored point (in the occupanct
			% matrix 'occMat') of the map to visit.

			% We get the occupancy matrix size
			sizeX = size(occMat, 1);
			sizeY = size(occMat, 2);

			% We initialize the distance
			mDist = Inf;

			% We iterate over each point of the occupancy matrix
			for i = 1:sizeX
				for j = 1:sizeY

					% Possible candidate for closest point are inexplored points
					if occMat(i, j) == -1
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
				if a > 0
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
	end
end
