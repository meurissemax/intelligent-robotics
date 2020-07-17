% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020

% Taken from Matlab official website.
% URL : https://nl.mathworks.com/matlabcentral/fileexchange/56877-astar-algorithm

function optimalPath = astar(startX, startY, map, goalRegister, connectingDistance)
	%% Preallocation of Matrices

	[Height, Width] = size(map); % Height and width of matrix
	GScore = zeros(Height, Width); % Matrix keeping track of G-scores 
	FScore = single(inf(Height, Width)); % Matrix keeping track of F-scores (only open list) 
	Hn = single(zeros(Height, Width)); % Heuristic matrix
	OpenMAT = int8(zeros(Height, Width)); % Matrix keeping of open grid cells
	ClosedMAT = int8(zeros(Height, Width)); % Matrix keeping track of closed grid cells
	ClosedMAT(map == 1) = 1; % Adding object-cells to closed matrix
	ParentX = int16(zeros(Height, Width)); % Matrix keeping track of X position of parent
	ParentY = int16(zeros(Height, Width)); % Matrix keeping track of Y position of parent
	
	%% Setting up matrices representing neighboors to be investigated

	NeighboorCheck = ones(2 * connectingDistance + 1);
	Dummy = 2 * connectingDistance + 2;
	Mid = connectingDistance + 1;
	
	for i = 1:connectingDistance - 1
		NeighboorCheck(i, i) = 0;
		NeighboorCheck(Dummy - i, i) = 0;
		NeighboorCheck(i, Dummy - i) = 0;
		NeighboorCheck(Dummy - i, Dummy - i) = 0;
		NeighboorCheck(Mid, i) = 0;
		NeighboorCheck(Mid, Dummy - i) = 0;
		NeighboorCheck(i, Mid) = 0;
		NeighboorCheck(Dummy - i, Mid) = 0;
	end
	
	NeighboorCheck(Mid, Mid) = 0;

	[row, col] = find(NeighboorCheck == 1);
	Neighboors = [row col] - (connectingDistance + 1);
	N_Neighboors = size(col, 1);
	
	%% Creating Heuristic-matrix based on distance to nearest goal node

	[col, row] = find(goalRegister == 1);
	RegisteredGoals = [row col];
	Nodesfound = size(RegisteredGoals, 1);

	for k = 1:size(goalRegister, 1)
		for j = 1:size(goalRegister, 2)
			if map(k, j) == 0
				Mat = RegisteredGoals - (repmat([j k], (Nodesfound), 1));
				Distance = (min(sqrt(sum(abs(Mat).^2, 2))));
				Hn(k, j) = Distance;
			end
		end
	end

	%% Initializign start node with FValue and opening first node.
	FScore(startY, startX) = Hn(startY, startX);         
	OpenMAT(startY, startX) = 1;   
	
	while 1 == 1
		MINopenFSCORE = min(min(FScore));
		
		if MINopenFSCORE == inf
			optimalPath = [inf];
			RECONSTRUCTPATH = 0;
			
			break
		end
		
		[CurrentY, CurrentX] = find(FScore == MINopenFSCORE);
		CurrentY = CurrentY(1);
		CurrentX = CurrentX(1);

		if goalRegister(CurrentY, CurrentX) == 1
			RECONSTRUCTPATH = 1;

			break
		end
		
		% Remobing node from OpenList to ClosedList  
		OpenMAT(CurrentY,CurrentX)=0;
		FScore(CurrentY,CurrentX)=inf;
		ClosedMAT(CurrentY,CurrentX)=1;

		for p = 1:N_Neighboors
			i = Neighboors(p, 1);
			j = Neighboors(p, 2);

			if CurrentY + i <1 || CurrentY + i > Height || CurrentX + j < 1 || CurrentX + j > Width
				continue
			end

			Flag = 1;

			if ClosedMAT(CurrentY + i, CurrentX + j) == 0
				if abs(i) > 1 || abs(j) > 1   
					JumpCells = 2 * max(abs(i), abs(j)) - 1;

					for K = 1:JumpCells
						YPOS = round(K * i / JumpCells);
						XPOS = round(K * j / JumpCells);
				
						if map(CurrentY + YPOS, CurrentX + XPOS) == 1
							Flag = 0;
						end
					end
				end

				if Flag == 1
					tentative_gScore = GScore(CurrentY, CurrentX) + sqrt(i^2 + j^2);

					if OpenMAT(CurrentY + i, CurrentX + j) == 0
						OpenMAT(CurrentY + i, CurrentX + j) = 1;                    
					elseif tentative_gScore >= GScore(CurrentY + i, CurrentX + j)
						continue
					end

					ParentX(CurrentY + i, CurrentX + j) = CurrentX;
					ParentY(CurrentY + i, CurrentX + j) = CurrentY;
					GScore(CurrentY + i, CurrentX + j) = tentative_gScore;
					FScore(CurrentY + i, CurrentX + j) = tentative_gScore + Hn(CurrentY + i, CurrentX + j);
				end
			end
		end
	end

	k = 2;

	if RECONSTRUCTPATH
		optimalPath(1, :) = [CurrentY CurrentX];

		while RECONSTRUCTPATH
			CurrentXDummy = ParentX(CurrentY, CurrentX);
			CurrentY = ParentY(CurrentY, CurrentX);
			CurrentX = CurrentXDummy;
			optimalPath(k, :) = [CurrentY CurrentX];

			k = k + 1;

			if (CurrentX == startX) && (CurrentY == startY)
				break
			end
		end
	end
end
