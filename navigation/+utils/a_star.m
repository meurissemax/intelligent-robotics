% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function OptimalPath = a_start(StartX, StartY, MAP, GoalRegister, Connecting_Distance)
    % Taken from Matlab official website.
    
    %%  Preallocation of Matrices

    [Height, Width] = size(MAP); % Height and width of matrix
    GScore = zeros(Height, Width); % Matrix keeping track of G-scores 
    FScore = single(inf(Height, Width)); % Matrix keeping track of F-scores (only open list) 
    Hn = single(zeros(Height, Width)); % Heuristic matrix
    OpenMAT = int8(zeros(Height, Width)); % Matrix keeping of open grid cells
    ClosedMAT = int8(zeros(Height, Width)); % Matrix keeping track of closed grid cells
    ClosedMAT(MAP == 1) = 1; % Adding object-cells to closed matrix
    ParentX = int16(zeros(Height, Width)); % Matrix keeping track of X position of parent
    ParentY = int16(zeros(Height, Width)); % Matrix keeping track of Y position of parent
    
    %% Setting up matrices representing neighboors to be investigated

    NeighboorCheck = ones(2 * Connecting_Distance + 1);
    Dummy = 2 * Connecting_Distance + 2;
    Mid = Connecting_Distance + 1;
    
    for i = 1:Connecting_Distance - 1
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
    Neighboors = [row col] - (Connecting_Distance + 1);
    N_Neighboors = size(col, 1);
    
    %% Creating Heuristic-matrix based on distance to nearest goal node

    [col, row] = find(GoalRegister == 1);
    RegisteredGoals = [row col];
    Nodesfound = size(RegisteredGoals, 1);

    for k = 1:size(GoalRegister, 1)
        for j = 1:size(GoalRegister, 2)
            if MAP(k, j) == 0
                Mat = RegisteredGoals - (repmat([j k], (Nodesfound), 1));
                Distance = (min(sqrt(sum(abs(Mat).^2, 2))));
                Hn(k, j) = Distance;
            end
        end
    end

    %% Initializign start node with FValue and opening first node.
    FScore(StartY, StartX) = Hn(StartY, StartX);         
    OpenMAT(StartY, StartX) = 1;   
    
    while 1 == 1
        MINopenFSCORE = min(min(FScore));
        
        if MINopenFSCORE == inf
            OptimalPath = [inf];
            RECONSTRUCTPATH = 0;
            
            break
        end
        
        [CurrentY, CurrentX] = find(FScore == MINopenFSCORE);
        CurrentY = CurrentY(1);
        CurrentX = CurrentX(1);

        if GoalRegister(CurrentY, CurrentX) == 1
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
                
                        if MAP(CurrentY + YPOS, CurrentX + XPOS) == 1
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
        OptimalPath(1, :) = [CurrentY CurrentX];

        while RECONSTRUCTPATH
            CurrentXDummy = ParentX(CurrentY, CurrentX);
            CurrentY = ParentY(CurrentY, CurrentX);
            CurrentX = CurrentXDummy;
            OptimalPath(k, :) = [CurrentY CurrentX];

            k = k + 1;

            if (CurrentX == StartX) && (CurrentY == StartY)
                break
            end
        end
    end
end