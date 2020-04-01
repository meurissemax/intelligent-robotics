% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function next = findNext(pos, occMat, threshDist, metric)
    % Returns the next inexplored point in 'occMat'
    % from 'pos'.
    %
    % 'pos' is a position (i, j) in 'occMat'
    % 'occMat' is a ternary occupancy matrix
    % 'threshDist' is a threshold on the distance
    % 'metric' is a metric to find the point (longest or shortest)

    % We get the occupancy matrix size
    sizeX = size(occMat, 1);
    sizeY = size(occMat, 2);

    % We initialize the distance
    if strcmp(metric, 'longest')
        mDist = -Inf;
    else
        mDist = Inf;
    end

    % We iterate over each point of the occupancy matrix
    for i = 1:sizeX
        for j = 1:sizeY

            % Possible candidate for closest point are inexplored points
            if occMat(i, j) == -1
                hasFreeNeighbor = false;
                hasOccNeighbor = false;

                % We check if there is (at least) an explored and free point
                % in the neighborhood of the possible candidate
                for x = i - 1:1:i + 1
                    for y = j - 1:1:j + 1
                        if x >= 1 && y >= 1 && x <= sizeX && y <= sizeY && (x ~= i || y ~= j)
                            if occMat(x, y) == 0
                                hasFreeNeighbor = true;
                            elseif occMat(x, y) == 1
                                hasOccNeighbor = true;
                            end
                        end
                    end
                end

                % If the inexplored point is a valid possible candidate
                if hasFreeNeighbor && ~hasOccNeighbor
                    d = pdist2([i, j], pos, 'euclidean');

                    if (strcmp(metric, 'longest') && d > mDist) || (strcmp(metric, 'shortest') && d < mDist)
                        if d >= threshDist
                            mDist = d;
                            next = [i, j];
                        end
                    end
                end
            end
        end
    end

    % If no point has been find
    if mDist == Inf || mDist == -Inf
        next = Inf;
    end
end
