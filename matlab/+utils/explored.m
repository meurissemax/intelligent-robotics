% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function explored = explored(occMat, mapSize, mapPrec, explThresh)
    % Returns a boolean indicating if the map is explored.
    %
    % 'occMat' is a ternary occupancy matrix
    % 'mapSize' is the (w, h) dimension of the map
    % 'mapPrec' is the precision of the 'occMat'
    % 'explThresh' is the percentage of points that we want to have discover to consider that the map is fully explored

    % By default, map is not yet fully explored
    explored = false;

    % Number of points representing the map in the occupancy matrix
    fullPts = mapSize(1) * mapPrec * mapSize(2) * mapPrec;

    % Count the number of points discovered (free and obstacle)
    freePts = nnz(occMat == 0);
    occPts = nnz(occMat == 1);

    totalPts = freePts + occPts;

    % Get the percentage of points discovered
    p = totalPts / fullPts;

    % Evaluate if the map is considered as fully explored
    if p >= explThresh
        explored = true;
    end
end
