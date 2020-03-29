% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function is_explored = is_map_explored(occMat, mapSize, mapPrec)
    % Returns a boolean indicating if the map is explored.
    %
    % 'occMat' is a ternary occupancy matrix
    % 'mapSize' is the (w, h) dimension of the map
    % 'mapPrec' is the precision of the 'occMat'

    % By default, map is not yet fully explored
    is_explored = false;

    % Number of points representing the map in the occupancy matrix
    full_pts = mapSize(1) * mapPrec * mapSize(2) * mapPrec;

    % Percentage of points that we want to have discover to consider
    % that the map is fully explored
    threshold = 0.9;

    % Count the number of points discovered (free and obstacle)
    free_pts = nnz(occMat == 0);
    occ_pts = nnz(occMat == 1);

    total_pts = free_pts + occ_pts;

    % Get the percentage of points discovered
    p = total_pts / full_pts;

    % Evaluate if the map is considered as fully explored
    if p >= threshold
        is_explored = true;
    end
end
