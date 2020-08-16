% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function closest = findClosest(from, points)
    % Find the closest point of the 'points' list
    % to the point 'from' and return it.

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
