% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function optiPath = optimize(path_list)
    % Returns an optimization of the path 'path_list'.
    %
    % 'path_list' is a path

    % We put the first point in the optimized path
    optiPath = [path_list(1, 1), path_list(1, 2)];
    count = 2;

    keep = false;

    % We check the size of the path
    if size(path_list, 1) < 3
        return;
    end

    % We iterate over the path
    for i = 3:size(path_list, 1)

        % We get last three points
        x = [path_list(i, 1), path_list(i - 1, 1), path_list(i - 2, 1)];
        y = [path_list(i, 2), path_list(i - 1, 2), path_list(i - 2, 2)];

        % We calculate the area
        a = polyarea(x, y);

        % We check if we have to keep the point
        if a > 0
            keep = true;
        end

        % We keep the point (if necessary)
        if keep
            optiPath(count, :) = [path_list(i - 1, 1), path_list(i - 1, 2)];
            count = count + 1;

            keep = false;
        end
    end
end
