% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function optiPath = optimizePath(path, optiDist)
    % Returns an optimization of the path 'path'.
    %
    % 'path' is a path
    % 'optiDist' is a threshold for the distance

    lastX = path(1, 1);
    lastY = path(1, 2);

    % We put the first point in the optimized path
    optiPath = [lastX, lastY];
    count = 2;

    keep = false;

    % We iterate over the path
    for i = 2:size(path, 1)
        x = path(i, 1);
        y = path(i, 2);

        % If consecutive points are aligned
        if x == lastX || y == lastY
            if pdist2([x, y], [lastX, lastY]) >= optiDist
                keep = true;
            end
        else
            keep = true;
        end

        % If we have to keep the point
        if keep
            optiPath(count, :) = [x, y];
            count = count + 1;

            lastX = x;
            lastY = y;

            keep = false;
        end
    end
end
