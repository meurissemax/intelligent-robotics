% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function optiPath = optimizePath(path)
    % Returns an optimization of the path 'path'.
    %
    % 'path' is a path

    lastX = path(1, 1);
    lastY = path(1, 2);

    optiPath = [lastX, lastY];
    count = 2;

    keep = false;

    minDist = 15;

    for i = 2:size(path, 1)
        x = path(i, 1);
        y = path(i, 2);

        if x == lastX || y == lastY
            if pdist2([x, y], [lastX, lastY]) >= minDist
                keep = true;
            end
        else
            keep = true;
        end

        if keep
            optiPath(count, :) = [x, y];
            count = count + 1;

            lastX = x;
            lastY = y;

            keep = false;
        end
    end
end
