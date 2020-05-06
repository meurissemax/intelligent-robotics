% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function map(occMat, varargin)
    % Returns a boolean indicating if the map is explored.
    %
    % 'occMat' is a ternary occupancy matrix
    % 'varargin' can contains :
    %   - 'pos' is the [x, y] position of the robot in 'occMat'
    %   - 'pathDisp' : the path of the robot

    % Visited free points
    [i_free, j_free] = find(occMat == 0);
    [x_free, y_free] = utils.tocart(i_free, j_free, size(occMat, 1));
    plot(x_free, y_free, '.g', 'MarkerSize', 10);
    hold on;

    % Visited occupied points
    [i_occ, j_occ] = find(occMat == 1);
    [x_occ, y_occ] = utils.tocart(i_occ, j_occ, size(occMat, 1));
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
            pathConverted = [];

            for i = 1:size(pathDisp, 1)
                [x, y] = utils.tocart(pathDisp(i, 1), pathDisp(i, 2), size(occMat, 1));
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

    % Others
    hold off;

    title('Explored map');

    drawnow;
end
