% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function [x, y] = mat_to_cart(i, j, height)
    % Returns the coordinates in a cartesian convention.
    %
    % 'i' the first coordinate in matrix convention
    % 'j' the second coordinate in matrix convention
    % 'height' the height (number of lines) of the matrix

    x = j;
    y = height - i;
end
