% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function ij = toMatrix(xy, height)
	% Returns the coordinates in a matrix convention.
	%
	% 'x' the first coordinate in cartesian convention
	% 'y' the second coordinate in cartesian convention
    % 'height' the height (number of lines) of the matrix

    ij(1) = height - (xy(:, 2) - 1);
    ij(2) = xy(:, 1);
end
