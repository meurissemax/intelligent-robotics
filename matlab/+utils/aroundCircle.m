% INFO0948-2 - Introduction to intelligent robotics
% University of Liege - Academic year 2019-2020
% Authors : Maxime Meurisse & Valentin Vermeylen

function points = aroundCircle(center, radius, number, prec)
    % Generate 'number' points equally spaced around a
    % circle centered at 'center' position and with a
    % radius of 'radius'.

    % Increase radius (to be sure to be accessible)
    radius = (5 / 4) * radius;

    % Initialize array of points
    points = zeros(number, 2);

    % Calculte the angle increment
    dx = (2 * pi) / number;

    % Initialize the angle
    ang = 0;

    % Generate each point
    for i = 1:number
        points(i, 1) = center(1) + radius * cos(ang);
        points(i, 2) = center(2) + radius * sin(ang);

        points(i, :) = round(points(i, :) .* prec) ./ prec;

        ang = ang + dx;
    end
end
