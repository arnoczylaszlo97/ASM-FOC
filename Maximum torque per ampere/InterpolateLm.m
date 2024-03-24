% This function shall interpolate the magnetizing inductance - phase current pairs.

% The point of the X-axis, we want to interpolate.
% linspace( min_value, max_value, number of points between min max)

% The interpolation.
% interp1(x,y,xq);
% x  - contains the sample points
% y  - contains the corresponding values, y(x)
% xq - contains the coordinates of the query points (linspace()).
% https://www.mathworks.com/help/matlab/ref/interp1.html#btwp6lt-1-method

function [x_course, y_course ] = InterpolateLm(Iph_array,Lm_array)
    x_course = linspace(min(Iph_array), max(Iph_array), 100000);
    y_course = interp1(Iph_array, Lm_array, x_course);
end