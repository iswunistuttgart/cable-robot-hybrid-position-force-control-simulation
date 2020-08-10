function vec = evecn(Coordinate, Dimension)%#codegen

% Make sure we do not create vectors larger than the specified dimension by
% providing a coordinate bigger than dimension
Coordinate = min(Coordinate, Dimension);

% Create an empty vector...
vec = zeros(Dimension, 1);
% ... and set the Coordinate-th value to 1
vec(Coordinate) = 1;


end