function Q = cart2quadrant(X, Y)
% CART2QUADRANT returns the quadrant of the given Cartesian coordinates
%
%   CART2QUADRANT(X, Y) returns
%       1 if    X > 0 & Y > 0
%       2 if    X < 0 & Y > 0
%       3 if    X < 0 & Y < 0
%       4 if    X > 0 & Y < 0
%       0       else
%   for any value inside the corresponding (i,j) pair of (X,Y).
%
%   Inputs:
%
%   X                   NxM array of Cartesian X-coordinates.
%
%   Y                   NxM array of Cartesian Y-coordinates.
%
%   Outputs:
%
%   Q                   NxM array of quadrant number of the given coordinates.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2018-08-01
% Changelog:
%   2018-08-01
%       * Initial release



%% Assert arguments
try
    % CART2QUADRANT(X, Y)
    narginchk(2, 2);
    
    % CART2QUADRANT(X, Y)
    % Q = CART2QUADRANT(X, Y)
    nargoutchk(0, 1);
    
    validateattributes(X, {'numeric'}, {'2d', 'nonempty', 'finite', 'nonnan', 'nonsparse'}, mfilename, 'X');
    validateattributes(Y, {'numeric'}, {'2d', 'nonempty', 'size', size(X), 'finite', 'nonnan', 'nonsparse'}, mfilename, 'Y');
catch me
    throwAsCaller(me);
end



%% Initialization
% Quadrants look up
persistent vQuadsLookup
if isempty(vQuadsLookup)
    vQuadsLookup = [3, 2, 4, 1];
end



%% Conversion
% We'll make things less computationally heavy and just the following SO post
% @see https://stackoverflow.com/a/9728189/4065558

% Get the signs of both X and Y coordinates
vNonzero_X = ( X >= 0 );
vNonzero_Y = ( Y >= 0 );

% Calculate the quadrant and convert it to unsigned integers (uint8)
uiQuadrants = uint8( ( 3 + vNonzero_X - vNonzero_Y - 2.*vNonzero_X.*vNonzero_Y ).*( ( X ~= 0 ) & ( Y ~= 0 ) ) );



%% Assign output quantities
% Q = CART2QUADRANT(X, Y);
Q = uiQuadrants;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
