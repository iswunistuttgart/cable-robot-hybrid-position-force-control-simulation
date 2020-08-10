function flag = ismeshgrid(grid)
% ISMESHGRID determines whether grid might come from meshgrid or not
%
%   Inputs:
%
%   GRID                Arbitrary MxN matrix
%
%   Outputs:
%
%   FLAG                True of GRID seems to be coming from MESHGRID, otherwise
%       false.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-12-23
% Changelog:
%   2016-12-23
%       * Initial release



%% Do your code magic here

% Find first non-singleton dimension
grid = shiftdata(grid, []);

flag = ismonotonic(grid(1,:)) || ismonotonic(grid(:,1));

% if any(size(grid) == 1)
%     dX = gradient(grid);
%     dY = 0;
% else
%     [dX, dY] = gradient(grid);
% end
% 
% 
% if all(dX(:)) || all(dY(:))
%     flag = 1;
% else
%     flag = 0;
% end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
