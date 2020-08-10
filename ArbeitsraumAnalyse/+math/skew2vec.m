function v = skew2vec(S)
%% SKEW2VEC Convert skew-symmetric matrix into skew-symmetric vector representation
%
%   V = SKEW2VEC(S) converts skew-symmetric matrix S into its skeww-symmetric
%   vector form such.
%
%   Inputs:
%
%   S                   3x3xN matrix of skew-symmetric matrices.
%
%   Outputs:
%
%   V                   Nx3 vector of skew-symmetric vectors.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2019-01-09
% Changelog:
%   2019-01-09
%       * Initial release



%% Validate arguments

narginchk(1, 1);
nargoutchk(0, 1);

validateattributes(S, {'numeric', 'symbolic', 'sym'}, {'nonempty', '3d', 'nrows', 3, 'ncols', 3}, mfilename, 'S');



%% Process

% Shift into the right dimension
S = permute(S, [3, 1, 2]);

% Simple concatenationm
v = cat(2, S(:,3,2), S(:,1,3), S(:,2,1));


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
