function Mn = mnormrow(M)%#codegen
% MNORMROW Normalize a matrix per row
% 
%   MN = MNORMROW(M) normalizes each row of matrix MAT by its norm.
%
%   
%   Inputs:
%   
%   M: Matrix of variable dimension to be normalized.
%
%   Outputs:
%
%   MN: Matrix with each row's norm being one.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2017-04-14
% Changelog:
%   2017-04-14
%       * Change assertion from ```assert``` to ```validateattributes```
%   2016-04-04
%       * Initial release



%% Assert arguments
narginchk(1, 1);
nargoutchk(0, 1);

validateattributes(M, {'numeric'}, {'2d', 'nonempty', 'finite'}, mfilename, 'M');



%% Magic, do your thing and create the output right away
Mn = bsxfun(@times, M, 1./sqrt(sum(M.^2,2)));


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
