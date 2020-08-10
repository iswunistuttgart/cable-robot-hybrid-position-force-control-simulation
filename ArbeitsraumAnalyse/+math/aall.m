function A = aall(V, dim)
% AALL is a wrapper over recursive calls to all(all(all(....)))
%
%   Inputs:
%
%   V                    Any type of value that is also accepted by `all(V)```
%
%   Outputs:
%
%   A                    Boolean flag, whether really all(all(all(...))) is
%       given or not.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-12-03
% Changelog:
%   2016-12-03
%       * Initial release



%% Assert arguments
% AALL(V) or AALL(V, DIM)
narginchk(1, 2);



%% Do your code magic here

% If no dimension is given, we will call ALL without dimension and let it decide
% what to do
if nargin < 2 || isempty(dim)
    A = all(V);
    
    while ~isscalar(A)
        A = all(A);
    end
% Called with dimension, so pass along to ALL
else
    A = all(V, dim);
    
    while ~isscalar(A)
        A = all(A, dim);
    end
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
