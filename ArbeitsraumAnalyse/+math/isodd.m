function flag = isodd(x)
% ISODD checks the given number(s) for being odd
%
%   ISODD(X) returns true, if the number NUMBER is odd i.e., not dividable by 2.
%
%   FLAG = ISODD(X) returns the flag.
%
%   Input:
%
%   X               Nx1 array to check for being odd.
%
%   Outputs:
%
%   FLAG            Logical flag whether NUMBER is odd (FLAG==1) or even
%       (FLAG==0).



%% File information
% Author: Peter J. Acklam <pjacklam@online.no>
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2018-03-10
% Changelog:
%   2018-03-10
%       * Add missing line to determine class of given variable
%   2016-12-23
%       * Change from nargchk to narginchk
%   2016-12-04
%       * Merge with Peter J. Acklam's implementation supporting different
%       classes for X
%   2016-09-19
%       * Initial release



%% Assert arguments
narginchk(1, 1);
nargoutchk(0, 1);



%% Do your code magic here
if ~isnumeric(x)
    error('Argument must be a numeric array.');
end

% class of input argument
chClass = class(x);

% Return empty array of same class
if isempty(x)
    flag = feval(chClass, x);
else
    switch chClass
        case 'double'
            flag = mod(x, 2) == 1;
        case 'single'
            % "mod" is not defined for class "single"; so convert input to
            % double, compare, and convert back
            flag = single(mod(double(x), 2) == 1);
        case {'uint8', 'uint16', 'uint32', 'uint64'}
            flag = bitand(x, 1);
        case {'int8', 'int16', 'int32', 'int64'}
            error('PHILIPPTEMPEL:MATLABTOOLING:MAT:ISODD:InvalidArgument', 'Not implemented for classes int8, int16, int32, and int64.');
        otherwise
            error('PHILIPPTEMPEL:MATLABTOOLING:MAT:ISODD:InvalidArgument', 'Argument is of unrecognized class.');
    end
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
