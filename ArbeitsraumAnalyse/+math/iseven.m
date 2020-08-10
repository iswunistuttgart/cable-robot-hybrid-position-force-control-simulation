function flag = iseven(x)
% ISEVEN checks the given number(s) for being even
%
%   ISEVEN(X) returns true, if the number NUMBER are even i.e., dividable by 2.
%
%   FLAG = ISEVEN(X) returns the flag.
%
%   Input:
%
%   X               Nx1 array to check for being even.
%
%   Outputs:
%
%   FLAG            Logical flag whether NUMBER is even (FLAG==1) or odd
%       (FLAG==0).



%% File information
% Author: Peter J. Acklam <pjacklam@online.no>
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-12-04
% Changelog:
%   2016-12-04
%       * Merge with Peter J. Acklam's implementation supporting different
%       classes for X
%   2016-09-19
%       * Initial release



%% Do your code magic here
if ~isnumeric(x)
    error('Argument must be a numeric array.');
end

% class of input argument
chClassX = class(x);

if isempty(x)
    % Return empty array of same class
    flag = feval(chClassX, x);
else
    switch chClassX
        case 'double'
            flag = ~mod(x, 2);
        case 'single'
            % "mod" is not defined for class "single"; so convert input to double, compare,
            % and convert back
            flag = single(~mod(double(x), 2));
        case {'uint8', 'uint16', 'uint32', 'uint64'}
            flag = ~bitand(x, 1);
        case {'int8', 'int16', 'int32', 'int64'}
            error('PHILIPPTEMPEL:MATLABTOOLING:MAT:ISEVEN:InvalidArgument', 'Not implemented for classes int8, int16, int32, and int64.');
        otherwise
            error('PHILIPPTEMPEL:MATLABTOOLING:MAT:ISEVEN:InvalidArgument', 'Argument is of unrecognized class %s.', chClassX);
    end
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
