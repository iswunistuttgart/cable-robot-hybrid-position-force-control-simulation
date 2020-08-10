function flag = isint(x)
% ISINT checks the given value to be of natural numbers
%
%   ISINT(X) returns true where X is a value of natural numbers.
%
%   Inputs:
%
%   X               Anything to be checked.
%
%   Outputs:
%
%   FLAG            TRUE where X is int and FALSE otherwise.



%% File information
% Author: Peter J. Acklam <pjacklam@online.no>
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-12-23
% Changelog:
%   2016-12-23
%       * Add narginchk and nargoutchk
%   2016-12-04
%       * Merge with Peter J. Acklam's isint implementation which supports
%       different types of data
%   2016-09-22
%       * Remove `all` from the checks so that it can be used to identify values
%       of an array that are int and ones that are not
%   2016-06-14
%       * Initial release



%% Assert arguments
narginchk(1, 1);
nargoutchk(0, 1);



%% Magic, do your thing

% class of input argument
chClassX = class(x);

if isempty(x)
    % return empty array of same class
    flag = feval(chClassX, x);
else
    switch chClassX
        case 'double'
            flag = x == round(x);
        case 'single'
            % "mod" is not defined for class "single"; so convert input to
            % double, compare, and convert back
            d = double(x);
            flag = single(d == round(d));
        case {'int8', 'int16', 'int32', 'int64', ...
            'uint8', 'uint16', 'uint32', 'uint64'}
            % Return an array of ones of the same class as input argument
            flag = logical(repmat(feval(chClassX, 1), size(x)));
        otherwise
            error('PHILIPPTEMPEL:MATLABTOOLING:MAT:ISINT:InvalidArgument', 'Argument is of unrecognized class %s.', chClassX);
    end
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
