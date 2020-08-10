function ord = ord(N)
% ORD returns the ordinal for the given number
%
%   Inputs:
%
%   N                   NxM array of numbers to determine ordinal for.
%
%   Outputs:
%
%   TH                  NxM cell array of ordinal for each number.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2017-08-25
% Changelog:
%   2017-08-25
%       * Initial release



%% Check arguments
narginchk(1, 1);
nargoutchk(0, 1);

try
    validateattributes(N, {'numeric'}, {'nonempty', '2d', 'finite', 'nonnan', 'nonsparse'}, mfilename, 'N');
catch me
    throwAsCaller(me);
end



%% Do your code magic here

% This is the ordinals for numbers ending in [1, 2, 3, 4..0];
ceOrdinals = {'st', 'nd', 'rd', 'th'};

% Get the last digit of each number
aLastDigit = rem(N, 10);

% Set everything with a last digit larger than 4 to end with 'th' which is 4 in
% ceOrdinals
aLastDigit(abs(aLastDigit) > 4) = 4;
% Also, any number ending on 0 is appended with 'th' which is the fourth entry
% in ceOrdinals
aLastDigit(aLastDigit == 0) = 4;

% Now, replace each entry of the last digits with its corresponding ordinal
% value
ord = arrayfun(@(i) ceOrdinals{i}, aLastDigit, 'UniformOutput', false);

% If only one input was given, we should be returning only a char array not a
% cell array for quicker use
if numel(N) == 1
    ord = ord{1};
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
