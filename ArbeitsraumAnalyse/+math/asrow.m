function Y = asrow(X, varargin)%#codegen
% ASROW ensures the vector is a row vector
% 
%   ASROW(X) returns the elements of X as a row vector, running through the
%   elements X in column major order.
%
%   ASROW(X, ORDER) will run through the dimensions of X in the order specified
%   in ORDER.  Unspecified dimensions will be run through in increasing order
%   after the specified dimensions.  For instance, if X is 4-D, then ASROW(X, [2
%   3]) = ASROW(X, [2 3 1 4]).
%
%   For example, if
%
%      X = [4 6 8   then  ASROW(X)    = [4 5 6 7 8 9]
%           5 7 9]        ASROW(X, 2) = [4 6 8 5 7 9]
%
%   where ASROW(X) = ASROW(X, 1) = ASROW(X, [1 2]) and
%   ASROW(X, 2) = ASROW(X, [2 1]).
%
%   Inputs:
% 
%   X               An arbitrary array to be sorted as row vector Y.
%
%   ORDER           Specifies the order in which dimensions should be sorted
%       into row vector Y.
% 
%   Outputs:
% 
%   ROW             Sorted row vector
%
%   See also: ASCOL, ASVEC, RESHAPE, PERMUTE.



%% File information
% Author: Peter J. Acklam <pjacklam@online.no>
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-12-03
% Changelog:
%   2016-12-03
%       * After adding `asvec` make this function use that function
%       * Add Peter J. Acklam as additional (first) author
%   2016-05-10
%       * Add check with isrow so we do not loose too much time in case we are
%       already already dealing with a row vector
%       * Add section `File Information`
%       * Add help section
%   2016-05-02
%       * Initial release



%% Reshaping
narginchk(1, 2);

Y = asvec(X, 2, varargin{:});


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
