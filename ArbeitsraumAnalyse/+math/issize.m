function flag = issize(A, r, c)
% ISSIZE - Check whether the given matrix is of provided size/dimensions
% 
%   FLAG = ISSIZE(A, r, c) checks matrix A is of dimensions r x c
%
% 
%   Inputs:
% 
%   A: matrix to check for squareness
%   
%   R: rows matrix A has to have. Can be empty to just check for the columns
%   couunt to match
%   
%   C: number of columns matrix A has to have. Can be empty to just check for
%   the rows count to match
% 
%   Outputs:
% 
%   FLAG: evaluates to true if A is of size r x c, otherwise false
% 



%% File Information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-12-23
% Changelog:
%   2016-12-23
%       * Add narginchk and nargoutchk
%   2016-05-10
%       * Add general `File Information` block
%       * Update inline documentation
%   2015-06-18
%       * Update to allow for empty arguments so that we can just check the
%       columns or rows count
%   2015-04-24
%       * Update validation function to use ```isequaln``` rather than
%       ```isequal``` (slight improvement of readability and speed)
%   2015-04-22
%       * Initial release



%% Assert arguments
narginchk(2, 2);
nargoutchk(0, 1);



%% Checking
% If no row and no column was given to check against...
if isempty(r) && isempty(c)
    flag = ismatrix(A) && isequal(size(A, 2), c);
% Just check for a row count
elseif ~isempty(r) && isempty(c)
    flag = ismatrix(A) && isequal(size(A, 1), r);
% Check for both row and column count
else
    flag = ismatrix(A) && isequaln(size(A, 1), [r, c]);
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changelog" section of the header
