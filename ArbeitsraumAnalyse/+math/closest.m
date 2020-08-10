function [i, cv] = closest(x, v)
% CLOSEST finds the row and column of the matrix element closest to a given
% value
%
%   [ROW, COL] = CLOSEST(MATRIX, VAL) finds the row and column index of a value
%   inside matrix MATRIX that is closest to value VAL.
%
%   Inputs:
%
%   MATRIX      Matrix or vector to extract value from.
%
%   VALUE       Value to find closest matrix element to. Can also be 
%
%   Outputs:
%
%   ROW         Row in which the closest value was found.
%
%   COL         Column in which the closes value was found.



%% File information
% Author: Dr. Murtaza Khan <drkhanmurtaza@gmail.com>
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-09-13
% Changelog:
%   2016-09-13
%       * Update to using binary search and then linear search
%   2016-09-02
%       * Initial release



%% Do your code magic here
% Stores the index of the closest value of x
idxClosest = [];

% Simple search first: v is larger or smaller than any value of x
if all(x < v)
    idxClosest = numel(x);
elseif all(v < x)
    idxClosest = 1;
end

if isempty(idxClosest)
    idxLeft = 1;
    idxRight = length(x);

    % % Phase 1: Binary Search
    while idxLeft <= idxRight
        idxMid = round((idxLeft + idxRight)/2);    
        dDifference = x(idxMid) - v;
        % Cannot check for diff = 0, but will make use of the machine constant
        % to compare both values
        if abs(dDifference) < 2*eps
            idxClosest = idxMid;
            dClosestValue = v;
            
            break
        elseif dDifference < 0     % x(mid) < v
            idxLeft = idxMid + 1;
        else              % x(mid) > v
            idxRight = idxMid - 1;			
        end
    end
    
    if isempty(idxClosest)
        % % Phase 2: Linear Search
        % % Remember Binary search could not find the value in x
        % % Therefore from > to. Search range is to:from
        y = x(idxRight:idxLeft);
        [~, mini] = min(abs(y - v));
        dClosestValue = y(mini);
        % % cv: closest value
        % % mini: local index of minium (closest) value with respect to y

        % % find global index of closest value with respect to x
        idxClosest = idxRight + mini - 1;
    end
else
    dClosestValue = x(idxClosest);
end



%% Assign output quantities
i = idxClosest;
cv = dClosestValue;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
