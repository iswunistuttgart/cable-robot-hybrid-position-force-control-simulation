function Rdiff = rotmdiff(Rot1, Rot2)%#codegen
% ROTMDIFF determine the difference between two rotation matrices
% 
%   RDIFF = ROTMDIFF(ROT1, ROT2) calculates the difference between ROT2 and ROT1
%   and returns this value
% 
%   Inputs:
% 
%   ROT1: 3x3 rotation matrix to compare ROT2 against.
%
%   ROT2: 3x3 rotation matrix to compare ROT1 against.
% 
%   Outputs:
% 
%   RDIFF: Difference from ROT2 to ROT1. Note that this algorithms sets values
%   of the difference smaller than 2*eps to zero to vanish all remains of
%   numerical errors
%



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-05-02
% Changelog:
%   2016-05-02
%       * Initial release



%% Calculation
% Determine difference
Rdiff = Rot2 - Rot1;

% Set everything smaller than two times the machine constant to zero just to
% vanish all numerical errors
Rdiff(abs(Rdiff) < 2*eps) = 0;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
