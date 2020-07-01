function [Distribution] = closed_form(Wrench, StructureMatrix, ForceMinimum, ForceMaximum)%#codegen
% CLOSED_FORM - Determine the force distribution for the given robot using the
% closed-form force distribution algorithm
% 
%   DISTRIBUTION = CLOSED_FORM(WRENCH, STRUCTUREMATRIX, FORCEMINIMUM,
%   FORCEMAXIMUM) calculates the closed-form force distribution for the given
%   wrench and the pre-calculated structure matrix.
%   
%   Inputs:
%   
%   WRENCH:             Column-vector of the wrench on the system. Preferably
%       should be a 6x1 vector, but if you know what you are doing, it might
%       work with other dimensions, too. Generally, 6x1 vectors will work fine,
%       too, as long as you adjust it properly to your cable robot design.
%   
%   STRUCTUREMATRIX:    The structure matrix At, which must be calculated
%       beforehand, for which to determine the closed-form force distribution as
%       given by A. Pott.
%   
%   FORCEMINIMUM:       Minimum force as required for the algorithm to work.
%       Must be either a scalar which is then being translated as the minimum
%       for all cables, or a column vector that has the same number of rows as
%       STRUCTUREMATRIX has columns.
%   
%   FORCEMAXIMUM:       Maximum force as required for the algorithm to work.
%       Must be either a scalar which is then being translated as the minimum
%       for all cables, or a column vector that has the same number of rows as
%       STRUCTUREMATRIX has columns.
% 
%   Outputs:
% 
%   DISTRIBUTION:       Vector of force distribution values as determined by the
%       algorithm.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-11-23
% Changelog:
%   2016-11-23
%       * Fix attribute validation
%   2016-10-08
%       * Move into package '+fdist'
%       * Add `narginchk`, `nargoutchk` for better standalone usage
%   2016-09-19
%       * Rename to `closed_form`
%   2016-03-29
%       * Code cleanup
%   2015-06-12
%       * Make sure that vForce{Max,Min}imum are column vectors
%       * Fix typos in comments and variable names
%       * Finally write method help documentation
%   2015-04-22
%       * Initial release



%% Argument processing
% Four input arguments
narginchk(4, 4);
% Zero to one output argument
nargoutchk(0, 1);

% Assertion of arguments
validateattributes(Wrench, {'numeric'}, {'vector', 'numel', 2, 'finite'}, mfilename, 'Wrench', 1);
validateattributes(StructureMatrix, {'numeric'}, {'2d', 'finite', 'nonnan'}, mfilename, 'StructureMatrix', 2);
validateattributes(ForceMinimum, {'numeric'}, {'vector', 'positive', 'finite'}, mfilename, 'ForceMinimum', 3);
validateattributes(ForceMaximum, {'numeric'}, {'vector', 'positive', 'finite'}, mfilename, 'ForceMaximum', 4);




%% Assert and parse variables
% Wrench
vWrench = Wrench;
% Structure matrix to determine force distribution for
aStructureMatrixAt = StructureMatrix;
% Number of cables (is being used quite often in the following code)
nNumberOfWires = size(aStructureMatrixAt, 2);

% Force minimum, can be given a scalar or a vector
if isscalar(ForceMinimum)
    vForceMinimum = ForceMinimum.*ones(nNumberOfWires, 1);
else
    vForceMinimum = ForceMinimum(:);
end
% Force maximum, can be given a scalar or a vector
if isscalar(ForceMaximum)
    vForceMaximum = ForceMaximum.*ones(nNumberOfWires, 1);
else
    vForceMaximum = ForceMaximum(:);
end
% Vector of mean force values
vForceMean = 0.5.*(vForceMinimum + vForceMaximum);



%% Do the magic
% Simple case where the number of wires matches the number of degrees of
% freedom, we can just solve the linear equation system At*f = -w;
% if issquare(aStructureMatrixAt)
%     aForceDistribution = aStructureMatrixAt\(-vWrench);
% % Non standard case, where we have more cables than degrees of freedom
% else
    % Solve A^t f_v = - ( w + A^t f_m )
    % Determine the pseudo inverse of A^t
    %aStructureMatrixPseudoInverse = transpose(aStructureMatrixAt)/(aStructureMatrixAt*transpose(aStructureMatrixAt));
    aStructureMatrixPseudoInverse = pinv(aStructureMatrixAt);
    % And determine the force distribution
    aForceDistribution = vForceMean - aStructureMatrixPseudoInverse*(vWrench + aStructureMatrixAt*vForceMean);
% end



%% Create output quantities
Distribution = aForceDistribution;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
