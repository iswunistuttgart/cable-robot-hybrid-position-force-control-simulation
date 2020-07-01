function [Distribution] = advanced_closed_form(Wrench, StructureMatrix, ForceMinimum, ForceMaximum) %#codegen
% ADVANCED_CLOSED_FORM - Determine the force distribution for the given
% robot using the closed-form force distribution algorithm
% 
%   DISTRIBUTION = ADVANCED_CLOSED_FORM(WRENCH, STRUCTUREMATRIX, FORCEMINIMUM,
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
%   DISTRIBUTION:       Vector of force distribution values as determined by the
%       algorithm.



%% File Information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-11-23
% Changelog:
%   2016-11-23
%       * Fix argument validation
%       * Fix incorrect call to related fdist.closed_form function
%   2016-10-08
%       * Move into package '+fdist'
%       * Add `narginchk`, `nargoutchk` for better standalone usage
%   2016-09-19
%       * Rename to `advanced_closed_form`
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
    vForceMinimum = reshape(ForceMinimum, nNumberOfWires, 1);
end
% Force maximum, can be given a scalar or a vector
if isscalar(ForceMaximum)
    vForceMaximum = ForceMaximum.*ones(nNumberOfWires, 1);
else
    vForceMaximum = reshape(ForceMaximum, nNumberOfWires, 1);
end



%% Do the magic
% Simple case where the number of wires matches the number of degrees of
% freedom, we can just solve the linear equation system At*f = -w;
% if issquare(aStructureMatrixAt)
%     vForceDistribution = aStructureMatrixAt\(-vWrench);
% % Non standard case, where we have more cables than degrees of freedom
% else
    vForceDistribution = closed_form(vWrench, aStructureMatrixAt, vForceMinimum, vForceMaximum);
    
    % Keeps the index of the violated force value
    iViolationIndex = 0;
    % Violation of min = -1 or max = 1 force
    iViolationType = 0;
    % Keeps the violated amount
    dViolationAmount = 0;
    for iUnit = 1:nNumberOfWires
        if vForceDistribution(iUnit) < vForceMinimum(iUnit)
            iViolationIndex = iUnit;
            iViolationType = -1;
            dViolationAmount = max(dViolationAmount, vForceMinimum(iUnit) - vForceDistribution(iUnit));
        elseif vForceDistribution(iUnit) > vForceMaximum(iUnit)
            iViolationIndex = iUnit;
            iViolationType = 1;
            dViolationAmount = max(dViolationAmount, vForceDistribution(iUnit) - vForceMaximum(iUnit));
        end
    end
    
    % Found a violation of forces?
    if iViolationIndex > 0
        % Get reduced maximum and minimum forces
        vReducedForceMinimum = zeros(nNumberOfWires - 1, 1);
        vReducedForceMaximum = zeros(nNumberOfWires - 1, 1);
        % Get reduced structure matrix
        aReducedStructureMatrixAt = zeros(size(aStructureMatrixAt, 1), size(aStructureMatrixAt, 2) - 1);
        
        iReducedUnit = 1;
        % Loop over all cables
        for iUnit = 1:nNumberOfWires
            % Skip the limits violating cable
            if iUnit == iViolationIndex || iUnit + nNumberOfWires == iViolationIndex
                continue
            end
            
            % Reduce the minimum and maximum cable forces
            vReducedForceMinimum(iReducedUnit) = vForceMinimum(iUnit);
            vReducedForceMaximum(iReducedUnit) = vForceMaximum(iUnit);
            % And also reduce the structure matrix omitting the violated
            % cable's column
            aReducedStructureMatrixAt(:,iReducedUnit) = aStructureMatrixAt(:,iUnit);
            
            % Counter so we know what our 
            iReducedUnit = iReducedUnit + 1;
        end
        
        % Copy the original wrench so we can alter it to the "reduced"
        % wrench
        vReducedWrench = vWrench;
        % Modify external wrench by the violated force, either the maximum
        if iViolationType == 1
            vReducedWrench = vReducedWrench + aStructureMatrixAt(:,iViolationIndex)*vForceMaximum(iUnit);
        % or minimum force
        else
            vReducedWrench = vReducedWrench + aStructureMatrixAt(:,iViolationIndex)*vForceMinimum(iUnit);
        end
        
        % Recursively call the algorithm for advanced closed form, yet this
        % time with the reduced values
        vReducedForceDistribution = advanced_closed_form(vReducedWrench, aReducedStructureMatrixAt, vReducedForceMinimum, vReducedForceMaximum);
        
        % Restore initial cable force distribution from the reduced cable
        % forces as well as the violated cable force
        iReducedUnit = 1;
        for iUnit = 1:nNumberOfWires
            % Violated unit?
            if iUnit == iViolationIndex
                % Then take its maximum force if the maximum was violated
                if iViolationType == 1
                    vForceDistribution(iUnit) = vForceMaximum(iUnit);
                % or the minimum force, if the minimum was violated
                else
                    vForceDistribution(iUnit) = vForceMinimum(iUnit);
                end
                
                continue;
            end
            
            % Not the violated unit so we can just take the reduced force
            % distribution's value
            vForceDistribution(iUnit) = vReducedForceDistribution(iReducedUnit);
            iReducedUnit = iReducedUnit + 1;
        end
    end
%end



%% Create output quantities
Distribution = vForceDistribution;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
