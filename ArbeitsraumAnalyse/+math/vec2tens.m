function [I] = vec2tens(v)
% VEC2TENS converts a vector to its 2d tensor matrix representation.
%
%   Inputs:
%
%   V       Mx6 Vector of tensor components ordered by [I_11, I_22, I_33, I_12,
%           I_23, I_13]. For each row of v, a tensor matrix will be generated.
%
%   Outputs:
%
%   I       Tensor matrix of vector T as 3x3xM matrix.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-08-25
% Changelog:
%   2016-08-25
%       * Initial release



%% Assertion
% Vector must be numeric or symbolic
assert(isa(v, 'double') || isa(v, 'sym'), 'Input must be double or symbolic.');
% Vector must have 6 columns (we'll allow multiple rows to be converted to
% tensor matrices).
assert(size(v, 2) == 6, 'PHILIPPTEMPEL:VEC2TENS:invalidColCoun', 'Input [v] must have 6 columns');



%% Do your code magic here

% Get all vectors for wich to get the skew-symmetric matrix
aVectors = transpose(v);
% Number of vectors == rows of aVectors
nVectors = size(aVectors, 2);
% Reshape the vector in the depth dimension
aVectors2 = reshape(aVectors, [6, 1, nVectors]);

% Quicker access to important parts
I_xx = aVectors2(1,1,:);
I_yy = aVectors2(2,1,:);
I_zz = aVectors2(3,1,:);
I_xy = aVectors2(4,1,:);
I_yz = aVectors2(5,1,:);
I_xz = aVectors2(6,1,:);



%% Magic (copied from quat2rotm)

% Explicitly define concatenation dimension for codegen
tempI = cat(1, I_xx, I_xy, I_xz, I_xy, I_yy, I_yz, I_xz, I_yz, I_zz);
I = reshape(tempI, [3, 3, nVectors]);
I = permute(I, [2, 1, 3]);




end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header
